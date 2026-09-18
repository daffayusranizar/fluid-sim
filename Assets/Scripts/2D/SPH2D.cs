using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

/// <summary>
/// 2D weakly compressible SPH solver.
///
/// This MonoBehaviour owns state and orchestration only. Particle data lives in
/// NativeArrays so the Burst-compiled solver passes in SPHSolver can read it
/// directly, and the neighbour structure is built by SPHGrid.
/// </summary>
public class SPH2D : MonoBehaviour
{
    [Header("Container")]
    public Vector2 boxSize = new Vector2(20f, 20f);

    [Header("Particles")]
    [Tooltip("Total number of particles. Spawning is a blue-noise scatter, so " +
             "there is no grid to describe and no x-by-y split.")]
    public int particleCount = 5000;

    [Header("Physics")]
    public Vector2 gravity = new Vector2(0f, -9.81f);
    public float collisionDamping = 0.5f;

    // h is derived at spawn as h = smoothingLengthInSpacing * particle spacing.
    // Deriving it keeps the neighbour count in a sane range no matter how many
    // particles there are or how large the spawn region is.
    public float smoothingLengthInSpacing = 2.2f;
    [HideInInspector] public float smoothingLength = 2f;

    [Header("Simulation")]
    // 5000 particles in the default spawn region gives h = 0.152 and a CFL step of
    // 0.5-0.7 ms, so keeping up with real time needs ~33 substeps per 60 Hz frame.
    // The cap has to sit above that or the fluid silently falls behind.
    public int maxSubSteps = 64;
    [Range(0.05f, 0.5f)] public float cflFactor = 0.25f;
    public float minTimeStep = 0.0002f;
    public float maxTimeStep = 0.02f;

    [Tooltip("Run the Burst job solver in Update(). Turn this off when a GPU " +
             "simulation is driving the same state, so the two do not diverge.")]
    public bool solveOnCpu = true;

    [Tooltip("Particles per job batch. Lower gives a more even spread across " +
             "cores but adds scheduling overhead; raise it until gains flatten out.")]
    public int batchCount = 64;

    [Header("Spawn")]
    // Fraction of boxSize the fluid starts in, per axis. The classic dam break is
    // a tall column: narrow in x, full height in y.
    public Vector2 spawnRegionSize = new Vector2(0.3f, 0.8f);
    public Vector2 spawnRegionCenter = new Vector2(-1f, 0f);

    [Header("Density")]
    public float restDensity = 1000f;
    public bool autoParticleMass = true;
    public float spawnCompression = 1f;
    public float particleMass = 1f;

    [Header("Pressure")]
    public float stiffness = 2000f;

    // Off by default: with the near-pressure channel present, short-range
    // repulsion no longer depends on the main pressure being positive, so
    // negative pressure is safe and gives the two-way restoring force that a
    // real equilibrium needs. Turn this back on to compare.
    public bool clampPressurePositive = false;

    // Strength of the always-repulsive near channel. Scale dependent -- this
    // value is a starting point, tune over an order of magnitude either way.
    public float nearPressureMultiplier = 20f;

    public bool showPressureColor = true;

    [Header("Viscosity")]
    public float viscosity = 50f;

    [Header("Boundary")]
    public bool useBoundaryParticles = true;

    [Header("Debug Visualization")]
    public bool debugLogs = false;
    public bool showForceArrows = true;
    public bool showViscosityArrows = false;
    public float forceArrowLength = 0.4f;

    [Tooltip("Draw particles as gizmo spheres in the Scene view. This is the " +
             "fallback visualiser and stays on by default: turn it off once " +
             "ParticleRenderer2D's instanced discs are confirmed working, since " +
             "drawing both doubles the work.")]
    public bool showParticleGizmos = true;

    // --- Native simulation state ---
    // Structure of arrays, so each pass writes one array and reads others with no
    // overlap. See ParticleState for why that matters.
    private ParticleState state;

    // Packed Particle2D view for the renderer. Kept separate so the solver's
    // memory layout can change without the shader's input changing with it.
    private NativeArray<Particle2D> renderBuffer;

    private NativeArray<float2> boundaryParticles;
    private NativeArray<float2> viscosityForces;
    private NeighborLists neighbors;

    private SPHGrid grid;
    private SolverParams solverParams;
    private int boundaryCount;

    private float spawnSpacing = 1f;
    private float timeAccumulator;
    private float lastStepSize;
    private bool warnedAboutClampConflict;

    private readonly System.Diagnostics.Stopwatch solverTimer = new System.Diagnostics.Stopwatch();
    private double lastSolverMs;

    // Managed snapshots for the editor-only renderers. Reading a NativeArray from
    // managed code goes through the safety system on every index access, and the
    // heat map alone does resolution^2 * particleCount of them per frame. One
    // bulk copy per frame is far cheaper than that.
    private Particle2D[] renderParticles;
    private float2[] renderViscosityForces;
    private float2[] renderBoundary;

    /// <summary>
    /// Live particle state, exposed so the renderer can read it. Native and
    /// read-only by convention: nothing outside the solver writes these.
    /// </summary>
    public NativeArray<Particle2D> Particles => renderBuffer;

    /// <summary>The static wall particles, as the solver sees them.</summary>
    public NativeArray<float2> BoundaryParticles => BoundaryView();

    /// <summary>
    /// The parameter block the Burst passes and the GPU kernels both read. Built
    /// in Awake and on every CPU step; when <see cref="solveOnCpu"/> is false it
    /// only reflects the values as of Awake.
    /// </summary>
    public SolverParams Parameters => solverParams;

    private void Awake()
    {
        state = new ParticleState(particleCount, Allocator.Persistent);
        renderBuffer = new NativeArray<Particle2D>(particleCount, Allocator.Persistent);
        viscosityForces = new NativeArray<float2>(particleCount, Allocator.Persistent);

        // Must exist before any job is scheduled. Jobs cannot be handed an
        // uncreated NativeArray, and mass calibration runs a density job before
        // boundary particles are generated. Parked empty, so calibration sees
        // fluid only -- which is what we want: the bulk fluid should land on
        // restDensity, with walls reading slightly over-dense afterwards.
        ParkDisabledBoundaryArray();

        SpawnParticles();

        // The grid and density passes need solver parameters, so build them
        // before the mass calibration that reads density.
        BuildSolverParams();

        if (autoParticleMass) CalibrateParticleMass();

        SpawnBoundaryParticles();
        BuildSolverParams();

        // Pack the render view once here, so Particles is already valid for
        // anything that reads it in Start() -- notably a GPU simulation seeding
        // itself. Without this the buffer stays zero until the first Update.
        RunRepack();
    }

    private void OnDestroy()
    {
        state.Dispose();
        if (renderBuffer.IsCreated) renderBuffer.Dispose();
        if (viscosityForces.IsCreated) viscosityForces.Dispose();
        if (boundaryParticles.IsCreated) boundaryParticles.Dispose();

        grid.Dispose();
    }

    /// <summary>
    /// Packs all tuning values into the blittable struct the Burst passes take.
    /// At T-026 this becomes the compute shader's constant buffer almost verbatim.
    /// </summary>
    private void BuildSolverParams()
    {
        float h = smoothingLength;

        solverParams = new SolverParams
        {
            smoothingLength = h,
            smoothingLengthSq = h * h,
            poly6Const = SPHMath.Poly6Constant(h),

            spikyPow2Const = SPHMath.SpikyPow2Constant(h),
            spikyPow2GradConst = SPHMath.SpikyPow2GradientConstant(h),
            spikyPow3Const = SPHMath.SpikyPow3Constant(h),
            spikyPow3GradConst = SPHMath.SpikyPow3GradientConstant(h),

            restDensity = restDensity,
            particleMass = particleMass,
            stiffness = stiffness,
            clampPressurePositive = clampPressurePositive,
            nearPressureMultiplier = nearPressureMultiplier,

            viscosity = viscosity,

            boundaryVolume = spawnSpacing * spawnSpacing,
            useBoundaryParticles = useBoundaryParticles && boundaryCount > 0,

            gravity = new float2(gravity.x, gravity.y),
            boxCenter = new float2(transform.position.x, transform.position.y),
            boxHalfSize = new float2(boxSize.x * 0.5f, boxSize.y * 0.5f),
            collisionDamping = collisionDamping,
        };

        WarnIfClampConflictsWithNearChannel();
    }

    /// <summary>
    /// Clamping and the near channel cannot both be active.
    /// </summary>
    /// <remarks>
    /// The near pressure has no rest-density offset, so for a packed fluid it is a
    /// near-constant outward push that does not respond to compression. The only
    /// thing that can hold it in is the main channel going negative. Clamping
    /// makes negative pressure impossible, so the two together mean the fluid
    /// expands without limit -- which reads as particles flying apart rather than
    /// as anything obviously clamp-related.
    ///
    /// Warns once on entering the bad state rather than every frame.
    /// </remarks>
    private void WarnIfClampConflictsWithNearChannel()
    {
        bool conflict = clampPressurePositive && nearPressureMultiplier > 0f;

        if (conflict && !warnedAboutClampConflict)
        {
            warnedAboutClampConflict = true;

            Debug.LogWarning(
                "clampPressurePositive is ON while nearPressureMultiplier > 0. The near " +
                "channel is a permanent outward pressure and needs negative main pressure " +
                "to balance it, so the fluid will expand without limit. Set " +
                "clampPressurePositive = false, or set nearPressureMultiplier = 0.");
        }
        else if (!conflict)
        {
            warnedAboutClampConflict = false;
        }
    }

    /// <summary>
    /// Blue-noise scatter: random positions with a minimum separation.
    ///
    /// Pure uniform random creates dense clumps and empty voids. The local
    /// density then swings far from restDensity and the pressure force spikes.
    /// Enforcing a minimum separation keeps the packing even while the layout
    /// still looks irregular.
    /// </summary>
    private void SpawnParticles()
    {
        Vector2 canvasCenter = transform.position;

        Vector2 sizeFraction = new Vector2(
            Mathf.Clamp(spawnRegionSize.x, 0.02f, 1f),
            Mathf.Clamp(spawnRegionSize.y, 0.02f, 1f));

        Vector2 regionSize = new Vector2(
            boxSize.x * sizeFraction.x,
            boxSize.y * sizeFraction.y);

        // Move the region while keeping it fully inside the container.
        // spawnRegionCenter is in -1..1: (-1,-1) is one corner, (0,0) centred.
        Vector2 maxOffset = (boxSize - regionSize) * 0.5f;
        Vector2 regionCenter = canvasCenter + new Vector2(
            spawnRegionCenter.x * maxOffset.x,
            spawnRegionCenter.y * maxOffset.y);

        float area = regionSize.x * regionSize.y;
        float nominalSpacing = Mathf.Sqrt(area / Mathf.Max(1, particleCount));
        spawnSpacing = nominalSpacing;

        // Derive the smoothing length from the packing so the neighbour count is
        // consistent regardless of particle count or spawn region size.
        smoothingLength = smoothingLengthInSpacing * nominalSpacing;

        float minDist = nominalSpacing * 0.85f;
        float minDistSq = minDist * minDist;
        const int maxAttempts = 30;

        for (int i = 0; i < particleCount; i++)
        {
            float2 candidate = regionCenter;

            for (int attempt = 0; attempt < maxAttempts; attempt++)
            {
                candidate = new float2(
                    regionCenter.x + UnityEngine.Random.Range(-regionSize.x * 0.5f, regionSize.x * 0.5f),
                    regionCenter.y + UnityEngine.Random.Range(-regionSize.y * 0.5f, regionSize.y * 0.5f));

                bool accepted = true;

                for (int k = 0; k < i; k++)
                {
                    if (math.distancesq(state.position[k], candidate) < minDistSq)
                    {
                        accepted = false;
                        break;
                    }
                }

                if (accepted) break;
            }

            state.position[i] = candidate;
            state.velocity[i] = float2.zero;
            state.force[i] = float2.zero;
            state.density[i] = 0f;
            state.pressure[i] = 0f;
            state.nearDensity[i] = 0f;
            state.nearPressure[i] = 0f;
        }
    }

    /// <summary>
    /// Scales particle mass so the spawn's mean density lands on restDensity.
    ///
    /// Calibrating to exactly restDensity would leave the fluid at uniform
    /// pressure, and a uniform pressure has zero gradient -- so nothing would
    /// move. spawnCompression raises the interior pressure above the free
    /// surface, and that difference is the gradient that drives the flow.
    /// </summary>
    private void CalibrateParticleMass()
    {
        particleMass = 1f;
        BuildSolverParams();

        RebuildNeighbors();
        RunDensityPass();

        float meanDensity = SPHSolver.MeanDensity(state.density);

        if (meanDensity > 0f)
        {
            particleMass = (restDensity * spawnCompression) / meanDensity;
            BuildSolverParams();
        }
    }

    /// <summary>
    /// Lays a layer of static particles inside the solid, just beyond each wall.
    /// They exist only to complete the fluid's kernel support near the wall and to
    /// push back when the fluid compresses against it.
    /// </summary>
    private void SpawnBoundaryParticles()
    {
        if (!useBoundaryParticles)
        {
            ParkDisabledBoundaryArray();
            return;
        }

        float spacing = Mathf.Max(0.0001f, spawnSpacing);
        Vector2 center = transform.position;
        Vector2 half = boxSize * 0.5f;
        float margin = smoothingLength;

        int nx = Mathf.Max(1, Mathf.CeilToInt((boxSize.x + 2f * margin) / spacing));
        int ny = Mathf.Max(1, Mathf.CeilToInt((boxSize.y + 2f * margin) / spacing));

        float stepX = (boxSize.x + 2f * margin) / nx;
        float stepY = (boxSize.y + 2f * margin) / ny;

        // NativeArray is fixed size, so gather into a list first, then copy.
        var found = new System.Collections.Generic.List<float2>(nx * ny);

        for (int y = 0; y < ny; y++)
        {
            for (int x = 0; x < nx; x++)
            {
                float px = center.x - half.x - margin + (x + 0.5f) * stepX;
                float py = center.y - half.y - margin + (y + 0.5f) * stepY;

                float outsideX = Mathf.Max(0f, Mathf.Abs(px - center.x) - half.x);
                float outsideY = Mathf.Max(0f, Mathf.Abs(py - center.y) - half.y);

                // Skip samples inside the fluid domain
                if (outsideX <= 0f && outsideY <= 0f) continue;

                // Skip samples too deep into the solid to matter
                if (Mathf.Sqrt(outsideX * outsideX + outsideY * outsideY) > margin) continue;

                found.Add(new float2(px, py));
            }
        }

        if (boundaryParticles.IsCreated) boundaryParticles.Dispose();

        boundaryCount = found.Count;

        if (boundaryCount == 0)
        {
            ParkDisabledBoundaryArray();
            return;
        }

        boundaryParticles = new NativeArray<float2>(boundaryCount, Allocator.Persistent);

        for (int i = 0; i < boundaryCount; i++)
        {
            boundaryParticles[i] = found[i];
        }

        // Static, so one snapshot at spawn is enough for the debug renderer
        renderBoundary = new float2[boundaryCount];
        boundaryParticles.CopyTo(renderBoundary);
    }

    /// <summary>
    /// Keeps a one-element boundary array around when there are no boundary
    /// particles, so jobs are never handed an uncreated NativeArray.
    /// </summary>
    /// <remarks>
    /// The slot is parked far outside the domain, so it can never fall inside a
    /// particle's smoothing radius and never contribute density or force. That is
    /// simpler and less fragile than passing a zero-length slice around, and it
    /// means the disabled case needs no special path in the solver.
    /// </remarks>
    private void ParkDisabledBoundaryArray()
    {
        boundaryCount = 0;

        if (boundaryParticles.IsCreated) boundaryParticles.Dispose();

        boundaryParticles = new NativeArray<float2>(1, Allocator.Persistent);
        boundaryParticles[0] = new float2(1e9f, 1e9f);

        renderBoundary = new float2[0];
    }

    /// <summary>
    /// The boundary array as the solver should see it. Returns an empty view when
    /// boundary particles are disabled, so the toggle takes effect immediately
    /// without reallocating.
    /// </summary>
    private NativeArray<float2> BoundaryView()
    {
        // Always a live array even when boundary particles are disabled, because
        // jobs cannot take an uncreated NativeArray. Self-healing rather than
        // relying on Awake ordering: getting this wrong throws at schedule time
        // with a message that says nothing about boundary particles.
        if (!boundaryParticles.IsCreated) ParkDisabledBoundaryArray();

        return boundaryParticles;
    }

    private void RebuildNeighbors()
    {
        // The grid must cover the container plus one smoothing length of margin,
        // because boundary particles sit just outside the walls.
        float margin = smoothingLength * 1.01f;

        Vector2 center = transform.position;
        Vector2 domainSize = boxSize + new Vector2(2f * margin, 2f * margin);
        float2 domainOrigin = new float2(
            center.x - domainSize.x * 0.5f,
            center.y - domainSize.y * 0.5f);

        neighbors = grid.Rebuild(
            state.position,
            BoundaryView(),
            domainOrigin,
            new float2(domainSize.x, domainSize.y),
            smoothingLength);
    }

    private void Update()
    {
        if (!state.IsCreated) return;

        // A GPU simulation owns the state when it is driving; running the Burst
        // solver as well would advance the same particles twice.
        if (!solveOnCpu) return;

        BuildSolverParams();

        // Frame time goes into a buffer rather than straight into the integrator.
        // The solver then consumes it in steps small enough to stay stable, so a
        // slow frame produces more steps instead of one large unstable step.
        timeAccumulator += Time.deltaTime;

        int steps = 0;

        solverTimer.Restart();

        while (timeAccumulator > 0f && steps < maxSubSteps)
        {
            SolveStep();

            // Size the next step from the state we just produced, then never step
            // past the remaining budget so the simulation stays in sync with time.
            float dt = Mathf.Clamp(
                SPHSolver.ComputeStableTimeStep(state, solverParams, cflFactor),
                minTimeStep, maxTimeStep);

            dt = Mathf.Min(dt, timeAccumulator);
            lastStepSize = dt;

            RunIntegratePass(dt);

            timeAccumulator -= dt;
            steps++;
        }

        // Drop any backlog. A long hitch must not put the solver into a spiral of
        // death where it falls further behind the frame rate every frame.
        timeAccumulator = 0f;

        solverTimer.Stop();
        lastSolverMs = solverTimer.Elapsed.TotalMilliseconds;

        // Refresh the render view every frame, independently of the gizmos.
        RunRepack();

        if (debugLogs)
        {
            SPHSolver.Diagnostics(state, solverParams,
                out float meanDensity, out float maxSpeed, out float maxAccel);

            Debug.Log(
                $"steps={steps}  dt={lastStepSize:F5}  solver={lastSolverMs:F1}ms\n" +
                $"  setup : gravity=({gravity.x:F2},{gravity.y:F2})  h={smoothingLength:F3}  " +
                $"mass={particleMass:F1}  boundary={boundaryCount}  " +
                $"clamp={clampPressurePositive}  nearK={nearPressureMultiplier:F1}  visc={viscosity:F1}\n" +
                $"  state : meanRho/restRho={meanDensity / Mathf.Max(1e-6f, restDensity):F4}  " +
                $"maxSpeed={maxSpeed:F2}  maxAccel={maxAccel:F1}  (gravity={Mathf.Abs(gravity.y):F2})");
        }
    }

    /// <summary>One full solver pass: neighbours, density, pressure, forces.</summary>
    /// <summary>
    /// Schedules every solver pass for one step and waits for them to finish.
    /// </summary>
    /// <remarks>
    /// The passes form two chains that meet at the force accumulation:
    ///
    ///   ResetForces ..........................\
    ///                                          +--> PressureForce --> Viscosity
    ///   Density --> Pressure ................/
    ///
    /// Dependencies rather than fusion keep each pass comparable to the serial
    /// version it replaced, at the cost of one extra scheduling step.
    /// </remarks>
    private void SolveStep()
    {
        RebuildNeighbors();

        int count = state.Count;
        int batch = Mathf.Max(1, batchCount);
        NativeArray<float2> boundary = BoundaryView();

        JobHandle density = new ComputeDensityJob
        {
            position = state.position,
            neighbors = neighbors,
            boundaryPositions = boundary,
            density = state.density,
            nearDensity = state.nearDensity,
            p = solverParams,
        }.Schedule(count, batch);

        JobHandle pressure = new ComputePressureJob
        {
            density = state.density,
            nearDensity = state.nearDensity,
            pressure = state.pressure,
            nearPressure = state.nearPressure,
            p = solverParams,
        }.Schedule(count, batch, density);

        JobHandle reset = new ResetForcesJob
        {
            force = state.force,
        }.Schedule(count, batch);

        JobHandle pressureForce = new ComputePressureForceJob
        {
            position = state.position,
            density = state.density,
            pressure = state.pressure,
            nearDensity = state.nearDensity,
            nearPressure = state.nearPressure,
            neighbors = neighbors,
            boundaryPositions = boundary,
            force = state.force,
            p = solverParams,
        }.Schedule(count, batch, JobHandle.CombineDependencies(reset, pressure));

        JobHandle viscosity = new ComputeViscosityJob
        {
            position = state.position,
            velocity = state.velocity,
            neighbors = neighbors,
            viscosityForce = viscosityForces,
            force = state.force,
            p = solverParams,
        }.Schedule(count, batch, pressureForce);

        viscosity.Complete();
    }

    /// <summary>Density only. Used once at spawn to calibrate particle mass.</summary>
    private void RunDensityPass()
    {
        new ComputeDensityJob
        {
            position = state.position,
            neighbors = neighbors,
            boundaryPositions = BoundaryView(),
            density = state.density,
            nearDensity = state.nearDensity,
            p = solverParams,
        }.Schedule(state.Count, Mathf.Max(1, batchCount)).Complete();
    }

    /// <summary>
    /// Integration is scheduled apart from the rest of the step because dt depends
    /// on the forces those passes just produced. Batching it together would mean
    /// sizing dt from the previous step, which lags the state.
    /// </summary>
    private void RunIntegratePass(float dt)
    {
        new IntegrateJob
        {
            position = state.position,
            velocity = state.velocity,
            force = state.force,
            density = state.density,
            p = solverParams,
            dt = dt,
        }.Schedule(state.Count, Mathf.Max(1, batchCount)).Complete();
    }

    /// <summary>Refreshes the packed Particle2D view the renderer uploads.</summary>
    private void RunRepack()
    {
        new RepackRenderBufferJob
        {
            position = state.position,
            velocity = state.velocity,
            force = state.force,
            density = state.density,
            pressure = state.pressure,
            nearDensity = state.nearDensity,
            nearPressure = state.nearPressure,
            render = renderBuffer,
        }.Schedule(state.Count, Mathf.Max(1, batchCount)).Complete();
    }

    // ------------------------------------------------------------------
    // Debug visualization
    // ------------------------------------------------------------------

    /// <summary>
    /// Copies native state into managed arrays for the gizmo code. Called once
    /// per frame from OnDrawGizmos, which is the only consumer.
    /// </summary>
    private void RefreshRenderSnapshot()
    {
        if (!renderBuffer.IsCreated) return;

        if (renderParticles == null || renderParticles.Length != renderBuffer.Length)
        {
            renderParticles = new Particle2D[renderBuffer.Length];
        }

        renderBuffer.CopyTo(renderParticles);

        if (viscosityForces.IsCreated)
        {
            if (renderViscosityForces == null || renderViscosityForces.Length != viscosityForces.Length)
            {
                renderViscosityForces = new float2[viscosityForces.Length];
            }

            viscosityForces.CopyTo(renderViscosityForces);
        }
    }

    private void OnDrawGizmos()
    {
        if (!renderBuffer.IsCreated) return;

        RefreshRenderSnapshot();

        // 1. Draw container
        Gizmos.color = Color.blue;
        Gizmos.DrawWireCube(transform.position, new Vector3(boxSize.x, boxSize.y, 0f));

        // 1a. Boundary particles (static solid material just outside the walls)
        if (renderBoundary != null && useBoundaryParticles)
        {
            Gizmos.color = new Color(0.4f, 0.4f, 0.4f, 1f);

            for (int b = 0; b < renderBoundary.Length; b++)
            {
                Gizmos.DrawSphere(new Vector3(renderBoundary[b].x, renderBoundary[b].y, 0f), 0.05f);
            }
        }

        // 2. Optional gizmo spheres, coloured by density or pressure.
        // ParticleRenderer2D draws the instanced discs; this is the fallback
        // visualiser, and running both at once doubles the work.
        if (showParticleGizmos)
        for (int i = 0; i < renderParticles.Length; i++)
        {
            Particle2D particle = renderParticles[i];

            float value = showPressureColor ? particle.pressure : particle.density;
            float maxValue = showPressureColor ? stiffness * (restDensity * 0.5f) : restDensity * 1.5f;
            float t = Mathf.InverseLerp(0f, maxValue, value);

            Gizmos.color = Color.Lerp(Color.blue, Color.red, t);
            Gizmos.DrawSphere(new Vector3(particle.position.x, particle.position.y, 0f), 0.08f);
        }

        // 3. Force arrows
        // Direction = force direction. Color = force magnitude (weak -> strong)
        if (showViscosityArrows && renderViscosityForces != null)
        {
            DrawArrows(renderViscosityForces, Color.yellow, new Color(1f, 0.35f, 0f));
        }
        else if (showForceArrows)
        {
            DrawParticleArrows();
        }
    }

    private void DrawArrows(float2[] forces, Color weak, Color strong)
    {
        float maxMag = 1e-6f;

        for (int i = 0; i < forces.Length; i++)
        {
            float mag = math.length(forces[i]);
            if (mag > maxMag) maxMag = mag;
        }

        for (int i = 0; i < forces.Length; i++)
        {
            float2 f = forces[i];
            float mag = math.length(f);
            if (mag < 1e-6f) continue;

            float2 pos = renderParticles[i].position;
            Vector3 origin = new Vector3(pos.x, pos.y, 0f);
            Vector3 dir = new Vector3(f.x / mag, f.y / mag, 0f);

            Gizmos.color = Color.Lerp(weak, strong, Mathf.InverseLerp(0f, maxMag, mag));
            Gizmos.DrawLine(origin, origin + dir * forceArrowLength);
        }
    }

    private void DrawParticleArrows()
    {
        float maxForce = 1e-6f;

        for (int i = 0; i < renderParticles.Length; i++)
        {
            float mag = math.length(renderParticles[i].force);
            if (mag > maxForce) maxForce = mag;
        }

        for (int i = 0; i < renderParticles.Length; i++)
        {
            float2 f = renderParticles[i].force;
            float mag = math.length(f);
            if (mag < 1e-6f) continue;

            float2 pos = renderParticles[i].position;
            Vector3 origin = new Vector3(pos.x, pos.y, 0f);
            Vector3 dir = new Vector3(f.x / mag, f.y / mag, 0f);

            Gizmos.color = Color.Lerp(Color.cyan, Color.magenta, Mathf.InverseLerp(0f, maxForce, mag));
            Gizmos.DrawLine(origin, origin + dir * forceArrowLength);
        }
    }

}

using Unity.Collections;
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
    public Vector2Int numToSpawn = new Vector2Int(20, 20);

    [Header("Physics")]
    public Vector2 gravity = new Vector2(0f, -9.81f);
    public float collisionDamping = 0.5f;

    // h is derived at spawn as h = smoothingLengthInSpacing * particle spacing.
    // Deriving it keeps the neighbour count in a sane range no matter how many
    // particles there are or how large the spawn region is.
    public float smoothingLengthInSpacing = 2.2f;
    [HideInInspector] public float smoothingLength = 2f;

    public int debugParticle = 0;

    [Header("Simulation")]
    public int maxSubSteps = 32;
    [Range(0.05f, 0.5f)] public float cflFactor = 0.25f;
    public float minTimeStep = 0.0002f;
    public float maxTimeStep = 0.02f;

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
    public bool clampPressurePositive = true;
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

    [Header("Pressure Heat Map")]
    public bool showPressureHeatMap = true;
    public int heatMapResolution = 64;
    [Range(1f, 3f)] public float heatMapBlur = 1.5f;
    [Range(0f, 1f)] public float heatMapOpacity = 0.6f;

    // --- Native simulation state ---
    private NativeArray<Particle2D> particles;
    private NativeArray<float2> boundaryParticles;
    private NativeArray<float2> viscosityForces;
    private NeighborLists neighbors;

    private SPHGrid grid;
    private SolverParams solverParams;
    private int boundaryCount;

    private float spawnSpacing = 1f;
    private float timeAccumulator;
    private float lastStepSize;

    private readonly System.Diagnostics.Stopwatch solverTimer = new System.Diagnostics.Stopwatch();
    private double lastSolverMs;

    // Managed snapshots for the editor-only renderers. Reading a NativeArray from
    // managed code goes through the safety system on every index access, and the
    // heat map alone does resolution^2 * particleCount of them per frame. One
    // bulk copy per frame is far cheaper than that.
    private Particle2D[] renderParticles;
    private float2[] renderViscosityForces;
    private float2[] renderBoundary;

    // --- Heat map (debug only, handled in managed space) ---
    private Mesh heatMapMesh;
    private Material heatMapMaterial;
    private Vector3[] heatMapVerts;
    private Color[] heatMapColors;
    private int builtHeatMapResolution = -1;
    private Vector2 builtBoxSize;

    public int TotalParticles => numToSpawn.x * numToSpawn.y;

    private void Awake()
    {
        particles = new NativeArray<Particle2D>(TotalParticles, Allocator.Persistent);
        viscosityForces = new NativeArray<float2>(TotalParticles, Allocator.Persistent);

        SpawnParticles();

        // The grid and density passes need solver parameters, so build them
        // before the mass calibration that reads density.
        BuildSolverParams();

        if (autoParticleMass) CalibrateParticleMass();

        SpawnBoundaryParticles();
        BuildSolverParams();
    }

    private void OnDestroy()
    {
        if (particles.IsCreated) particles.Dispose();
        if (viscosityForces.IsCreated) viscosityForces.Dispose();
        if (boundaryParticles.IsCreated) boundaryParticles.Dispose();

        grid.Dispose();

        if (heatMapMesh != null)
        {
            if (Application.isPlaying) Destroy(heatMapMesh);
            else DestroyImmediate(heatMapMesh);
        }

        if (heatMapMaterial != null)
        {
            if (Application.isPlaying) Destroy(heatMapMaterial);
            else DestroyImmediate(heatMapMaterial);
        }
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
            spikyConst = SPHMath.SpikyGradientConstant(h),
            viscConst = SPHMath.ViscosityLaplacianConstant(h),

            restDensity = restDensity,
            particleMass = particleMass,
            stiffness = stiffness,
            clampPressurePositive = clampPressurePositive,

            viscosity = viscosity,

            boundaryVolume = spawnSpacing * spawnSpacing,
            useBoundaryParticles = useBoundaryParticles && boundaryCount > 0,

            gravity = new float2(gravity.x, gravity.y),
            boxCenter = new float2(transform.position.x, transform.position.y),
            boxHalfSize = new float2(boxSize.x * 0.5f, boxSize.y * 0.5f),
            collisionDamping = collisionDamping,
        };
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
        float nominalSpacing = Mathf.Sqrt(area / Mathf.Max(1, TotalParticles));
        spawnSpacing = nominalSpacing;

        // Derive the smoothing length from the packing so the neighbour count is
        // consistent regardless of particle count or spawn region size.
        smoothingLength = smoothingLengthInSpacing * nominalSpacing;

        float minDist = nominalSpacing * 0.85f;
        float minDistSq = minDist * minDist;
        const int maxAttempts = 30;

        for (int i = 0; i < TotalParticles; i++)
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
                    if (math.distancesq(particles[k].position, candidate) < minDistSq)
                    {
                        accepted = false;
                        break;
                    }
                }

                if (accepted) break;
            }

            particles[i] = new Particle2D
            {
                position = candidate,
                velocity = float2.zero,
                force = float2.zero,
                density = 0f,
                pressure = 0f,
            };
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
        SPHSolver.ComputeDensity(particles, neighbors, BoundaryView(), solverParams);

        float meanDensity = SPHSolver.MeanDensity(particles);

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
            boundaryCount = 0;
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
        boundaryParticles = new NativeArray<float2>(
            Mathf.Max(1, boundaryCount), Allocator.Persistent);

        for (int i = 0; i < boundaryCount; i++)
        {
            boundaryParticles[i] = found[i];
        }

        // Static, so one snapshot at spawn is enough for the debug renderer
        renderBoundary = new float2[boundaryCount];
        boundaryParticles.GetSubArray(0, boundaryCount).CopyTo(renderBoundary);
    }

    /// <summary>
    /// The boundary array as the solver should see it. Returns an empty view when
    /// boundary particles are disabled, so the toggle takes effect immediately
    /// without reallocating.
    /// </summary>
    private NativeArray<float2> BoundaryView()
    {
        if (!boundaryParticles.IsCreated || !useBoundaryParticles)
        {
            return new NativeArray<float2>();
        }

        return boundaryParticles.GetSubArray(0, boundaryCount);
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
            particles,
            BoundaryView(),
            domainOrigin,
            new float2(domainSize.x, domainSize.y),
            smoothingLength);
    }

    private void Update()
    {
        if (!particles.IsCreated) return;

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
                SPHSolver.ComputeStableTimeStep(particles, solverParams, cflFactor),
                minTimeStep, maxTimeStep);

            dt = Mathf.Min(dt, timeAccumulator);
            lastStepSize = dt;

            SPHSolver.Integrate(particles, solverParams, dt);

            timeAccumulator -= dt;
            steps++;
        }

        // Drop any backlog. A long hitch must not put the solver into a spiral of
        // death where it falls further behind the frame rate every frame.
        timeAccumulator = 0f;

        solverTimer.Stop();
        lastSolverMs = solverTimer.Elapsed.TotalMilliseconds;

        if (debugLogs)
        {
            Debug.Log($"steps={steps}, dt={lastStepSize:F5}, solver={lastSolverMs:F2}ms, " +
                      $"h={smoothingLength:F3}, mass={particleMass:F2}, boundary={boundaryCount}");
        }
    }

    /// <summary>One full solver pass: neighbours, density, pressure, forces.</summary>
    private void SolveStep()
    {
        RebuildNeighbors();

        SPHSolver.ComputeDensity(particles, neighbors, BoundaryView(), solverParams);
        SPHSolver.ComputePressure(particles, solverParams);

        SPHSolver.ResetForces(particles);
        SPHSolver.ComputePressureForce(particles, neighbors, BoundaryView(), solverParams);
        SPHSolver.ComputeViscosityForce(particles, neighbors, solverParams, viscosityForces);
    }

    // ------------------------------------------------------------------
    // Debug visualization
    // ------------------------------------------------------------------

    private int HeatMapResolutionClamped => Mathf.Clamp(heatMapResolution, 2, 512);

    /// <summary>
    /// Copies native state into managed arrays for the gizmo and heat map code.
    /// Called once per frame from OnDrawGizmos, which is the only consumer.
    /// </summary>
    private void RefreshRenderSnapshot()
    {
        if (renderParticles == null || renderParticles.Length != particles.Length)
        {
            renderParticles = new Particle2D[particles.Length];
        }

        particles.CopyTo(renderParticles);

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
        if (!particles.IsCreated) return;

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

        // 1b. Pressure heat map across the whole canvas
        if (showPressureHeatMap)
        {
            DrawPressureHeatMap();
        }

        // 2. Draw particles colored by density or pressure
        // Low value = blue, high value = red
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

        // 4. Debug: one particle's neighbour connections
        DrawDebugNeighbors();
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

    /// <summary>
    /// Draws only one particle's neighbour connections, so you can tell which
    /// connections belong to whom. Drawing them all produces an unreadable mesh.
    /// </summary>
    private void DrawDebugNeighbors()
    {
        if (!neighbors.fluidStart.IsCreated || renderParticles.Length == 0) return;

        int target = Mathf.Clamp(debugParticle, 0, renderParticles.Length - 1);

        Gizmos.color = Color.red;
        float2 targetPos = renderParticles[target].position;
        Gizmos.DrawWireSphere(new Vector3(targetPos.x, targetPos.y, 0f), 0.12f);

        Gizmos.color = Color.green;

        for (int k = neighbors.fluidStart[target]; k < neighbors.fluidStart[target + 1]; k++)
        {
            float2 otherPos = renderParticles[neighbors.fluid[k]].position;
            Gizmos.DrawLine(
                new Vector3(targetPos.x, targetPos.y, 0f),
                new Vector3(otherPos.x, otherPos.y, 0f));
        }
    }

    // ------------------------------------------------------------------
    // Heat map: samples the pressure field into a vertex-coloured mesh
    // ------------------------------------------------------------------

    private void DrawPressureHeatMap()
    {
        if (heatMapMesh == null ||
            builtHeatMapResolution != HeatMapResolutionClamped ||
            builtBoxSize != boxSize)
        {
            RebuildHeatMapMesh();
        }

        UpdateHeatMapColors();
        DrawHeatMapMesh();
    }

    /// <summary>
    /// Builds a quad-grid mesh covering the container. Vertex colors are written
    /// each frame; the GPU interpolates between them, which gives a smooth field.
    /// </summary>
    private void RebuildHeatMapMesh()
    {
        int res = HeatMapResolutionClamped;
        int side = res + 1;

        heatMapVerts = new Vector3[side * side];
        heatMapColors = new Color[side * side];
        int[] tris = new int[res * res * 6];

        float cellW = boxSize.x / res;
        float cellH = boxSize.y / res;

        for (int y = 0; y < side; y++)
        {
            for (int x = 0; x < side; x++)
            {
                int idx = y * side + x;
                heatMapVerts[idx] = new Vector3(
                    -boxSize.x * 0.5f + x * cellW,
                    -boxSize.y * 0.5f + y * cellH,
                    0f);
            }
        }

        int t = 0;
        for (int y = 0; y < res; y++)
        {
            for (int x = 0; x < res; x++)
            {
                int i0 = y * side + x;
                int i1 = i0 + 1;
                int i2 = i0 + side;
                int i3 = i2 + 1;

                tris[t++] = i0; tris[t++] = i2; tris[t++] = i1;
                tris[t++] = i1; tris[t++] = i2; tris[t++] = i3;
            }
        }

        if (heatMapMesh == null)
        {
            heatMapMesh = new Mesh { hideFlags = HideFlags.HideAndDontSave };
        }

        heatMapMesh.Clear();

        // A 16-bit index buffer caps a mesh at 65,535 vertices. Resolutions above
        // ~255 cells per side exceed that, so switch to 32-bit indices.
        heatMapMesh.indexFormat = heatMapVerts.Length > 65535
            ? UnityEngine.Rendering.IndexFormat.UInt32
            : UnityEngine.Rendering.IndexFormat.UInt16;

        heatMapMesh.vertices = heatMapVerts;
        heatMapMesh.colors = heatMapColors;
        heatMapMesh.triangles = tris;

        builtHeatMapResolution = res;
        builtBoxSize = boxSize;
    }

    /// <summary>
    /// Samples the pressure field at every mesh vertex and writes the vertex colors:
    /// blue = below rest density, white = at rest density, red = above rest density
    /// </summary>
    private void UpdateHeatMapColors()
    {
        // Sample with a wider kernel than the physics kernel to smooth the field
        float sampleH = smoothingLength * heatMapBlur;
        float h2 = sampleH * sampleH;
        float poly6Const = 4f / (Mathf.PI * Mathf.Pow(sampleH, 8f));
        Vector2 center = transform.position;

        // Auto-range so the heat map always shows contrast
        float maxAbs = 1f;

        for (int i = 0; i < renderParticles.Length; i++)
        {
            float absP = Mathf.Abs(renderParticles[i].pressure);
            if (absP > maxAbs) maxAbs = absP;
        }

        for (int v = 0; v < heatMapVerts.Length; v++)
        {
            float2 sample = new float2(
                center.x + heatMapVerts[v].x,
                center.y + heatMapVerts[v].y);

            float weightedPressure = 0f;
            float weightSum = 0f;

            for (int i = 0; i < renderParticles.Length; i++)
            {
                float r2 = math.distancesq(renderParticles[i].position, sample);

                if (r2 < h2)
                {
                    float diff = h2 - r2;
                    float w = poly6Const * diff * diff * diff;
                    weightedPressure += renderParticles[i].pressure * w;
                    weightSum += w;
                }
            }

            // No particles nearby: leave vertex fully transparent
            if (weightSum <= 0f)
            {
                heatMapColors[v] = new Color(0f, 0f, 0f, 0f);
                continue;
            }

            float p = weightedPressure / weightSum;
            float t = Mathf.Clamp(p / maxAbs, -1f, 1f);

            Color c = t < 0f
                ? Color.Lerp(Color.white, Color.blue, -t)
                : Color.Lerp(Color.white, Color.red, t);

            c.a = heatMapOpacity;
            heatMapColors[v] = c;
        }

        heatMapMesh.colors = heatMapColors;
    }

    private void DrawHeatMapMesh()
    {
        if (heatMapMaterial == null)
        {
            Shader shader = Shader.Find("Hidden/Internal-Colored");
            if (shader == null) return;

            heatMapMaterial = new Material(shader) { hideFlags = HideFlags.HideAndDontSave };
            heatMapMaterial.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            heatMapMaterial.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            heatMapMaterial.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
            heatMapMaterial.SetInt("_ZWrite", 0);
        }

        heatMapMaterial.SetPass(0);
        Graphics.DrawMeshNow(heatMapMesh, transform.localToWorldMatrix);
    }
}

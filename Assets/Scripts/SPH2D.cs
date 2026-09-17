using UnityEngine;
using System.Collections.Generic;

public class SPH2D : MonoBehaviour
{
    public Vector2 boxSize = new Vector2(20f, 20f);

    public Vector2Int numToSpawn = new Vector2Int(20, 20);

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

    private Particle2D[] particles;
    private List<int>[] neighbors;

    private Vector2[] viscosityForces;
    private Vector2[] boundaryParticles;
    private float boundaryVolume;
    private float spawnSpacing = 1f;
    private float timeAccumulator;
    private float lastStepSize;

    private Mesh heatMapMesh;
    private Material heatMapMaterial;
    private Vector3[] heatMapVerts;
    private Color[] heatMapColors;
    private int builtHeatMapResolution = -1;
    private Vector2 builtBoxSize;

    public int TotalParticles => numToSpawn.x * numToSpawn.y;

    private void Awake()
    {
        SpawnParticles();

        // Scale particle mass so the spawn's mean density lands on restDensity.
        // Without this, restDensity and particleMass must be hand-matched or the
        // pressure field starts far from rest and the fluid explodes on frame one.
        if (autoParticleMass) CalibrateParticleMass();

        SpawnBoundaryParticles();
    }

    // Measures the density of the spawn using unit mass, then rescales mass so
    // the mean density equals restDensity. Makes the parameters self-consistent.
    private void CalibrateParticleMass()
    {
        particleMass = 1f;

        FindNeighbors();
        ComputeDensity();

        float sum = 0f;
        int count = 0;

        for (int i = 0; i < particles.Length; i++)
        {
            if (particles[i].density > 0f)
            {
                sum += particles[i].density;
                count++;
            }
        }

        if (count > 0)
        {
            float meanDensity = sum / count;

            // Calibrating to exactly restDensity would leave the fluid at uniform
            // pressure, and a uniform pressure has zero gradient -- so nothing would
            // move. Spawning denser raises the interior pressure above the free
            // surface, and that difference is the gradient that drives the flow.
            particleMass = (restDensity * spawnCompression) / meanDensity;
        }
    }

    private void SpawnParticles()
    {
        particles = new Particle2D[TotalParticles];

        // The fluid starts packed into a sub-region of the container.
        //
        // A uniform density field has zero pressure gradient, and force comes from
        // the gradient -- so filling the whole canvas leaves the fluid jammed
        // against the walls with nothing to do. Packing it into part of the canvas
        // gives it room to move into, which is what produces visible flow.
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

        // Blue-noise spacing derived from the region the fluid actually occupies
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
            Vector2 candidate = regionCenter;

            for (int attempt = 0; attempt < maxAttempts; attempt++)
            {
                candidate = regionCenter + new Vector2(
                    Random.Range(-regionSize.x * 0.5f, regionSize.x * 0.5f),
                    Random.Range(-regionSize.y * 0.5f, regionSize.y * 0.5f)
                );

                bool accepted = true;
                for (int k = 0; k < i; k++)
                {
                    if ((particles[k].position - candidate).sqrMagnitude < minDistSq)
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
                velocity = Vector2.zero,
                force = Vector2.zero,
                density = 0f,
                pressure = 0f
            };
        }
    }

    private void Update()
    {
        if (particles == null)
            return;

        // Frame time goes into a buffer rather than straight into the integrator.
        // The solver then consumes it in steps small enough to stay stable, so a
        // slow frame produces more steps instead of one large unstable step.
        timeAccumulator += Time.deltaTime;

        int steps = 0;

        while (timeAccumulator > 0f && steps < maxSubSteps)
        {
            // Size the step from the current state, then never step past the
            // remaining budget so the simulation stays in sync with real time.
            float dt = Mathf.Clamp(ComputeStableTimeStep(), minTimeStep, maxTimeStep);
            dt = Mathf.Min(dt, timeAccumulator);
            lastStepSize = dt;

            FindNeighbors();
            ComputeDensity();
            ComputePressure();

            // Zero the accumulators once, then let every force pass add into them
            ResetForces();
            ComputePressureForce();
            ComputeViscosityForce();

            Integrate(dt);

            timeAccumulator -= dt;
            steps++;
        }

        // Drop any backlog. A long hitch must not put the solver into a spiral of
        // death where it falls further behind the frame rate every frame.
        timeAccumulator = 0f;

        // DEBUG: log density/pressure per particle so you can verify the Tait EOS
        if (debugLogs)
        {
            Debug.Log($"steps={steps}, dt={lastStepSize:F5}");

            for (int i = 0; i < particles.Length; i++)
            {
                Debug.Log($"Particle {i}: density={particles[i].density:F2}, pressure={particles[i].pressure:F2}");
            }
        }

    }

    // Largest step that keeps the explicit integration stable.
    //
    // CFL limit: information must not travel further than one smoothing length per
    // step, so dt <= C * h / (maxSpeed + soundSpeed). The sound speed comes from the
    // equation of state: with p = k(rho - rho0) we have dp/drho = k, so c = sqrt(k).
    // A stiffer fluid therefore forces a smaller step -- this is the price of
    // weak compressibility.
    //
    // Acceleration limit: a particle must not be flung across its own
    // neighbourhood within a single step.
    private float ComputeStableTimeStep()
    {
        float h = smoothingLength;
        float soundSpeed = Mathf.Sqrt(Mathf.Max(0f, stiffness));

        float maxSpeed = 0f;
        float maxAccel = 0f;

        for (int i = 0; i < particles.Length; i++)
        {
            float speed = particles[i].velocity.magnitude;
            if (speed > maxSpeed) maxSpeed = speed;

            if (particles[i].density > 0.0001f)
            {
                float accel = (particles[i].force / particles[i].density + gravity).magnitude;
                if (accel > maxAccel) maxAccel = accel;
            }
        }

        float dtCfl = cflFactor * h / (maxSpeed + soundSpeed + 0.0001f);

        float dtAcc = maxAccel > 0.0001f
            ? cflFactor * Mathf.Sqrt(h / maxAccel)
            : float.MaxValue;

        return Mathf.Min(dtCfl, dtAcc);
    }

    private void FindNeighbors()
    {
        int n = particles.Length;                                                 
                                                                                 
       neighbors = new List<int>[n];                                             
                                                                                 
       for (int i = 0; i < n; i++)                                               
       {                                                                         
           neighbors[i] = new List<int>();                                       
                                                                                 
           for (int j = 0; j < n; j++)                                           
           {                                                                      
               if (i == j) continue;                                              
                                                                                  
               float dist = Vector2.Distance(particles[i].position, particles[j].position);                                                         
                                                                                  
               if (dist < smoothingLength)                                       
               {                                                                 
                   neighbors[i].Add(j);                                          
               }                                                                 
           }                                                                     
       }                                                                         
                                                                                  
       // DEBUG: log neighbor count per particle so you can verify neighbor search is working
       if (debugLogs)
       {
           for (int i = 0; i < n; i++)
           {
               Debug.Log($"Particle {i}: {neighbors[i].Count} neighbors");
           }
       }                                                                         
    }    

    private void ComputeDensity()
    {
        float h2 = smoothingLength * smoothingLength;
        float poly6Const = 4f / (Mathf.PI * Mathf.Pow(smoothingLength, 8f));

        for (int i = 0; i < particles.Length; i++)
        {
            float density = 0f;

            // Fluid neighbours
            for (int j = 0; j < neighbors[i].Count; j++)
            {
                int n = neighbors[i][j];
                float r = Vector2.Distance(particles[i].position, particles[n].position);

                if (r > 0f && r < smoothingLength)
                {
                    float diff = h2 - r * r;
                    density += particleMass * poly6Const * diff * diff * diff;
                }
            }

            // Boundary particles stand for solid material just beyond the wall.
            // Near a wall roughly half the kernel support lies inside the solid, so
            // without these contributions the density is badly under-estimated and
            // the pressure that should push the fluid off the wall never appears.
            if (useBoundaryParticles && boundaryParticles != null)
            {
                for (int b = 0; b < boundaryParticles.Length; b++)
                {
                    float r2 = (boundaryParticles[b] - particles[i].position).sqrMagnitude;
                    if (r2 >= h2) continue;

                    float diff = h2 - r2;
                    density += restDensity * boundaryVolume * poly6Const * diff * diff * diff;
                }
            }

            particles[i].density = density;
        }
    }

    private void ComputePressure()
    {
        for (int i = 0; i < particles.Length; i++)
        {
            // Tait equation of state: p = k * (density - restDensity)
            float p = stiffness * (particles[i].density - restDensity);

            // Clamping away negative pressure removes cohesion, which otherwise
            // makes particles string together and stick at free surfaces.
            if (clampPressurePositive && p < 0f) p = 0f;

            particles[i].pressure = p;
        }
    }

    // Turns the scalar pressure field into a vector force between neighbors.
    // Uses the Spiky kernel gradient, whose non-zero value at r = 0 prevents clumping.
    private void ComputePressureForce()
    {
        float h = smoothingLength;

        // Magnitude of the 2D Spiky gradient: |dW/dr| = 30 / (pi h^5) * (h - r)^2
        // It stays non-zero at r = 0, which is what keeps particles from overlapping.
        float spikyGradMag = 30f / (Mathf.PI * Mathf.Pow(h, 5f));

        for (int i = 0; i < particles.Length; i++)
        {
            for (int j = 0; j < neighbors[i].Count; j++)
            {
                int n = neighbors[i][j];

                // Guard against zero density from isolated particles
                if (particles[n].density <= 0.0001f) continue;

                // Points from neighbor j to particle i
                Vector2 rVec = particles[i].position - particles[n].position;
                float r = rVec.magnitude;

                if (r <= 0f || r >= h) continue;

                Vector2 dir = rVec / r;

                float hr = h - r;

                // Muller Eqn (10): m_j * (p_i + p_j) / (2 rho_j) * gradW_spiky
                //
                // Sign check: f_i = -(p_i + p_j) * gradW, and gradW points from i to j.
                // So with positive pressure the force must point from j to i (away from j).
                float contribution = particleMass
                                     * (particles[i].pressure + particles[n].pressure)
                                     / (2f * particles[n].density)
                                     * spikyGradMag * hr * hr;

                particles[i].force += dir * contribution;
            }

            // Boundary particles push back using this fluid particle's own pressure
            // (pressure mirroring), which is what enforces no-penetration at the wall.
            if (useBoundaryParticles && boundaryParticles != null)
            {
                float pressure = particles[i].pressure;

                for (int b = 0; b < boundaryParticles.Length; b++)
                {
                    // Points from the boundary particle to the fluid particle (inward)
                    Vector2 rVec = particles[i].position - boundaryParticles[b];
                    float r2 = rVec.sqrMagnitude;
                    if (r2 <= 0f || r2 >= h * h) continue;

                    float r = Mathf.Sqrt(r2);
                    Vector2 dir = rVec / r;
                    float hr = h - r;

                    // m_b = restDensity * V_b, and mirroring gives p_b = p_i and
                    // rho_b = restDensity, so the coefficient collapses to V_b * p_i.
                    particles[i].force += dir * (boundaryVolume * pressure * spikyGradMag * hr * hr);
                }
            }
        }
    }

    // Zeroes every particle's force accumulator. Called once per step, before the
    // individual force passes add their contributions.
    private void ResetForces()
    {
        for (int i = 0; i < particles.Length; i++)
        {
            particles[i].force = Vector2.zero;
        }
    }

    // Internal friction: drags each particle's velocity toward its neighbours'.
    // This is the only dissipative term in the solver, so it is what allows the
    // fluid to settle instead of sloshing forever.
    private void ComputeViscosityForce()
    {
        float h = smoothingLength;

        // Laplacian of the 2D viscosity kernel: 40 / (pi h^5) * (h - r).
        // It is positive everywhere for 0 <= r <= h, which guarantees the force
        // always shrinks velocity differences instead of growing them.
        float viscLapMag = 40f / (Mathf.PI * Mathf.Pow(h, 5f));

        if (viscosityForces == null || viscosityForces.Length != particles.Length)
        {
            viscosityForces = new Vector2[particles.Length];
        }

        for (int i = 0; i < particles.Length; i++)
        {
            Vector2 viscForce = Vector2.zero;

            for (int j = 0; j < neighbors[i].Count; j++)
            {
                int n = neighbors[i][j];

                if (particles[n].density <= 0.0001f) continue;

                float r = Vector2.Distance(particles[i].position, particles[n].position);
                if (r >= h) continue;

                // Muller Eqn (14): mu * m_j * (v_j - v_i) / rho_j * laplacian(W_visc)
                // The direction lives in (v_j - v_i), so the force always pulls this
                // particle's velocity toward the neighbour's.
                viscForce += viscosity
                             * particleMass
                             * (particles[n].velocity - particles[i].velocity)
                             / particles[n].density
                             * viscLapMag * (h - r);
            }

            viscosityForces[i] = viscForce;
            particles[i].force += viscForce;
        }
    }

    // Lays a layer of static particles inside the solid, just beyond each wall.
    // They exist only to complete the fluid's kernel support near the wall and to
    // push back when the fluid compresses against it.
    private void SpawnBoundaryParticles()
    {
        if (!useBoundaryParticles)
        {
            boundaryParticles = null;
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

        List<Vector2> list = new List<Vector2>();

        for (int y = 0; y < ny; y++)
        {
            for (int x = 0; x < nx; x++)
            {
                Vector2 pos = new Vector2(
                    center.x - half.x - margin + (x + 0.5f) * stepX,
                    center.y - half.y - margin + (y + 0.5f) * stepY);

                Vector2 rel = pos - center;

                // How far outside the fluid domain this sample sits
                float outsideX = Mathf.Max(0f, Mathf.Abs(rel.x) - half.x);
                float outsideY = Mathf.Max(0f, Mathf.Abs(rel.y) - half.y);

                // Skip samples inside the fluid domain
                if (outsideX <= 0f && outsideY <= 0f) continue;

                // Skip samples too deep into the solid to matter
                float distToWall = Mathf.Sqrt(outsideX * outsideX + outsideY * outsideY);
                if (distToWall > margin) continue;

                list.Add(pos);
            }
        }

        boundaryParticles = list.ToArray();
        boundaryVolume = stepX * stepY;
    }

    private int HeatMapResolutionClamped => Mathf.Clamp(heatMapResolution, 2, 512);

    // Semi-implicit (symplectic) Euler. Velocity is updated first, then position
    // uses the NEW velocity, which damps energy instead of injecting it -- that is
    // what makes this scheme usable for stiff pressure forces.
    //
    // The wall clamp here is now only a penetration safety net. The physics of the
    // wall is handled by the boundary particles in ComputeDensity and
    // ComputePressureForce; this just guarantees a particle can never escape if a
    // step does overshoot.
    private void Integrate(float dt)
    {
        Vector2 center = transform.position;
        float halfX = boxSize.x * 0.5f;
        float halfY = boxSize.y * 0.5f;

        for (int i = 0; i < particles.Length; i++)
        {
            // Isolated particles have no density, so they cannot be accelerated
            if (particles[i].density <= 0.0001f) continue;

            // acceleration = pressure force / density + gravity
            Vector2 acceleration = particles[i].force / particles[i].density + gravity;

            // Semi-implicit Euler: velocity first, then position with the new velocity
            particles[i].velocity += acceleration * dt;
            particles[i].position += particles[i].velocity * dt;

            // Simple wall clamp so particles stay inside the container
            Vector2 pos = particles[i].position;
            Vector2 vel = particles[i].velocity;

            float relX = pos.x - center.x;
            float relY = pos.y - center.y;

            if (relX > halfX)
            {
                pos.x = center.x + halfX;
                vel.x *= -collisionDamping;
            }
            else if (relX < -halfX)
            {
                pos.x = center.x - halfX;
                vel.x *= -collisionDamping;
            }

            if (relY > halfY)
            {
                pos.y = center.y + halfY;
                vel.y *= -collisionDamping;
            }
            else if (relY < -halfY)
            {
                pos.y = center.y - halfY;
                vel.y *= -collisionDamping;
            }

            particles[i].position = pos;
            particles[i].velocity = vel;
        }
    }

    // Builds a quad-grid mesh covering the container. Vertex colors are written
    // each frame; the GPU interpolates between them, which gives a smooth field.
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

    // Samples the pressure field at every mesh vertex and writes the vertex colors:
    // blue = below rest density, white = at rest density, red = above rest density
    private void UpdateHeatMapColors()
    {
        // Sample with a wider kernel than the physics kernel to smooth the field
        float sampleH = smoothingLength * heatMapBlur;
        float h2 = sampleH * sampleH;
        float poly6Const = 4f / (Mathf.PI * Mathf.Pow(sampleH, 8f));
        Vector2 center = transform.position;

        // Auto-range so the heat map always shows contrast
        float maxAbs = 1f;
        for (int i = 0; i < particles.Length; i++)
        {
            float absP = Mathf.Abs(particles[i].pressure);
            if (absP > maxAbs) maxAbs = absP;
        }

        for (int v = 0; v < heatMapVerts.Length; v++)
        {
            Vector2 sample = center + new Vector2(heatMapVerts[v].x, heatMapVerts[v].y);

            float weightedPressure = 0f;
            float weightSum = 0f;

            for (int i = 0; i < particles.Length; i++)
            {
                float r2 = (particles[i].position - sample).sqrMagnitude;
                if (r2 < h2)
                {
                    float diff = h2 - r2;
                    float w = poly6Const * diff * diff * diff;
                    weightedPressure += particles[i].pressure * w;
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

    private void OnDestroy()
    {
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
                                                                                   
    private void OnDrawGizmos()
    {
        
        if (particles == null) return;                                            
                                                                                 
       // 1. Draw container                                                      
       Gizmos.color = Color.blue;                                                
       Gizmos.DrawWireCube(transform.position, new Vector3(boxSize.x, boxSize.y, 0f));                                                                           

       // 1a. Boundary particles (static solid material just outside the walls)
       if (useBoundaryParticles && boundaryParticles != null)
       {
           Gizmos.color = new Color(0.4f, 0.4f, 0.4f, 1f);
           for (int b = 0; b < boundaryParticles.Length; b++)
           {
               Gizmos.DrawSphere(new Vector3(boundaryParticles[b].x, boundaryParticles[b].y, 0f), 0.05f);
           }
       }

       // 1b. Pressure heat map across the whole canvas
       if (showPressureHeatMap)
       {
           DrawPressureHeatMap();
       }
                                                                                 
       // 2. Draw particles colored by density or pressure
       // Low value = blue, high value = red
       for (int i = 0; i < particles.Length; i++)
       {
           float value = showPressureColor ? particles[i].pressure : particles[i].density;
           float maxValue = showPressureColor ? stiffness * (restDensity * 0.5f) : restDensity * 1.5f;
           float t = Mathf.InverseLerp(0f, maxValue, value);
           Gizmos.color = Color.Lerp(Color.blue, Color.red, t);
           Gizmos.DrawSphere(new Vector3(particles[i].position.x, particles[i].position.y, 0f), 0.08f);
       }

       // 3. Force arrows
       // Direction = force direction. Color = force magnitude (weak -> strong)
       if (showViscosityArrows && viscosityForces != null)
       {
           float maxVisc = 1e-6f;
           for (int i = 0; i < viscosityForces.Length; i++)
           {
               float mag = viscosityForces[i].magnitude;
               if (mag > maxVisc) maxVisc = mag;
           }

           for (int i = 0; i < viscosityForces.Length; i++)
           {
               Vector2 vf = viscosityForces[i];
               float mag = vf.magnitude;
               if (mag < 1e-6f) continue;

               Vector3 viscOrigin = new Vector3(particles[i].position.x, particles[i].position.y, 0f);
               Vector3 viscDir = new Vector3(vf.x / mag, vf.y / mag, 0f);

               float viscT = Mathf.InverseLerp(0f, maxVisc, mag);
               Gizmos.color = Color.Lerp(Color.yellow, new Color(1f, 0.35f, 0f), viscT);
               Gizmos.DrawLine(viscOrigin, viscOrigin + viscDir * forceArrowLength);
           }
       }
       else if (showForceArrows)
       {
           float maxForce = 1e-6f;
           for (int i = 0; i < particles.Length; i++)
           {
               float mag = particles[i].force.magnitude;
               if (mag > maxForce) maxForce = mag;
           }

           for (int i = 0; i < particles.Length; i++)
           {
               Vector2 f = particles[i].force;
               float mag = f.magnitude;
               if (mag < 1e-6f) continue;

               Vector3 origin = new Vector3(particles[i].position.x, particles[i].position.y, 0f);
               Vector3 dir = new Vector3(f.x / mag, f.y / mag, 0f);

               float magT = Mathf.InverseLerp(0f, maxForce, mag);
               Gizmos.color = Color.Lerp(Color.cyan, Color.magenta, magT);
               Gizmos.DrawLine(origin, origin + dir * forceArrowLength);
           }
       }                                                                          
                                                                                  
       // DEBUG: draw only one debug particle and its neighbors
       // Purpose: lets you see which particle owns which neighbor connections
       if (neighbors != null && neighbors.Length > 0)
       {
           int target = Mathf.Clamp(debugParticle, 0, particles.Length - 1);

           // Highlight debug particle in red
           Gizmos.color = Color.red;
           Gizmos.DrawWireSphere(
               new Vector3(particles[target].position.x, particles[target].position.y, 0f),
               0.12f
           );

           // Draw lines from debug particle to its neighbors in green
           Gizmos.color = Color.green;
           for (int j = 0; j < neighbors[target].Count; j++)
           {
               int neighborIndex = neighbors[target][j];
               Gizmos.DrawLine(
                   new Vector3(particles[target].position.x, particles[target].position.y, 0f),
                   new Vector3(particles[neighborIndex].position.x, particles[neighborIndex].position.y, 0f)
               );
           }
       }                                                                          
}                                                                                   
}
using UnityEngine;
using System.Collections.Generic;


public struct Particle2D
{
    public Vector2 position;
    public Vector2 velocity;
    public Vector2 force;
    public float density;
    public float pressure;
}

public class SPH2D : MonoBehaviour
{
    public Vector2 boxSize = new Vector2(10f, 10f);

    public Vector2Int numToSpawn = new Vector2Int(10, 10);
    public Vector2 spawnCenter = new Vector2(0f, 2f);
    public float particleSpacing = 0.4f;

    // Gravity is disabled for now so the pressure force can be observed on its own.
    // Set it back to (0, -9.81) to bring gravity back.
    public Vector2 gravity = new Vector2(0f, 0f);
    public float collisionDamping = 0.5f;

    public float smoothingLength = 2f;
    public int debugParticle = 0;

    [Header("Simulation")]
    public int subSteps = 8;

    [Header("Density")]
    public float restDensity = 1000f;
    public bool autoParticleMass = true;
    public float particleMass = 1f;

    [Header("Pressure")]
    public float stiffness = 2000f;
    public bool clampPressurePositive = true;
    public bool showPressureColor = true;

    [Header("Spawn")]
    public bool useRandomSpawn = true;

    [Header("Debug Visualization")]
    public bool debugLogs = false;
    public bool showForceArrows = true;
    public float forceArrowLength = 0.4f;

    [Header("Pressure Heat Map")]
    public bool showPressureHeatMap = true;
    public int heatMapResolution = 64;
    [Range(1f, 3f)] public float heatMapBlur = 1.5f;
    [Range(0f, 1f)] public float heatMapOpacity = 0.6f;

    private Particle2D[] particles;
    private List<int>[] neighbors;

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
            particleMass = restDensity / meanDensity;
        }
    }

    private void SpawnParticles()
    {
        particles = new Particle2D[TotalParticles];

        // Random scatter with a minimum separation (blue noise).
        // Pure uniform random creates dense clumps and empty voids. The local
        // density then swings far from restDensity and the pressure force spikes.
        // Enforcing a minimum separation keeps the packing even while the layout
        // still looks irregular.
        if (useRandomSpawn)
        {
            Vector2 center = transform.position;

            float area = boxSize.x * boxSize.y;
            float nominalSpacing = Mathf.Sqrt(area / Mathf.Max(1, TotalParticles));
            float minDist = nominalSpacing * 0.85f;
            float minDistSq = minDist * minDist;
            const int maxAttempts = 30;

            for (int i = 0; i < TotalParticles; i++)
            {
                Vector2 candidate = center;

                for (int attempt = 0; attempt < maxAttempts; attempt++)
                {
                    candidate = center + new Vector2(
                        Random.Range(-boxSize.x * 0.5f, boxSize.x * 0.5f),
                        Random.Range(-boxSize.y * 0.5f, boxSize.y * 0.5f)
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
            return;
        }

        int index = 0;

        float gridWidth = (numToSpawn.x - 1) * particleSpacing;
        float gridHeight = (numToSpawn.y - 1) * particleSpacing;

        Vector2 origin = spawnCenter - new Vector2(gridWidth * 0.5f, gridHeight * 0.5f);

        for (int y = 0; y < numToSpawn.y; y++)
        {
            for (int x = 0; x < numToSpawn.x; x++)
            {
                Vector2 position = origin + new Vector2(x * particleSpacing, y * particleSpacing);
                particles[index] = new Particle2D
                {
                    position = position,
                    velocity = Vector2.zero,
                    density = 0f,
                    pressure = 0f
                };
                index++;
            }
        }
    }

    private void Update()
    {
        if (particles == null)
            return;
        
        // Run the solver in small substeps so explicit integration stays stable.
        // The whole solver still advances by Time.deltaTime per frame in total.
        int steps = Mathf.Max(1, subSteps);
        float dt = Time.deltaTime / steps;

        for (int s = 0; s < steps; s++)
        {
            FindNeighbors();
            ComputeDensity();
            ComputePressure();
            ComputePressureForce();

            // TEMPORARY integration + wall clamp so you can see motion now.
            // Proper versions arrive in T-020 (boundaries) and T-021 (integration scheme).
            Integrate(dt);
        }

        // DEBUG: log density/pressure per particle so you can verify the Tait EOS
        if (debugLogs)
        {
            for (int i = 0; i < particles.Length; i++)
            {
                Debug.Log($"Particle {i}: density={particles[i].density:F2}, pressure={particles[i].pressure:F2}");
            }
        }

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
       for (int i = 0; i < particles.Length; i++)                                              
       {                                                                                       
           float density = 0f;                                                                 
                                                                                               
           for (int j = 0; j < neighbors[i].Count; j++)                                        
           {                                                                                   
               int neighborIndex = neighbors[i][j];                                            
               float r = Vector2.Distance(particles[i].position, particles[neighborIndex].position);                                                           
                                                                                               
               if (r < smoothingLength && r > 0f)                                              
               {                                                                               
                   float diff = smoothingLength * smoothingLength - r * r;                     
                   density += particleMass * (4f / (Mathf.PI * Mathf.Pow(smoothingLength, 8f))) * Mathf.Pow(diff, 3f);                                                                  
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
            // Forces are accumulated fresh each frame
            particles[i].force = Vector2.zero;

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
        }
    }

    private int HeatMapResolutionClamped => Mathf.Clamp(heatMapResolution, 2, 512);

    // TEMPORARY: semi-implicit Euler with a simple wall clamp.
    // T-020 will replace the boundary handling and T-021 the integration scheme.
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
       // Direction = force direction. Color = force magnitude (cyan = weak, magenta = strong)
       if (showForceArrows)
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
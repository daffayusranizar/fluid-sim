using UnityEngine;
using System.Collections.Generic;


public struct Particle2D
{
    public Vector2 position;
    public Vector2 velocity;
    public float density;
    public float pressure;
}

public class SPH2D : MonoBehaviour
{
    public Vector2 boxSize = new Vector2(10f, 10f);

    public Vector2Int numToSpawn = new Vector2Int(10, 10);
    public Vector2 spawnCenter = new Vector2(0f, 2f);
    public float particleSpacing = 0.4f;

    public Vector2 gravity = new Vector2(0f, -9.81f);
    public float collisionDamping = 0.5f;

    public float smoothingLength = 1f;
    public int debugParticle = 0;

    [Header("Density")]
    public float restDensity = 1000f;
    public float particleMass = 0.1f;

    [Header("Pressure")]
    public float stiffness = 2000f;
    public bool showPressureColor = true;

    [Header("Spawn")]
    public bool useRandomSpawn = true;

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
    }

    private void SpawnParticles()
    {
        particles = new Particle2D[TotalParticles];

        // Random scatter across the whole canvas
        if (useRandomSpawn)
        {
            Vector2 center = transform.position;

            for (int i = 0; i < TotalParticles; i++)
            {
                Vector2 position = center + new Vector2(
                    Random.Range(-boxSize.x * 0.5f, boxSize.x * 0.5f),
                    Random.Range(-boxSize.y * 0.5f, boxSize.y * 0.5f)
                );

                particles[i] = new Particle2D
                {
                    position = position,
                    velocity = Vector2.zero,
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
        
        float dt = Time.deltaTime;
        // float halfX = boxSize.x * 0.5f;
        // float halfY = boxSize.y * 0.5f;

        FindNeighbors();
        ComputeDensity();
        ComputePressure();

        for (int i = 0; i < particles.Length; i++)
        {
            
        }

        // DEBUG: log pressure per particle so you can verify Tait EOS
       for (int i = 0; i < particles.Length; i++)
       {
           Debug.Log($"Particle {i}: density={particles[i].density:F2}, pressure={particles[i].pressure:F2}");
       }

       // debug print

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
       for (int i = 0; i < n; i++)                                                
       {                                                                         
           Debug.Log($"Particle {i}: {neighbors[i].Count} neighbors");           
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
            particles[i].pressure = stiffness * (particles[i].density - restDensity);
        }
    }

    private int HeatMapResolutionClamped => Mathf.Clamp(heatMapResolution, 2, 512);

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
           Gizmos.DrawWireSphere(new Vector3(particles[i].position.x, particles[i].position.y, 0f), 0.08f);
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
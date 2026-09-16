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

    private Particle2D[] particles;
    private List<int>[] neighbors;

    public int TotalParticles => numToSpawn.x * numToSpawn.y;

    private void Awake()
    {
        SpawnParticles();
    }

    private void SpawnParticles()
    {
        particles = new Particle2D[TotalParticles];

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

        for (int i = 0; i < particles.Length; i++)
        {
            
        }

        // DEBUG: log density per particle so you can verify density computation
       for (int i = 0; i < particles.Length; i++)
       {
           Debug.Log($"Particle {i}: density={particles[i].density:F2}");
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
                                                                                   
    private void OnDrawGizmos()
    {
        
        if (particles == null) return;                                            
                                                                                 
       // 1. Draw container                                                      
       Gizmos.color = Color.blue;                                                
       Gizmos.DrawWireCube(transform.position, new Vector3(boxSize.x, boxSize.y, 0f));                                                                           
                                                                                 
       // 2. Draw particles colored by density
       // Low density = blue, high density = red
       for (int i = 0; i < particles.Length; i++)
       {
           float t = Mathf.InverseLerp(0f, restDensity * 1.5f, particles[i].density);
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
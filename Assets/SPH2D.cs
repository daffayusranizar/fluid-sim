using UnityEngine;
using System.Runtime.InteropServices;

[System.Serializable]
[StructLayout(LayoutKind.Sequential, Size = 32)]
public struct Particle2D
{
    public Vector2 position;
    public Vector2 velocity;
    public Vector2 force;
    public float density;
    public float pressure;
}

[ExecuteAlways]
public class SPH2D : MonoBehaviour
{
    [Header("Container Settings")]
    public Vector2 boxSize = new Vector2(10f, 10f);

    [Header("Particle Spawning")]
    public Vector2Int numToSpawn = new Vector2Int(10, 10); // 100 particles
    public Vector2 spawnCenter = new Vector2(0f, 2f);
    public float particleSpacing = 0.4f;

    [Header("Physics Settings")]
    public Vector2 gravity = new Vector2(0f, -9.81f);
    public float collisionDamping = 0.5f; // Bounciness loss on collision

    private Particle2D[] particles;
    public int TotalParticles => numToSpawn.x * numToSpawn.y;

    private void Awake()
    {
        SpawnParticles();
    }

    private void SpawnParticles()
    {
        particles = new Particle2D[TotalParticles];

        Vector2 spawnOrigin = spawnCenter - new Vector2(
            (numToSpawn.x - 1) * particleSpacing * 0.5f,
            (numToSpawn.y - 1) * particleSpacing * 0.5f
        );

        int index = 0;
        for (int x = 0; x < numToSpawn.x; x++)
        {
            for (int y = 0; y < numToSpawn.y; y++)
            {
                Vector2 pos = spawnOrigin + new Vector2(x * particleSpacing, y * particleSpacing);

                particles[index] = new Particle2D
                {
                    position = pos,
                    velocity = Vector2.zero,
                    force = Vector2.zero,
                    density = 0f,
                    pressure = 0f
                };
                index++;
            }
        }
    }

    private void Update()
    {
        if (particles == null) return;

        float dt = Time.deltaTime;
        Vector2 halfBox = boxSize * 0.5f;

        for (int i = 0; i < particles.Length; i++)
        {
            // 1. Apply gravity to velocity
            particles[i].velocity += gravity * dt;

            // 2. Update position based on velocity
            particles[i].position += particles[i].velocity * dt;

            // 3. Resolve collisions with blue box container walls
            Vector2 pos = particles[i].position;
            Vector2 vel = particles[i].velocity;

            // Left / Right walls
            if (Mathf.Abs(pos.x) > halfBox.x)
            {
                pos.x = Mathf.Sign(pos.x) * halfBox.x;
                vel.x *= -collisionDamping;
            }

            // Top / Bottom walls
            if (Mathf.Abs(pos.y) > halfBox.y)
            {
                pos.y = Mathf.Sign(pos.y) * halfBox.y;
                vel.y *= -collisionDamping;
            }

            particles[i].position = pos;
            particles[i].velocity = vel;
        }
    }

    private void OnDrawGizmos()
    {
        // 1. Draw container box
        Gizmos.color = Color.blue;
        Gizmos.DrawWireCube(transform.position, new Vector3(boxSize.x, boxSize.y, 0f));

        // 2. Draw spawn region (only in editor when not playing)
        if (!Application.isPlaying)
        {
            Gizmos.color = Color.cyan;
            Vector3 spawnSize = new Vector3(numToSpawn.x * particleSpacing, numToSpawn.y * particleSpacing, 0f);
            Gizmos.DrawWireCube(spawnCenter, spawnSize);
        }

        // 3. Draw particles
        if (particles != null)
        {
            Gizmos.color = Color.yellow;
            foreach (var p in particles)
            {
                Gizmos.DrawWireSphere(p.position, 0.08f);
            }
        }
    }
}

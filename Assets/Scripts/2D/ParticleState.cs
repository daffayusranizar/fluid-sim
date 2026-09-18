using Unity.Collections;
using Unity.Mathematics;

/// <summary>
/// Particle state in structure-of-arrays form: one NativeArray per field rather
/// than one array of structs.
///
/// Why this rather than Particle2D[]:
///
/// 1. No data race. Each pass writes exactly one array and reads others. With an
///    array of structs, writing particles[i] rewrites position and velocity too,
///    while other threads are reading particles[j].position as a neighbour. That
///    race is benign in practice -- the rewritten values are unchanged -- but it
///    is still a race, and correctness should not rest on a coincidence.
///
/// 2. No aliasing. When the same array is both read and written, Burst must
///    assume the pointers may overlap, which blocks vectorisation. Separate
///    arrays remove that ambiguity, and vectorisation is the reason Burst is here.
///
/// 3. It is where T-026 is heading anyway. Compute shaders want separate buffers
///    per attribute, so this is the layout the GPU path will use.
///
/// Particle2D still exists and is still the data contract -- but now as the
/// *render* view, repacked once per frame into a form the shader can read.
/// </summary>
public struct ParticleState
{
    public NativeArray<float2> position;
    public NativeArray<float2> velocity;
    public NativeArray<float2> force;

    public NativeArray<float> density;
    public NativeArray<float> pressure;

    public NativeArray<float> nearDensity;
    public NativeArray<float> nearPressure;

    public int Count => position.Length;
    public bool IsCreated => position.IsCreated;

    public ParticleState(int count, Allocator allocator)
    {
        position = new NativeArray<float2>(count, allocator);
        velocity = new NativeArray<float2>(count, allocator);
        force = new NativeArray<float2>(count, allocator);
        density = new NativeArray<float>(count, allocator);
        pressure = new NativeArray<float>(count, allocator);
        nearDensity = new NativeArray<float>(count, allocator);
        nearPressure = new NativeArray<float>(count, allocator);
    }

    public void Dispose()
    {
        DisposeIfCreated(position);
        DisposeIfCreated(velocity);
        DisposeIfCreated(force);
        DisposeIfCreated(density);
        DisposeIfCreated(pressure);
        DisposeIfCreated(nearDensity);
        DisposeIfCreated(nearPressure);
    }

    private static void DisposeIfCreated<T>(NativeArray<T> array) where T : struct
    {
        if (array.IsCreated) array.Dispose();
    }
}

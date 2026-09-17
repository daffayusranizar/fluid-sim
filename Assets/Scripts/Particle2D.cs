using Unity.Mathematics;

/// <summary>
/// Per-particle simulation state.
///
/// This is the data contract of the solver. It is 40 bytes (10 floats), and at
/// T-026 the GPU buffer and HLSL struct have to match it field for field.
///
/// It uses float2 rather than Vector2 for two reasons: Unity.Mathematics types
/// are translated to native SIMD vectors by Burst, and float2 is exactly the
/// type HLSL uses, so the GPU buffer at T-026 needs no translation.
/// </summary>
public struct Particle2D
{
    public float2 position;
    public float2 velocity;
    public float2 force;
    public float density;
    public float pressure;
    public float nearDensity;
    public float nearPressure;
}

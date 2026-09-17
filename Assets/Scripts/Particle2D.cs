using UnityEngine;

/// <summary>
/// Per-particle simulation state.
///
/// This is the data contract of the solver: every stage of the SPH loop reads
/// and writes these fields, and at T-026 this layout has to be mirrored exactly
/// by the GPU buffer and the HLSL struct. Keep the field order stable and keep
/// the type at 32 bytes (8 floats) so the CPU and GPU views stay in sync.
/// </summary>
public struct Particle2D
{
    public Vector2 position;
    public Vector2 velocity;
    public Vector2 force;
    public float density;
    public float pressure;
}

using System.Runtime.InteropServices;
using Unity.Mathematics;

/// <summary>
/// The interactive cursor as the solver sees it: a solid disc the fluid cannot
/// enter, carrying a velocity.
/// </summary>
/// <remarks>
/// A plain value, not a MonoBehaviour, and owned by neither solver. The same
/// struct is handed to the Burst integrate job as a field and uploaded to the
/// compute shader as constants, so both paths act on one definition of what the
/// pointer is. <see cref="SPHInteractor2D"/> produces it and pushes it to whoever
/// is stepping the simulation; the solvers only read what they were last given.
///
/// That direction matters. Reading input inside the solver would put a
/// MonoBehaviour and the new Input System on the Burst/GPU side of the fence, and
/// would make the simulation unrunnable without a mouse. Here, no interactor
/// simply means <see cref="active"/> stays false and the solver behaves exactly as
/// it did before this existed.
///
/// The bool is marshalled as one byte for the same reason it is in
/// <see cref="SolverParams"/>: a plain bool has no fixed size in the CLR, which
/// makes the struct non-blittable and silently pushes <c>[BurstCompile]</c> direct
/// calls back to managed code rather than failing loudly.
/// </remarks>
public struct PointerState
{
    /// <summary>Centre of the disc, in the same space as particle positions.</summary>
    public float2 position;

    /// <summary>
    /// Velocity of the disc. Fluid in contact is resolved against this rather than
    /// against world zero, which is what lets the disc carry water instead of only
    /// pushing it away.
    /// </summary>
    public float2 velocity;

    /// <summary>Disc radius, in world units.</summary>
    public float radius;

    /// <summary>Restitution of the contact. 0 is fully inelastic, 1 is a bounce.</summary>
    public float restitution;

    /// <summary>Tangential drag toward the disc's own velocity, 0..1.</summary>
    public float friction;

    /// <summary>
    /// False leaves particles untouched, so the disc can be tracked and drawn
    /// without being felt.
    /// </summary>
    [MarshalAs(UnmanagedType.U1)] public bool active;
}

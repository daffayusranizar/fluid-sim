using Unity.Burst;
using Unity.Mathematics;

/// <summary>
/// Pure smoothing-kernel and equation-of-state math.
///
/// Every method here is stateless and Burst compiled, and nothing in this file
/// knows about particles or neighbours, so the same formulas can be reused by
/// the CPU solver now and translated into HLSL at T-026.
///
/// There are two pressure channels, following the double-density relaxation
/// scheme (Clavet et al. 2005) rather than Muller's single-density one:
///
///   main channel : SpikyPow2 density -> Tait pressure -> linear gradient
///                  can go negative, which is what gives a two-way restoring
///                  force and therefore a real equilibrium
///   near channel : SpikyPow3 density -> near pressure -> quadratic gradient
///                  never negative, acts only at short range, and is what stops
///                  the tensile instability the moment the main channel pulls
///
/// The two gradients differ on purpose. Near/main force ratio runs about
/// 2.5x at r = 0 and falls to 0 at r = h, so the near term wins close in and
/// vanishes far out.
///
/// Poly6 is still here, but now serves viscosity rather than density: it is the
/// weight in the XSPH velocity blend, and being non-negative everywhere is
/// exactly what makes that blend incapable of adding energy.
///
/// All normalisation constants are 2D and are computed once per step by the
/// caller, because evaluating math.pow per particle pair would dominate the cost.
/// </summary>
[BurstCompile]
public static class SPHMath
{
    // ------------------------------------------------------------------
    // Main channel: SpikyPow2
    // ------------------------------------------------------------------

    /// <summary>2D Spiky Pow2 kernel value: 6/(pi h^4) * (h - r)^2.</summary>
    [BurstCompile]
    public static float SpikyPow2Kernel(float r, float h, float const2)
    {
        if (r >= h) return 0f;

        float hr = h - r;
        return const2 * hr * hr;
    }

    /// <summary>Gradient magnitude of SpikyPow2: 12/(pi h^4) * (h - r). Linear.</summary>
    [BurstCompile]
    public static float SpikyPow2GradientMagnitude(float r, float h, float const2Grad)
    {
        return const2Grad * (h - r);
    }

    /// <summary>2D Spiky Pow2 normalisation: 6 / (pi h^4).</summary>
    [BurstCompile]
    public static float SpikyPow2Constant(float h)
    {
        return 6f / (math.PI * math.pow(h, 4f));
    }

    /// <summary>2D Spiky Pow2 gradient normalisation: 12 / (pi h^4).</summary>
    [BurstCompile]
    public static float SpikyPow2GradientConstant(float h)
    {
        return 12f / (math.PI * math.pow(h, 4f));
    }

    // ------------------------------------------------------------------
    // Near channel: SpikyPow3
    // ------------------------------------------------------------------

    /// <summary>2D Spiky Pow3 kernel value: 10/(pi h^5) * (h - r)^3.</summary>
    [BurstCompile]
    public static float SpikyPow3Kernel(float r, float h, float const3)
    {
        if (r >= h) return 0f;

        float hr = h - r;
        return const3 * hr * hr * hr;
    }

    /// <summary>Gradient magnitude of SpikyPow3: 30/(pi h^5) * (h - r)^2.</summary>
    /// <remarks>
    /// Stays non-zero as r approaches 0. Combined with a pressure that can never
    /// be negative, this is what guarantees repulsion exactly when particles are
    /// on top of each other.
    /// </remarks>
    [BurstCompile]
    public static float SpikyPow3GradientMagnitude(float r, float h, float const3Grad)
    {
        float hr = h - r;
        return const3Grad * hr * hr;
    }

    /// <summary>2D Spiky Pow3 normalisation: 10 / (pi h^5).</summary>
    [BurstCompile]
    public static float SpikyPow3Constant(float h)
    {
        return 10f / (math.PI * math.pow(h, 5f));
    }

    /// <summary>2D Spiky Pow3 gradient normalisation: 30 / (pi h^5).</summary>
    [BurstCompile]
    public static float SpikyPow3GradientConstant(float h)
    {
        return 30f / (math.PI * math.pow(h, 5f));
    }

    /// <summary>
    /// Near pressure. Deliberately has no rest-density offset, so it is zero only
    /// when a particle has no close neighbours at all and can never become
    /// attractive.
    /// </summary>
    [BurstCompile]
    public static float NearPressure(float nearDensity, float multiplier)
    {
        return multiplier * nearDensity;
    }

    // ------------------------------------------------------------------
    // Viscosity weight: Poly6
    // ------------------------------------------------------------------

    /// <summary>2D Poly6 kernel value. Used as the XSPH viscosity weight.</summary>
    /// <remarks>
    /// Non-negative everywhere, which is the property that matters here: a
    /// weighted average toward neighbours' velocities cannot add energy no matter
    /// how tightly particles are packed. Muller's Laplacian formulation needs a
    /// dedicated kernel precisely because a Laplacian can change sign; a weight
    /// cannot.
    /// </remarks>
    [BurstCompile]
    public static float Poly6Kernel(float r2, float h2, float poly6Const)
    {
        if (r2 >= h2) return 0f;

        float diff = h2 - r2;
        return poly6Const * diff * diff * diff;
    }

    /// <summary>2D Poly6 normalisation: 4 / (pi h^8).</summary>
    [BurstCompile]
    public static float Poly6Constant(float h)
    {
        return 4f / (math.PI * math.pow(h, 8f));
    }

    // ------------------------------------------------------------------
    // Equation of state
    // ------------------------------------------------------------------

    /// <summary>
    /// Tait equation of state, linear form: p = k * (density - restDensity).
    /// </summary>
    /// <remarks>
    /// Clamping removes negative pressure, and with it the cohesion that makes
    /// particles string together at free surfaces -- but it also removes one
    /// direction of correction, so the fluid can no longer reach a true
    /// equilibrium. With the near channel present this can now be left off, since
    /// short-range repulsion no longer depends on the sign of this term.
    /// </remarks>
    [BurstCompile]
    public static float Pressure(float density, float restDensity, float stiffness, bool clampPositive)
    {
        float p = stiffness * (density - restDensity);

        if (clampPositive && p < 0f) p = 0f;

        return p;
    }

    // ------------------------------------------------------------------
    // Interaction
    // ------------------------------------------------------------------

    /// <summary>
    /// Resolves one particle against the interactive pointer disc.
    /// </summary>
    /// <remarks>
    /// A position constraint rather than a force, and that choice is what makes
    /// the interaction safe. A force applied inside a radius would inject energy
    /// the CFL step then has to absorb: peak speed rises, dt shrinks, and at a
    /// fixed substep cap the fluid silently starts running in slow motion instead
    /// of visibly breaking. Clamping position and resolving velocity against the
    /// disc's own motion cannot do that -- the disc carries at most the velocity it
    /// was given, and it was given a capped one.
    ///
    /// The disc is deliberately absolute: it ignores <c>p.restDensity</c> and the
    /// pressure field entirely, so contact does not depend on being near rest
    /// density and stays predictable at a free surface.
    ///
    /// The GPU twin of this function is <c>ApplyPointer</c> in SPH2D.compute. The
    /// two must stay in step, or the CPU and GPU solvers stop being comparable,
    /// which is the only reason to keep both paths at all.
    /// </remarks>
    [BurstCompile]
    public static void ResolvePointerCollision(
        in PointerState pointer,
        ref float2 position,
        ref float2 velocity)
    {
        if (!pointer.active) return;

        float radius = math.max(1e-4f, pointer.radius);
        float2 toParticle = position - pointer.position;
        float distanceSq = math.lengthsq(toParticle);

        if (distanceSq >= radius * radius) return;

        float distance = math.sqrt(distanceSq);

        // Exactly on the centre there is no direction to push along. Any direction
        // is better than dividing by zero here: a NaN would propagate into the
        // density and force of every particle that touches it.
        float2 normal = distance > 1e-6f ? toParticle / distance : new float2(0f, 1f);

        // The disc is solid. Snapping to the surface is what ejects particles that
        // were already inside when it appeared, which is what a tap feels like.
        position = pointer.position + normal * radius;

        // Everything below is relative to the disc, so a disc moving at velocity V
        // transfers V to the fluid it touches rather than acting like a static wall.
        float2 relative = velocity - pointer.velocity;
        float inbound = math.dot(relative, normal);

        // Only remove motion going into the disc. An outward-moving particle keeps
        // its speed, so the disc can never brake fluid that is already leaving.
        if (inbound < 0f)
        {
            relative -= (1f + pointer.restitution) * inbound * normal;
        }

        // Tangential drag is the difference between poking the fluid and scooping
        // it. Without this term a fast drag slides through the water and the fluid
        // only ever moves radially out of the way, which reads as stirring air.
        float2 normalPart = math.dot(relative, normal) * normal;
        float2 tangentPart = relative - normalPart;

        velocity = pointer.velocity + normalPart + tangentPart * (1f - math.saturate(pointer.friction));
    }
}

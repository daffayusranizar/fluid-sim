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
}

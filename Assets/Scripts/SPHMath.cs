using Unity.Burst;
using Unity.Mathematics;

/// <summary>
/// Pure smoothing-kernel and equation-of-state math.
///
/// Every method here is stateless and Burst compiled. Nothing in this file knows
/// about particles, neighbours or the grid, which means the same formulas can be
/// reused by the CPU solver now and translated into HLSL at T-026 without
/// re-deriving anything.
///
/// The normalisation constants are 2D. They are computed once per step by the
/// caller and passed in, because evaluating math.pow per particle pair would
/// dominate the cost.
/// </summary>
[BurstCompile]
public static class SPHMath
{
    /// <summary>2D Poly6 kernel value. Used only for density.</summary>
    /// <remarks>
    /// Smooth, and its gradient vanishes at r = 0. That is fine for a weighted
    /// average of mass, and is exactly why it must never be used for pressure:
    /// overlapping particles would feel no repulsion and would clump.
    /// </remarks>
    [BurstCompile]
    public static float Poly6Kernel(float r2, float h2, float poly6Const)
    {
        if (r2 >= h2) return 0f;

        float diff = h2 - r2;
        return poly6Const * diff * diff * diff;
    }

    /// <summary>
    /// Magnitude of the 2D Spiky gradient: const * (h - r)^2, with
    /// const = 30 / (pi h^5).
    /// </summary>
    /// <remarks>
    /// Stays non-zero as r approaches 0, which is the whole point of this kernel:
    /// the repulsion does not vanish exactly when particles are on top of each
    /// other.
    /// </remarks>
    [BurstCompile]
    public static float SpikyGradientMagnitude(float r, float h, float spikyConst)
    {
        float hr = h - r;
        return spikyConst * hr * hr;
    }

    /// <summary>
    /// Laplacian magnitude of the 2D viscosity kernel: const * (h - r), with
    /// const = 40 / (pi h^5).
    /// </summary>
    /// <remarks>
    /// Positive everywhere for 0 &lt;= r &lt;= h. That guarantees the viscosity
    /// force always shrinks velocity differences rather than growing them, which
    /// is what makes it dissipative instead of an energy source.
    /// </remarks>
    [BurstCompile]
    public static float ViscosityLaplacianMagnitude(float r, float h, float viscConst)
    {
        return viscConst * (h - r);
    }

    /// <summary>
    /// Tait equation of state, linear form: p = k * (density - restDensity).
    /// </summary>
    /// <remarks>
    /// Clamping removes negative pressure, and with it the cohesion that makes
    /// particles string together at free surfaces. It also removes one direction
    /// of correction, so the fluid can no longer reach a true equilibrium -- a
    /// deliberate trade recorded in T-017 and T-018.
    /// </remarks>
    [BurstCompile]
    public static float Pressure(float density, float restDensity, float stiffness, bool clampPositive)
    {
        float p = stiffness * (density - restDensity);

        if (clampPositive && p < 0f) p = 0f;

        return p;
    }

    /// <summary>2D Poly6 normalisation: 4 / (pi h^8).</summary>
    [BurstCompile]
    public static float Poly6Constant(float h)
    {
        return 4f / (math.PI * math.pow(h, 8f));
    }

    /// <summary>2D Spiky gradient normalisation: 30 / (pi h^5).</summary>
    [BurstCompile]
    public static float SpikyGradientConstant(float h)
    {
        return 30f / (math.PI * math.pow(h, 5f));
    }

    /// <summary>2D viscosity Laplacian normalisation: 40 / (pi h^5).</summary>
    [BurstCompile]
    public static float ViscosityLaplacianConstant(float h)
    {
        return 40f / (math.PI * math.pow(h, 5f));
    }
}

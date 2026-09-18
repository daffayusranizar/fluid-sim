using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

/// <summary>
/// Everything the solver needs to know that is not particle state.
///
/// Grouping these into one struct has two benefits: jobs and Burst methods take
/// one value instead of a dozen arguments, and at T-026 this becomes the compute
/// shader's constant buffer almost verbatim.
/// </summary>
public struct SolverParams
{
    // --- Kernel geometry (derived from smoothingLength at spawn) ---
    public float smoothingLength;
    public float smoothingLengthSq;

    public float poly6Const;            // viscosity weight

    public float spikyPow2Const;        // main channel density
    public float spikyPow2GradConst;    // main channel force

    public float spikyPow3Const;        // near channel density
    public float spikyPow3GradConst;    // near channel force

    // --- Density and pressure ---
    public float restDensity;
    public float particleMass;
    public float stiffness;

    // Burst cannot pass a plain bool through a direct-call function pointer: a bool
    // has no fixed size in the CLR. Marshalling it as one byte makes the struct
    // blittable so the [BurstCompile] direct calls can be compiled instead of
    // silently falling back to managed code.
    [MarshalAs(UnmanagedType.U1)] public bool clampPressurePositive;

    // --- Near pressure (double density relaxation) ---
    public float nearPressureMultiplier;

    // --- Viscosity ---
    public float viscosity;

    // --- Boundary particles ---
    public float boundaryVolume;
    [MarshalAs(UnmanagedType.U1)] public bool useBoundaryParticles;

    // --- Container and integration ---
    public float2 gravity;
    public float2 boxCenter;
    public float2 boxHalfSize;
    public float collisionDamping;
}

/// <summary>
/// The parts of the solver that are not per-particle passes.
///
/// The passes themselves live in SPHJobs as IJobParallelFor implementations. What
/// remains here is the reduction work -- finding a maximum or a mean over all
/// particles -- plus the parameter block.
///
/// These are still single-threaded Burst methods rather than jobs. They are O(N)
/// with a tiny constant, and the time-step one runs before the solver each step, so
/// parallelising them would cost more in scheduling than it saves. A two-stage
/// reduction job would be the move if that ever changes.
/// </summary>
[BurstCompile]
public static class SPHSolver
{
    /// <summary>
    /// Largest step that keeps the explicit integration stable.
    /// </summary>
    /// <remarks>
    /// CFL limit: information must not cross more than one smoothing length per
    /// step, so dt &lt;= C * h / (maxSpeed + soundSpeed). The sound speed comes
    /// from the equation of state: p = k(rho - rho0) gives dp/drho = k, so
    /// c = sqrt(k). A stiffer fluid forces a smaller step.
    ///
    /// Acceleration limit: a particle must not be flung across its own
    /// neighbourhood within a single step.
    ///
    /// Note this reads only the equation of state's sound speed, not the
    /// discretised force's. Changing the kernels changes the latter, so if the
    /// simulation ever destabilises while cflFactor looks generous, this estimate
    /// is the first thing to re-derive.
    /// </remarks>
    [BurstCompile]
    public static float ComputeStableTimeStep(
        in ParticleState state,
        in SolverParams p,
        float cflFactor)
    {
        float soundSpeed = math.sqrt(math.max(0f, p.stiffness));
        float maxSpeed = 0f;
        float maxAccel = 0f;

        int count = state.Count;

        for (int i = 0; i < count; i++)
        {
            float speed = math.length(state.velocity[i]);
            if (speed > maxSpeed) maxSpeed = speed;

            float density = state.density[i];

            if (density > 0.0001f)
            {
                float accel = math.length(state.force[i] / density + p.gravity);
                if (accel > maxAccel) maxAccel = accel;
            }
        }

        float dtCfl = cflFactor * p.smoothingLength / (maxSpeed + soundSpeed + 0.0001f);

        float dtAcc = maxAccel > 0.0001f
            ? cflFactor * math.sqrt(p.smoothingLength / maxAccel)
            : float.MaxValue;

        return math.min(dtCfl, dtAcc);
    }

    /// <summary>Mean density over particles that have any density at all.</summary>
    /// <remarks>
    /// Used once at spawn to scale particle mass so the mean density lands on
    /// restDensity, which keeps the pressure field from starting far from rest.
    /// </remarks>
    [BurstCompile]
    public static float MeanDensity(in NativeArray<float> density)
    {
        float sum = 0f;
        int count = 0;

        for (int i = 0; i < density.Length; i++)
        {
            float d = density[i];

            if (d > 0f)
            {
                sum += d;
                count++;
            }
        }

        return count > 0 ? sum / count : 0f;
    }

    /// <summary>
    /// One pass reporting the numbers needed to tell a solver problem from a setup
    /// problem: mean density (is the fluid expanding or compressing?), peak speed
    /// (is anything running away?) and peak acceleration (are the forces sane
    /// relative to gravity?).
    /// </summary>
    [BurstCompile]
    public static void Diagnostics(
        in ParticleState state,
        in SolverParams p,
        out float meanDensity,
        out float maxSpeed,
        out float maxAccel)
    {
        float sum = 0f;
        meanDensity = 0f;
        maxSpeed = 0f;
        maxAccel = 0f;

        int count = state.Count;

        for (int i = 0; i < count; i++)
        {
            sum += state.density[i];

            float speed = math.length(state.velocity[i]);
            if (speed > maxSpeed) maxSpeed = speed;

            float density = state.density[i];

            if (density > 0.0001f)
            {
                float accel = math.length(state.force[i] / density + p.gravity);
                if (accel > maxAccel) maxAccel = accel;
            }
        }

        if (count > 0) meanDensity = sum / count;
    }
}

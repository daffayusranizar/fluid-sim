using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

/// <summary>
/// Everything the solver needs to know that is not particle state.
///
/// Grouping these into one struct has two benefits: Burst passes one value
/// instead of a dozen arguments, and at T-026 this struct becomes the compute
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
    public bool clampPressurePositive;

    // --- Near pressure (double density relaxation) ---
    public float nearPressureMultiplier;

    // --- Viscosity ---
    public float viscosity;

    // --- Boundary particles ---
    public float boundaryVolume;
    public bool useBoundaryParticles;

    // --- Container and integration ---
    public float2 gravity;
    public float2 boxCenter;
    public float2 boxHalfSize;
    public float collisionDamping;
}

/// <summary>
/// The SPH solver passes, all Burst compiled and all operating directly on
/// native arrays.
///
/// Each method owns exactly one stage of the pipeline, in the order they must
/// run: density, pressure, forces, integration. They are static and stateless,
/// which is what makes them both Burst-compilable and later parallelisable
/// without restructuring (T-025).
/// </summary>
[BurstCompile]
public static class SPHSolver
{
    /// <summary>
    /// Density by Poly6 summation over neighbours, plus the same summation over
    /// boundary particles.
    /// </summary>
    /// <remarks>
    /// Two density estimates are produced. The main one (SpikyPow2) drives the
    /// Tait pressure and can fall below rest density, which is what gives a
    /// two-way restoring force. The near one (SpikyPow3) drives an
    /// always-repulsive short-range term that stops the fluid collapsing when
    /// the main term pulls.
    ///
    /// The boundary term is what makes wall behaviour work. Near a wall roughly
    /// half of a particle's kernel support lies inside the solid, so without it
    /// the density is badly under-estimated, pressure collapses to zero, and
    /// particles weld themselves to the wall.
    /// </remarks>
    [BurstCompile]
    public static void ComputeDensity(
        NativeArray<Particle2D> particles,
        in NeighborLists neighbors,
        in NativeArray<float2> boundaryPositions,
        in SolverParams p)
    {
        float h = p.smoothingLength;

        for (int i = 0; i < particles.Length; i++)
        {
            Particle2D particle = particles[i];
            float2 posI = particle.position;
            float density = 0f;
            float nearDensity = 0f;

            int fluidStart = neighbors.fluidStart[i];
            int fluidEnd = neighbors.fluidStart[i + 1];

            for (int k = fluidStart; k < fluidEnd; k++)
            {
                float r = math.sqrt(math.distancesq(particles[neighbors.fluid[k]].position, posI));

                density += p.particleMass * SPHMath.SpikyPow2Kernel(r, h, p.spikyPow2Const);
                nearDensity += p.particleMass * SPHMath.SpikyPow3Kernel(r, h, p.spikyPow3Const);
            }

            // Boundary particles contribute to the main density only. The near
            // channel is about fluid particles crowded against each other; the
            // wall is handled by the main channel plus the position clamp.
            if (p.useBoundaryParticles)
            {
                int bStart = neighbors.boundaryStart[i];
                int bEnd = neighbors.boundaryStart[i + 1];

                for (int k = bStart; k < bEnd; k++)
                {
                    float r = math.sqrt(math.distancesq(boundaryPositions[neighbors.boundary[k]], posI));
                    density += p.restDensity * p.boundaryVolume
                               * SPHMath.SpikyPow2Kernel(r, h, p.spikyPow2Const);
                }
            }

            particle.density = density;
            particle.nearDensity = nearDensity;
            particles[i] = particle;
        }
    }

    /// <summary>Tait equation of state, one scalar per particle.</summary>
    [BurstCompile]
    public static void ComputePressure(NativeArray<Particle2D> particles, in SolverParams p)
    {
        for (int i = 0; i < particles.Length; i++)
        {
            Particle2D particle = particles[i];

            particle.pressure = SPHMath.Pressure(
                particle.density, p.restDensity, p.stiffness, p.clampPressurePositive);

            particle.nearPressure = SPHMath.NearPressure(
                particle.nearDensity, p.nearPressureMultiplier);

            particles[i] = particle;
        }
    }

    /// <summary>Zeroes the force accumulators. Must run before the force passes.</summary>
    [BurstCompile]
    public static void ResetForces(NativeArray<Particle2D> particles)
    {
        for (int i = 0; i < particles.Length; i++)
        {
            Particle2D particle = particles[i];
            particle.force = float2.zero;
            particles[i] = particle;
        }
    }

    /// <summary>
    /// Pressure force between neighbours, using the Spiky gradient, plus the
    /// boundary push.
    /// </summary>
    /// <remarks>
    /// Every term carries particleMass, because density is mass-scaled and the
    /// force has to be consistent with it. Without it the acceleration ends up
    /// proportional to 1/mass instead of being mass-independent, which is a
    /// large silent error the moment mass is not 1.
    ///
    /// Two channels are summed. The main channel uses the linear SpikyPow2
    /// gradient and can be attractive when the Tait pressure is negative. The
    /// near channel uses the quadratic SpikyPow3 gradient with a pressure that
    /// is never negative, so it is always repulsive and dominates at short
    /// range. Together they give a fluid that can hold itself together without
    /// collapsing into pairs.
    ///
    /// Sign: gradW points from i toward j, so with positive pressure the force
    /// must point from j to i, i.e. away from the neighbour. dir below is
    /// already that direction, hence the positive sign.
    ///
    /// For boundary particles the coefficient collapses. Muller's term is
    /// m_b * (p_i + p_b) / (2 rho_b); with m_b = restDensity * V_b, pressure
    /// mirroring p_b = p_i, and rho_b = restDensity, the whole thing reduces to
    /// V_b * p_i.
    /// </remarks>
    [BurstCompile]
    public static void ComputePressureForce(
        NativeArray<Particle2D> particles,
        in NeighborLists neighbors,
        in NativeArray<float2> boundaryPositions,
        in SolverParams p)
    {
        float h = p.smoothingLength;

        for (int i = 0; i < particles.Length; i++)
        {
            Particle2D particle = particles[i];
            float2 posI = particle.position;
            float2 force = particle.force;

            int fluidStart = neighbors.fluidStart[i];
            int fluidEnd = neighbors.fluidStart[i + 1];

            for (int k = fluidStart; k < fluidEnd; k++)
            {
                int n = neighbors.fluid[k];

                Particle2D other = particles[n];

                // Guard against zero density from isolated particles
                if (other.density <= 0.0001f) continue;

                float2 rVec = posI - other.position;
                float r2 = math.lengthsq(rVec);
                if (r2 <= 0f) continue;

                float r = math.sqrt(r2);
                float2 dir = rVec / r;

                // Main channel: symmetrised Tait pressure over the linear
                // SpikyPow2 gradient. This one can be attractive.
                float sharedPressure = (particle.pressure + other.pressure) * 0.5f;

                force += dir * (p.particleMass
                                * SPHMath.SpikyPow2GradientMagnitude(r, h, p.spikyPow2GradConst)
                                * sharedPressure / other.density);

                // Near channel: always repulsive, short range only. This is what
                // keeps the fluid from collapsing when the main term goes negative.
                if (other.nearDensity > 0.0001f)
                {
                    float sharedNearPressure = (particle.nearPressure + other.nearPressure) * 0.5f;

                    force += dir * (p.particleMass
                                    * SPHMath.SpikyPow3GradientMagnitude(r, h, p.spikyPow3GradConst)
                                    * sharedNearPressure / other.nearDensity);
                }
            }

            if (p.useBoundaryParticles)
            {
                // The wall may push but never pull. Pressure mirroring copies the
                // fluid particle's own pressure, and once the main channel is
                // allowed to go negative that value is often negative near a wall
                // (where density is below rest). A mirrored negative pressure
                // makes the boundary attractive: particles get sucked onto the
                // surface, the position clamp snaps them back, and the velocity
                // reflection throws them out. Clamping the boundary pressure is
                // the standard remedy for an under-resolved boundary layer.
                float pressure = math.max(0f, particle.pressure);

                int bStart = neighbors.boundaryStart[i];
                int bEnd = neighbors.boundaryStart[i + 1];

                for (int k = bStart; k < bEnd; k++)
                {
                    float2 rVec = posI - boundaryPositions[neighbors.boundary[k]];
                    float r2 = math.lengthsq(rVec);
                    if (r2 <= 0f) continue;

                    float r = math.sqrt(r2);
                    float2 dir = rVec / r;

                    // Pressure mirroring collapses the coefficient to V_b * p_i.
                    force += dir * (p.boundaryVolume * pressure
                                    * SPHMath.SpikyPow2GradientMagnitude(r, h, p.spikyPow2GradConst));
                }
            }

            particle.force = force;
            particles[i] = particle;
        }
    }

    /// <summary>
    /// Viscosity force: internal friction, dragging each velocity toward its
    /// neighbours'.
    /// </summary>
    /// <remarks>
    /// XSPH, not Muller's Laplacian formulation. Poly6 is used as a weight on
    /// the velocity difference, and because it is non-negative everywhere the
    /// blend can only move a velocity toward its neighbours'. That removes the
    /// hazard Muller's version has, where the Laplacian of a standard kernel can
    /// go negative for close particles and start adding energy.
    ///
    /// This is still the only dissipative term in the solver. Without it the
    /// fluid sloshes forever no matter how correct the pressure force is,
    /// because pressure is conservative.
    ///
    /// The direction lives in (v_j - v_i), so there is no unit vector to get
    /// backwards. Writes the per-particle contribution to viscosityForces as
    /// well as accumulating into force, so it can be drawn for debugging.
    /// </remarks>
    [BurstCompile]
    public static void ComputeViscosityForce(
        NativeArray<Particle2D> particles,
        in NeighborLists neighbors,
        in SolverParams p,
        NativeArray<float2> viscosityForces)
    {
        for (int i = 0; i < particles.Length; i++)
        {
            Particle2D particle = particles[i];
            float2 posI = particle.position;
            float2 velI = particle.velocity;
            float2 viscForce = float2.zero;

            int start = neighbors.fluidStart[i];
            int end = neighbors.fluidStart[i + 1];

            for (int k = start; k < end; k++)
            {
                int n = neighbors.fluid[k];

                Particle2D other = particles[n];

                // XSPH: Poly6 is used as a weight on the velocity difference, not
                // as a Laplacian. Being non-negative everywhere it can only pull
                // this velocity toward its neighbours', so there is no sign to get
                // wrong and no way for the term to add energy. It also needs no
                // square root, since Poly6 is evaluated from squared distance.
                float r2 = math.distancesq(other.position, posI);
                viscForce += (other.velocity - velI)
                             * SPHMath.Poly6Kernel(r2, p.smoothingLengthSq, p.poly6Const);
            }

            if (viscosityForces.IsCreated && i < viscosityForces.Length)
            {
                viscosityForces[i] = viscForce;
            }

            particle.force += viscForce * p.viscosity;
            particles[i] = particle;
        }
    }

    /// <summary>
    /// Semi-implicit (symplectic) Euler, plus the wall clamp.
    /// </summary>
    /// <remarks>
    /// Velocity updates first, then position uses the NEW velocity. That is what
    /// makes the scheme symplectic, and why it damps energy instead of injecting
    /// it.
    ///
    /// The clamp is a penetration safety net only. Wall physics is handled by the
    /// boundary particles in the density and pressure passes; this just
    /// guarantees nothing escapes if a step overshoots.
    /// </remarks>
    [BurstCompile]
    public static void Integrate(NativeArray<Particle2D> particles, in SolverParams p, float dt)
    {
        for (int i = 0; i < particles.Length; i++)
        {
            Particle2D particle = particles[i];

            if (particle.density <= 0.0001f) continue;

            float2 acceleration = particle.force / particle.density + p.gravity;
            particle.velocity += acceleration * dt;
            particle.position += particle.velocity * dt;

            float relX = particle.position.x - p.boxCenter.x;
            float relY = particle.position.y - p.boxCenter.y;

            if (relX > p.boxHalfSize.x)
            {
                particle.position.x = p.boxCenter.x + p.boxHalfSize.x;
                particle.velocity.x *= -p.collisionDamping;
            }
            else if (relX < -p.boxHalfSize.x)
            {
                particle.position.x = p.boxCenter.x - p.boxHalfSize.x;
                particle.velocity.x *= -p.collisionDamping;
            }

            if (relY > p.boxHalfSize.y)
            {
                particle.position.y = p.boxCenter.y + p.boxHalfSize.y;
                particle.velocity.y *= -p.collisionDamping;
            }
            else if (relY < -p.boxHalfSize.y)
            {
                particle.position.y = p.boxCenter.y - p.boxHalfSize.y;
                particle.velocity.y *= -p.collisionDamping;
            }

            particles[i] = particle;
        }
    }

    /// <summary>
    /// Largest step that keeps the explicit integration stable.
    /// </summary>
    /// <remarks>
    /// CFL limit: information must not cross more than one smoothing length per
    /// step, so dt &lt;= C * h / (maxSpeed + soundSpeed). The sound speed comes
    /// from the equation of state: p = k(rho - rho0) gives dp/drho = k, so
    /// c = sqrt(k). A stiffer fluid forces a smaller step -- that is the price of
    /// weak compressibility.
    ///
    /// Acceleration limit: a particle must not be flung across its own
    /// neighbourhood within a single step.
    ///
    /// Takes the max over all particles, so it is a reduction. T-025 will
    /// parallelise it; T-029 has to decide how the GPU supplies these maxima.
    /// </remarks>
    [BurstCompile]
    public static float ComputeStableTimeStep(
        in NativeArray<Particle2D> particles,
        in SolverParams p,
        float cflFactor)
    {
        float soundSpeed = math.sqrt(math.max(0f, p.stiffness));
        float maxSpeed = 0f;
        float maxAccel = 0f;

        for (int i = 0; i < particles.Length; i++)
        {
            Particle2D particle = particles[i];

            float speed = math.length(particle.velocity);
            if (speed > maxSpeed) maxSpeed = speed;

            if (particle.density > 0.0001f)
            {
                float accel = math.length(particle.force / particle.density + p.gravity);
                if (accel > maxAccel) maxAccel = accel;
            }
        }

        float dtCfl = cflFactor * p.smoothingLength / (maxSpeed + soundSpeed + 0.0001f);

        float dtAcc = maxAccel > 0.0001f
            ? cflFactor * math.sqrt(p.smoothingLength / maxAccel)
            : float.MaxValue;

        return math.min(dtCfl, dtAcc);
    }

    /// <summary>
    /// One pass reporting the numbers needed to tell a solver problem from a
    /// setup problem: mean density (is the fluid expanding or compressing?),
    /// peak speed (is anything running away?) and peak acceleration (are the
    /// forces sane relative to gravity?).
    /// </summary>
    [BurstCompile]
    public static void Diagnostics(
        in NativeArray<Particle2D> particles,
        in SolverParams p,
        out float meanDensity,
        out float maxSpeed,
        out float maxAccel)
    {
        float sum = 0f;
        meanDensity = 0f;
        maxSpeed = 0f;
        maxAccel = 0f;

        for (int i = 0; i < particles.Length; i++)
        {
            Particle2D particle = particles[i];
            sum += particle.density;

            float speed = math.length(particle.velocity);
            if (speed > maxSpeed) maxSpeed = speed;

            if (particle.density > 0.0001f)
            {
                float accel = math.length(particle.force / particle.density + p.gravity);
                if (accel > maxAccel) maxAccel = accel;
            }
        }

        if (particles.Length > 0) meanDensity = sum / particles.Length;
    }

    /// <summary>Mean density over particles that have any density at all.</summary>
    /// <remarks>
    /// Used once at spawn to scale particle mass so the mean density lands on
    /// restDensity, which keeps the pressure field from starting far from rest.
    /// </remarks>
    [BurstCompile]
    public static float MeanDensity(in NativeArray<Particle2D> particles)
    {
        float sum = 0f;
        int count = 0;

        for (int i = 0; i < particles.Length; i++)
        {
            float density = particles[i].density;

            if (density > 0f)
            {
                sum += density;
                count++;
            }
        }

        return count > 0 ? sum / count : 0f;
    }
}

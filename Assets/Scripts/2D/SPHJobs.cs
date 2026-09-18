using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

/// <summary>
/// The solver passes as parallel jobs.
///
/// Each pass is a single <see cref="IJobParallelFor"/> over particle indices.
/// That is only safe because of the SoA layout: every pass writes exactly one
/// array and reads others, so no thread reads a slot another thread is writing.
///
/// Scheduling order matters and is expressed with job dependencies in SPH2D:
///
///   ResetForces -> ComputePressureForce -> ComputeViscosity   (all touch force)
///   ComputeDensity -> ComputePressure                         (pressure needs density)
///   ComputePressureForce -> Integrate                         (integration needs force)
///
/// The two force jobs are split rather than fused so each stays comparable to the
/// serial version they replaced, at the cost of one dependency between them.
///
/// Batch count is a schedule-time argument, not a property of the job. The Unity
/// docs suggest starting at 1 for even distribution, then raising it until the
/// gains flatten out.
/// </summary>
[BurstCompile]
public struct ResetForcesJob : IJobParallelFor
{
    [WriteOnly] public NativeArray<float2> force;

    public void Execute(int i)
    {
        force[i] = float2.zero;
    }
}

[BurstCompile]
public struct ComputeDensityJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float2> position;
    [ReadOnly] public NeighborLists neighbors;
    [ReadOnly] public NativeArray<float2> boundaryPositions;

    [WriteOnly] public NativeArray<float> density;
    [WriteOnly] public NativeArray<float> nearDensity;

    public SolverParams p;

    public void Execute(int i)
    {
        float h = p.smoothingLength;
        float2 posI = position[i];
        float mainDensity = 0f;
        float near = 0f;

        int start = neighbors.fluidStart[i];
        int end = neighbors.fluidStart[i + 1];

        for (int k = start; k < end; k++)
        {
            float r = math.sqrt(math.distancesq(position[neighbors.fluid[k]], posI));

            mainDensity += p.particleMass * SPHMath.SpikyPow2Kernel(r, h, p.spikyPow2Const);
            near += p.particleMass * SPHMath.SpikyPow3Kernel(r, h, p.spikyPow3Const);
        }

        // Boundary particles contribute to the main density only. The near channel
        // is about fluid particles crowded against each other; the wall is handled
        // by the main channel plus the position clamp.
        if (p.useBoundaryParticles)
        {
            int bStart = neighbors.boundaryStart[i];
            int bEnd = neighbors.boundaryStart[i + 1];

            for (int k = bStart; k < bEnd; k++)
            {
                float r = math.sqrt(math.distancesq(boundaryPositions[neighbors.boundary[k]], posI));

                mainDensity += p.restDensity * p.boundaryVolume
                               * SPHMath.SpikyPow2Kernel(r, h, p.spikyPow2Const);
            }
        }

        density[i] = mainDensity;
        nearDensity[i] = near;
    }
}

[BurstCompile]
public struct ComputePressureJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float> density;
    [ReadOnly] public NativeArray<float> nearDensity;

    [WriteOnly] public NativeArray<float> pressure;
    [WriteOnly] public NativeArray<float> nearPressure;

    public SolverParams p;

    public void Execute(int i)
    {
        pressure[i] = SPHMath.Pressure(
            density[i], p.restDensity, p.stiffness, p.clampPressurePositive);

        nearPressure[i] = SPHMath.NearPressure(nearDensity[i], p.nearPressureMultiplier);
    }
}

[BurstCompile]
public struct ComputePressureForceJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float2> position;
    [ReadOnly] public NativeArray<float> density;
    [ReadOnly] public NativeArray<float> pressure;
    [ReadOnly] public NativeArray<float> nearDensity;
    [ReadOnly] public NativeArray<float> nearPressure;
    [ReadOnly] public NeighborLists neighbors;
    [ReadOnly] public NativeArray<float2> boundaryPositions;

    [WriteOnly] public NativeArray<float2> force;

    public SolverParams p;

    public void Execute(int i)
    {
        float h = p.smoothingLength;
        float2 posI = position[i];
        float2 accumulated = float2.zero;

        int start = neighbors.fluidStart[i];
        int end = neighbors.fluidStart[i + 1];

        for (int k = start; k < end; k++)
        {
            int n = neighbors.fluid[k];

            // Guard against zero density from isolated particles
            float otherDensity = density[n];
            if (otherDensity <= 0.0001f) continue;

            float2 rVec = posI - position[n];
            float r2 = math.lengthsq(rVec);
            if (r2 <= 0f) continue;

            float r = math.sqrt(r2);
            float2 dir = rVec / r;

            // Main channel: symmetrised Tait pressure over the linear SpikyPow2
            // gradient. This one can be attractive.
            float sharedPressure = (pressure[i] + pressure[n]) * 0.5f;

            accumulated += dir * (p.particleMass
                                  * SPHMath.SpikyPow2GradientMagnitude(r, h, p.spikyPow2GradConst)
                                  * sharedPressure / otherDensity);

            // Near channel: always repulsive, short range only. This is what keeps
            // the fluid from collapsing when the main term goes negative.
            float otherNearDensity = nearDensity[n];
            if (otherNearDensity > 0.0001f)
            {
                float sharedNearPressure = (nearPressure[i] + nearPressure[n]) * 0.5f;

                accumulated += dir * (p.particleMass
                                      * SPHMath.SpikyPow3GradientMagnitude(r, h, p.spikyPow3GradConst)
                                      * sharedNearPressure / otherNearDensity);
            }
        }

        // The wall may push but never pull. Pressure mirroring copies this
        // particle's own pressure, and that value is often negative near a wall
        // where density is below rest. A mirrored negative pressure would make the
        // boundary attractive, so it is clamped.
        if (p.useBoundaryParticles)
        {
            float boundaryPressure = math.max(0f, pressure[i]);

            int bStart = neighbors.boundaryStart[i];
            int bEnd = neighbors.boundaryStart[i + 1];

            for (int k = bStart; k < bEnd; k++)
            {
                float2 rVec = posI - boundaryPositions[neighbors.boundary[k]];
                float r2 = math.lengthsq(rVec);
                if (r2 <= 0f) continue;

                float r = math.sqrt(r2);
                float2 dir = rVec / r;

                accumulated += dir * (p.boundaryVolume * boundaryPressure
                                      * SPHMath.SpikyPow2GradientMagnitude(r, h, p.spikyPow2GradConst));
            }
        }

        force[i] = accumulated;
    }
}

[BurstCompile]
public struct ComputeViscosityJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float2> position;
    [ReadOnly] public NativeArray<float2> velocity;
    [ReadOnly] public NeighborLists neighbors;

    [WriteOnly] public NativeArray<float2> viscosityForce;

    // Read and written: this pass accumulates on top of the pressure force, which
    // is why it must be scheduled as a dependent job rather than in parallel.
    public NativeArray<float2> force;

    public SolverParams p;

    public void Execute(int i)
    {
        float2 posI = position[i];
        float2 velI = velocity[i];
        float2 viscForce = float2.zero;

        int start = neighbors.fluidStart[i];
        int end = neighbors.fluidStart[i + 1];

        for (int k = start; k < end; k++)
        {
            int n = neighbors.fluid[k];

            // XSPH: Poly6 is a weight on the velocity difference, not a Laplacian.
            // Being non-negative everywhere it can only pull this velocity toward
            // its neighbours', so there is no sign to get wrong and no way for the
            // term to add energy. It also needs no square root.
            float r2 = math.distancesq(position[n], posI);
            viscForce += (velocity[n] - velI)
                         * SPHMath.Poly6Kernel(r2, p.smoothingLengthSq, p.poly6Const);
        }

        viscosityForce[i] = viscForce;
        force[i] += viscForce * p.viscosity;
    }
}

[BurstCompile]
public struct IntegrateJob : IJobParallelFor
{
    public NativeArray<float2> position;
    public NativeArray<float2> velocity;

    [ReadOnly] public NativeArray<float2> force;
    [ReadOnly] public NativeArray<float> density;

    public SolverParams p;

    /// <summary>
    /// The interactive disc. A position constraint applied after the wall clamp,
    /// so it wins where the two disagree.
    /// </summary>
    public PointerState pointer;

    public float dt;

    public void Execute(int i)
    {
        float particleDensity = density[i];

        if (particleDensity <= 0.0001f)
        {
            // Nothing to integrate, but the pointer constrains position rather than
            // applying a force, so it still applies. Skipping it would leave a
            // stray particle inside a disc every other particle respects.
            float2 isolatedPosition = position[i];
            float2 isolatedVelocity = velocity[i];

            SPHMath.ResolvePointerCollision(pointer, ref isolatedPosition, ref isolatedVelocity);

            position[i] = isolatedPosition;
            velocity[i] = isolatedVelocity;
            return;
        }

        float2 acceleration = force[i] / particleDensity + p.gravity;
        float2 v = velocity[i] + acceleration * dt;
        float2 pos = position[i] + v * dt;

        // Wall clamp: a penetration safety net only. Wall physics is handled by the
        // boundary particles; this guarantees nothing escapes if a step overshoots.
        float relX = pos.x - p.boxCenter.x;
        float relY = pos.y - p.boxCenter.y;

        if (relX > p.boxHalfSize.x)
        {
            pos.x = p.boxCenter.x + p.boxHalfSize.x;
            v.x *= -p.collisionDamping;
        }
        else if (relX < -p.boxHalfSize.x)
        {
            pos.x = p.boxCenter.x - p.boxHalfSize.x;
            v.x *= -p.collisionDamping;
        }

        if (relY > p.boxHalfSize.y)
        {
            pos.y = p.boxCenter.y + p.boxHalfSize.y;
            v.y *= -p.collisionDamping;
        }
        else if (relY < -p.boxHalfSize.y)
        {
            pos.y = p.boxCenter.y - p.boxHalfSize.y;
            v.y *= -p.collisionDamping;
        }

        // After the wall clamp, so the disc wins if both act on one particle.
        // SPHInteractor2D keeps the disc a full radius clear of the walls, so in
        // practice only one of them ever does.
        SPHMath.ResolvePointerCollision(pointer, ref pos, ref v);

        velocity[i] = v;
        position[i] = pos;
    }
}

/// <summary>
/// Repacks the SoA state into the Particle2D array the renderer uploads.
///
/// The renderer keeps its own buffer format rather than reading the solver's
/// layout directly. That is deliberate: it means changing the solver's memory
/// layout cannot silently break rendering, and it is the same separation the GPU
/// path needs, where display buffers and simulation buffers are distinct.
/// </summary>
[BurstCompile]
public struct RepackRenderBufferJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float2> position;
    [ReadOnly] public NativeArray<float2> velocity;
    [ReadOnly] public NativeArray<float2> force;
    [ReadOnly] public NativeArray<float> density;
    [ReadOnly] public NativeArray<float> pressure;
    [ReadOnly] public NativeArray<float> nearDensity;
    [ReadOnly] public NativeArray<float> nearPressure;

    [WriteOnly] public NativeArray<Particle2D> render;

    public void Execute(int i)
    {
        render[i] = new Particle2D
        {
            position = position[i],
            velocity = velocity[i],
            force = force[i],
            density = density[i],
            pressure = pressure[i],
            nearDensity = nearDensity[i],
            nearPressure = nearPressure[i],
        };
    }
}

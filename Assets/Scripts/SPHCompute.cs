using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

/// <summary>
/// Owner of the GPU-side particle buffers and dispatch.
///
/// T-026 built the plumbing (allocate, upload, dispatch, read back). T-027 adds
/// the first physics: a density kernel and a pressure kernel that mirror the
/// CPU's two jobs one for one, so the two implementations can be compared on
/// identical input. T-028 adds forces, T-029 removes the per-frame readback.
///
/// The GPU layout mirrors <see cref="ParticleState"/> on purpose: one structured
/// buffer holding the same 40-byte Particle2D the renderer already consumes. That
/// is deliberate for two reasons. It means the solver's CPU memory layout is free
/// to change without touching the shader, and it means the buffer the simulation
/// writes is already the buffer the renderer will want at T-032.
///
/// Boundary particles get their own buffer. They are uploaded once at spawn and
/// never written again, so they need no double buffering and no place in the
/// integration state. See docs/TASKS.md, GPU Port Decisions #2.
/// </summary>
public class SPHCompute : MonoBehaviour
{
    /// <summary>Bytes per particle. Must match the Particle struct in SPH2D.compute.</summary>
    public const int ParticleStride = sizeof(float) * 10;

    /// <summary>Must match [numthreads(...)] in SPH2D.compute.</summary>
    public const int ThreadGroupSize = 64;

    private const string ProbeKernelName = "ProbeRoundTrip";
    private const string DensityKernelName = "ComputeDensity";
    private const string PressureKernelName = "ComputePressure";
    private const string ForceKernelName = "ComputePressureForce";
    private const string ViscosityKernelName = "ComputeViscosity";
    private const string IntegrateKernelName = "Integrate";

    /// <summary>
    /// Displacement the probe kernel applies to every position. Passed to the
    /// shader rather than hardcoded on both sides, so there is one source of truth.
    /// </summary>
    public static readonly float2 ProbeOffset = new float2(0.125f, -0.25f);

    [Tooltip("Compute shader asset. Needs the ProbeRoundTrip, ComputeDensity and " +
             "ComputePressure kernels.")]
    public ComputeShader shader;

    [Tooltip("Particle capacity. Buffers are allocated to this by Initialize().")]
    public int particleCapacity = 400;

    [Tooltip("Scalar the solver uses to weight boundary contributions. Placeholder " +
             "value until T-028 feeds the real one (spawnSpacing squared).")]
    public float boundaryVolume = 0.0625f;

    [Tooltip("Run the round-trip self test once in Start(). Useful while the " +
             "pipeline is being built; turn it off once physics is running.")]
    public bool verifyOnStart = true;

    private ComputeBuffer particlesIn;
    private ComputeBuffer particlesOut;
    private ComputeBuffer boundary;

    // Density, pressure and force are kept out of the Particle struct so no pass
    // writes a field another thread is reading. See the note in SPH2D.compute.
    private ComputeBuffer density;
    private ComputeBuffer nearDensity;
    private ComputeBuffer pressure;
    private ComputeBuffer nearPressure;
    private ComputeBuffer force;
    private ComputeBuffer viscosityForce;

    private int probeKernel = -1;
    private int densityKernel = -1;
    private int pressureKernel = -1;
    private int forceKernel = -1;
    private int viscosityKernel = -1;
    private int integrateKernel = -1;

    private int boundaryCapacity;
    private bool ready;

    private SolverParams parameters;
    private bool hasParameters;

    // Reused between readbacks so the one unavoidable managed hop does not
    // allocate every frame. T-029 removes the readback entirely.
    private Particle2D[] readbackScratch;
    private float[] scalarScratch;
    private float2[] vectorScratch;

    public bool IsReady => ready;
    public int ParticleCapacity => particleCapacity;
    public int BoundaryCount { get; private set; }

    private void Start()
    {
        if (!verifyOnStart) return;

        if (RunRoundTripSelfTest(particleCapacity, out string report))
        {
            Debug.Log($"SPHCompute: GPU round trip OK. {report}");
        }
        else
        {
            Debug.LogError($"SPHCompute: GPU round trip FAILED. {report}");
        }
    }

    /// <summary>
    /// Allocates the buffers for <paramref name="capacity"/> particles.
    /// </summary>
    /// <remarks>
    /// Returns false and logs why, rather than leaving a half-built pipeline that
    /// only fails later inside a dispatch. A compute shader asset can load while
    /// still missing a kernel (renamed in HLSL, stripped by a pragma typo), and
    /// that failure should surface at setup, not mid-simulation.
    /// </remarks>
    public bool Initialize(int capacity)
    {
        if (shader == null)
        {
            Debug.LogError("SPHCompute: no compute shader assigned.");
            return false;
        }

        foreach (string name in new[]
                 { ProbeKernelName, DensityKernelName, PressureKernelName,
                   ForceKernelName, ViscosityKernelName, IntegrateKernelName })
        {
            if (!shader.HasKernel(name))
            {
                Debug.LogError($"SPHCompute: compute shader has no '{name}' kernel.");
                return false;
            }
        }

        Release();

        particleCapacity = Mathf.Max(1, capacity);
        probeKernel = shader.FindKernel(ProbeKernelName);
        densityKernel = shader.FindKernel(DensityKernelName);
        pressureKernel = shader.FindKernel(PressureKernelName);
        forceKernel = shader.FindKernel(ForceKernelName);
        viscosityKernel = shader.FindKernel(ViscosityKernelName);
        integrateKernel = shader.FindKernel(IntegrateKernelName);

        particlesIn = new ComputeBuffer(particleCapacity, ParticleStride, ComputeBufferType.Structured);
        particlesOut = new ComputeBuffer(particleCapacity, ParticleStride, ComputeBufferType.Structured);

        density = new ComputeBuffer(particleCapacity, sizeof(float), ComputeBufferType.Structured);
        nearDensity = new ComputeBuffer(particleCapacity, sizeof(float), ComputeBufferType.Structured);
        pressure = new ComputeBuffer(particleCapacity, sizeof(float), ComputeBufferType.Structured);
        nearPressure = new ComputeBuffer(particleCapacity, sizeof(float), ComputeBufferType.Structured);
        force = new ComputeBuffer(particleCapacity, sizeof(float) * 2, ComputeBufferType.Structured);
        viscosityForce = new ComputeBuffer(particleCapacity, sizeof(float) * 2, ComputeBufferType.Structured);

        // Park a one-element boundary buffer immediately. The kernel reads element
        // zero unconditionally, so it must never be null, and a zero-length
        // ComputeBuffer cannot be allocated at all.
        boundary = new ComputeBuffer(1, sizeof(float) * 2, ComputeBufferType.Structured);
        boundary.SetData(new[] { new float2(1e9f, 1e9f) });
        boundaryCapacity = 1;
        BoundaryCount = 0;

        ready = true;
        return true;
    }

    /// <summary>Uploads the fluid particle state. Called once per frame at T-029.</summary>
    public void UploadParticles(NativeArray<Particle2D> particles)
    {
        if (!ready || particles.Length == 0) return;

        int count = Mathf.Min(particles.Length, particleCapacity);
        particlesIn.SetData(particles, 0, 0, count);
    }

    /// <summary>
    /// Uploads the static wall particles. Uploaded once at spawn and never
    /// updated, because they never move.
    /// </summary>
    /// <remarks>
    /// The buffer always holds at least one element. A zero-length ComputeBuffer
    /// is not allocatable, and the shader reads element zero unconditionally, so
    /// the same trick the CPU side uses applies: the "disabled" case is one
    /// parked element, not an absent buffer.
    /// </remarks>
    public void UploadBoundary(NativeArray<float2> positions)
    {
        if (!ready) return;

        BoundaryCount = positions.Length;

        int needed = Mathf.Max(1, positions.Length);

        if (boundary == null || boundaryCapacity < needed)
        {
            boundary?.Release();
            boundary = new ComputeBuffer(needed, sizeof(float) * 2, ComputeBufferType.Structured);
            boundaryCapacity = needed;
        }

        if (positions.Length > 0) boundary.SetData(positions);
    }

    /// <summary>
    /// Caches the solver parameters the kernels read as shader constants.
    /// </summary>
    /// <remarks>
    /// Takes the CPU's own <see cref="SolverParams"/> rather than a parallel set
    /// of fields, so there is one definition of what h, the kernel constants and
    /// the EOS coefficients mean. This is the struct T-026's comment predicted
    /// would become the constant block almost verbatim; the only change is that
    /// the two bools cross the boundary as 0/1.
    /// </remarks>
    public void UploadParameters(in SolverParams p)
    {
        parameters = p;
        hasParameters = true;
    }

    /// <summary>Dispatches the probe kernel over the whole capacity.</summary>
    public void DispatchProbe()
    {
        if (!ready) return;

        shader.SetBuffer(probeKernel, "_ParticlesIn", particlesIn);
        shader.SetBuffer(probeKernel, "_ParticlesOut", particlesOut);
        shader.SetBuffer(probeKernel, "_BoundaryPositions", boundary);

        shader.SetInt("_ParticleCount", particleCapacity);
        shader.SetFloat("_BoundaryVolume", boundaryVolume);
        shader.SetVector("_ProbeOffset", new Vector4(ProbeOffset.x, ProbeOffset.y, 0f, 0f));

        shader.Dispatch(probeKernel, GroupCount(), 1, 1);
    }

    /// <summary>
    /// Runs the density pass, then the pressure pass that consumes it.
    /// </summary>
    /// <remarks>
    /// Two dispatches rather than one fused kernel, matching the CPU's split
    /// ComputeDensityJob -> ComputePressureJob. The split is what makes "does the
    /// GPU match the CPU" a per-stage question instead of a whole-pipeline one:
    /// if density already differs, pressure cannot be trusted even if it looks
    /// plausible.
    /// </remarks>
    public void DispatchDensityPressure()
    {
        if (!ready || !hasParameters) return;

        SetParameterUniforms();

        shader.SetBuffer(densityKernel, "_ParticlesIn", particlesIn);
        shader.SetBuffer(densityKernel, "_BoundaryPositions", boundary);
        shader.SetBuffer(densityKernel, "_Density", density);
        shader.SetBuffer(densityKernel, "_NearDensity", nearDensity);
        shader.Dispatch(densityKernel, GroupCount(), 1, 1);

        shader.SetBuffer(pressureKernel, "_Density", density);
        shader.SetBuffer(pressureKernel, "_NearDensity", nearDensity);
        shader.SetBuffer(pressureKernel, "_Pressure", pressure);
        shader.SetBuffer(pressureKernel, "_NearPressure", nearPressure);
        shader.Dispatch(pressureKernel, GroupCount(), 1, 1);
    }

    /// <summary>
    /// Runs the pressure-force pass, then the viscosity pass that adds to it.
    /// </summary>
    /// <remarks>
    /// The order is load-bearing: ComputeViscosity reads _Force and adds to it.
    /// Unity records dispatches on one queue and puts a UAV barrier between passes
    /// that touch the same buffer, so sequential Dispatch calls are the GPU
    /// equivalent of the CPU's JobHandle dependency chain.
    /// </remarks>
    public void DispatchForces()
    {
        if (!ready || !hasParameters) return;

        SetParameterUniforms();

        shader.SetBuffer(forceKernel, "_ParticlesIn", particlesIn);
        shader.SetBuffer(forceKernel, "_BoundaryPositions", boundary);
        shader.SetBuffer(forceKernel, "_Density", density);
        shader.SetBuffer(forceKernel, "_NearDensity", nearDensity);
        shader.SetBuffer(forceKernel, "_Pressure", pressure);
        shader.SetBuffer(forceKernel, "_NearPressure", nearPressure);
        shader.SetBuffer(forceKernel, "_Force", force);
        shader.Dispatch(forceKernel, GroupCount(), 1, 1);

        shader.SetBuffer(viscosityKernel, "_ParticlesIn", particlesIn);
        shader.SetBuffer(viscosityKernel, "_ViscosityForce", viscosityForce);
        shader.SetBuffer(viscosityKernel, "_Force", force);
        shader.Dispatch(viscosityKernel, GroupCount(), 1, 1);
    }

    /// <summary>
    /// Integrates one step of semi-implicit Euler and writes the new state into
    /// the output particle buffer.
    /// </summary>
    public void DispatchIntegrate(float dt)
    {
        if (!ready || !hasParameters) return;

        SetParameterUniforms();
        shader.SetFloat("_DeltaTime", dt);

        shader.SetBuffer(integrateKernel, "_ParticlesIn", particlesIn);
        shader.SetBuffer(integrateKernel, "_ParticlesOut", particlesOut);
        shader.SetBuffer(integrateKernel, "_Density", density);
        shader.SetBuffer(integrateKernel, "_Force", force);
        shader.Dispatch(integrateKernel, GroupCount(), 1, 1);
    }

    private void SetParameterUniforms()
    {
        shader.SetInt("_ParticleCount", particleCapacity);
        shader.SetInt("_BoundaryCount", BoundaryCount);

        shader.SetFloat("_SmoothingLength", parameters.smoothingLength);
        shader.SetFloat("_SmoothingLengthSq", parameters.smoothingLengthSq);
        shader.SetFloat("_SpikyPow2Const", parameters.spikyPow2Const);
        shader.SetFloat("_SpikyPow3Const", parameters.spikyPow3Const);
        shader.SetFloat("_SpikyPow2GradConst", parameters.spikyPow2GradConst);
        shader.SetFloat("_SpikyPow3GradConst", parameters.spikyPow3GradConst);
        shader.SetFloat("_Poly6Const", parameters.poly6Const);
        shader.SetFloat("_RestDensity", parameters.restDensity);
        shader.SetFloat("_ParticleMass", parameters.particleMass);
        shader.SetFloat("_BoundaryVolume", parameters.boundaryVolume);
        shader.SetFloat("_NearPressureMultiplier", parameters.nearPressureMultiplier);
        shader.SetFloat("_Stiffness", parameters.stiffness);
        shader.SetFloat("_Viscosity", parameters.viscosity);

        // float2 has no implicit conversion to Vector4, so widen explicitly.
        shader.SetVector("_Gravity", new Vector4(parameters.gravity.x, parameters.gravity.y, 0f, 0f));
        shader.SetVector("_BoxCenter", new Vector4(parameters.boxCenter.x, parameters.boxCenter.y, 0f, 0f));
        shader.SetVector("_BoxHalfSize", new Vector4(parameters.boxHalfSize.x, parameters.boxHalfSize.y, 0f, 0f));
        shader.SetFloat("_CollisionDamping", parameters.collisionDamping);

        // Bools cross as 0/1, matching how the CPU treats them. HLSL has no
        // fixed-size bool, which is the same reason SolverParams needs
        // [MarshalAs(UnmanagedType.U1)] for the Burst direct calls.
        shader.SetInt("_ClampPressurePositive", parameters.clampPressurePositive ? 1 : 0);
        shader.SetInt("_UseBoundaryParticles", parameters.useBoundaryParticles ? 1 : 0);
    }

    /// <summary>Copies the kernel's output buffer back into managed-visible memory.</summary>
    /// <remarks>
    /// The destination must hold the whole buffer. ComputeBuffer in this Unity
    /// version exposes GetData only for managed arrays, not for NativeArray, so
    /// the copy goes buffer -> managed scratch -> NativeArray. That is one extra
    /// copy, which is acceptable here because T-029 deletes this path.
    /// </remarks>
    public void Readback(NativeArray<Particle2D> into)
    {
        if (!ready || into.Length == 0) return;

        if (into.Length < particleCapacity)
        {
            Debug.LogError($"SPHCompute.Readback: destination has {into.Length} slots " +
                           $"but the buffer holds {particleCapacity}.");
            return;
        }

        if (readbackScratch == null || readbackScratch.Length != particleCapacity)
        {
            readbackScratch = new Particle2D[particleCapacity];
        }

        particlesOut.GetData(readbackScratch, 0, 0, particleCapacity);
        NativeArray<Particle2D>.Copy(readbackScratch, into, particleCapacity);
    }

    /// <summary>
    /// Reads back the four scalar fields the density and pressure kernels write.
    /// Each destination must hold the whole buffer.
    /// </summary>
    public void ReadbackScalars(
        NativeArray<float> outDensity,
        NativeArray<float> outNearDensity,
        NativeArray<float> outPressure,
        NativeArray<float> outNearPressure)
    {
        if (!ready) return;

        if (scalarScratch == null || scalarScratch.Length != particleCapacity)
        {
            scalarScratch = new float[particleCapacity];
        }

        ReadScalar(density, outDensity);
        ReadScalar(nearDensity, outNearDensity);
        ReadScalar(pressure, outPressure);
        ReadScalar(nearPressure, outNearPressure);
    }

    private void ReadScalar(ComputeBuffer buffer, NativeArray<float> into)
    {
        if (into.Length < particleCapacity)
        {
            Debug.LogError($"SPHCompute.ReadbackScalars: destination has {into.Length} slots " +
                           $"but the buffer holds {particleCapacity}.");
            return;
        }

        buffer.GetData(scalarScratch, 0, 0, particleCapacity);
        NativeArray<float>.Copy(scalarScratch, into, particleCapacity);
    }

    /// <summary>
    /// Reads back the two force vector buffers. Each destination must hold the
    /// whole buffer.
    /// </summary>
    public void ReadbackForce(NativeArray<float2> outForce, NativeArray<float2> outViscosityForce)
    {
        if (!ready) return;

        ReadVector(force, outForce);
        ReadVector(viscosityForce, outViscosityForce);
    }

    private void ReadVector(ComputeBuffer buffer, NativeArray<float2> into)
    {
        if (into.Length < particleCapacity)
        {
            Debug.LogError($"SPHCompute.ReadbackForce: destination has {into.Length} slots " +
                           $"but the buffer holds {particleCapacity}.");
            return;
        }

        if (vectorScratch == null || vectorScratch.Length != particleCapacity)
        {
            vectorScratch = new float2[particleCapacity];
        }

        buffer.GetData(vectorScratch, 0, 0, particleCapacity);
        NativeArray<float2>.Copy(vectorScratch, into, particleCapacity);
    }

    /// <summary>
    /// Pushes a known pattern through the GPU and checks what comes back.
    /// </summary>
    /// <remarks>
    /// Verifies three things at once, because they fail in different ways. The
    /// position check proves the read and write buffers are both bound and that
    /// the whole queue actually executed. The force check proves the static
    /// boundary buffer is bound and readable. The density check proves the scalar
    /// constant path works. The last two would otherwise stay invisible until
    /// density is computed from walls in T-027.
    /// </remarks>
    public bool RunRoundTripSelfTest(int count, out string report)
    {
        count = Mathf.Max(1, count);

        if (!Initialize(count))
        {
            report = "Initialize failed (see the previous error).";
            return false;
        }

        var input = new NativeArray<Particle2D>(count, Allocator.Temp);
        var output = new NativeArray<Particle2D>(count, Allocator.Temp);
        var wall = new NativeArray<float2>(1, Allocator.Temp);

        var wallPosition = new float2(7.5f, -3.25f);
        wall[0] = wallPosition;

        for (int i = 0; i < count; i++)
        {
            input[i] = new Particle2D
            {
                position = new float2(i * 0.5f, -i * 0.25f),
                velocity = new float2(i, -i),
                force = new float2(-1f, -1f),
                density = 1000f + i,
                pressure = i,
                nearDensity = i * 2f,
                nearPressure = i * 3f,
            };
        }

        bool ok = true;
        int firstBad = -1;

        try
        {
            UploadParticles(input);
            UploadBoundary(wall);
            DispatchProbe();
            Readback(output);

            for (int i = 0; i < count; i++)
            {
                float2 expectedPosition = input[i].position + ProbeOffset;
                bool positionOk = math.all(math.abs(output[i].position - expectedPosition) <= 1e-5f);
                bool boundaryOk = math.all(output[i].force == wallPosition);
                bool volumeOk = math.abs(output[i].density - boundaryVolume) <= 1e-6f;

                if (!positionOk || !boundaryOk || !volumeOk)
                {
                    ok = false;
                    firstBad = i;
                    break;
                }
            }

            if (ok)
            {
                report = $"{count} particles round-tripped; boundary binding read back as " +
                         $"({wallPosition.x}, {wallPosition.y}); boundaryVolume read back as {boundaryVolume}.";
            }
            else
            {
                report =
                    $"index {firstBad} mismatch: position={output[firstBad].position} " +
                    $"expected={input[firstBad].position + ProbeOffset}; " +
                    $"force={output[firstBad].force} expected={wallPosition}; " +
                    $"density={output[firstBad].density} expected={boundaryVolume}";
            }
        }
        finally
        {
            input.Dispose();
            output.Dispose();
            wall.Dispose();
        }

        return ok;
    }

    private int GroupCount()
    {
        return (particleCapacity + ThreadGroupSize - 1) / ThreadGroupSize;
    }

    private void OnDestroy() => Release();

    /// <summary>
    /// Releases the GPU buffers. ComputeBuffer is unmanaged, so nothing else will
    /// free it: forgetting this leaks GPU memory for the life of the process, and
    /// entering play mode repeatedly makes that visible.
    /// </summary>
    public void Release()
    {
        particlesIn?.Release();
        particlesOut?.Release();
        boundary?.Release();

        density?.Release();
        nearDensity?.Release();
        pressure?.Release();
        nearPressure?.Release();
        force?.Release();
        viscosityForce?.Release();

        particlesIn = null;
        particlesOut = null;
        boundary = null;

        density = null;
        nearDensity = null;
        pressure = null;
        nearPressure = null;
        force = null;
        viscosityForce = null;

        boundaryCapacity = 0;
        BoundaryCount = 0;

        probeKernel = -1;
        densityKernel = -1;
        pressureKernel = -1;
        forceKernel = -1;
        viscosityKernel = -1;
        integrateKernel = -1;

        readbackScratch = null;
        scalarScratch = null;
        vectorScratch = null;

        hasParameters = false;
        ready = false;
    }
}

using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

/// <summary>
/// Owner of the GPU-side particle buffers and dispatch.
///
/// T-026 is plumbing only: allocate, upload, dispatch one kernel, read back.
/// Physics is deliberately absent — T-027 adds density and pressure, T-028 adds
/// forces and integration, T-029 removes the per-frame readback. Keeping the data
/// flow separate from the physics is what makes each of those a small change.
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

    private const string ProbeKernel = "ProbeRoundTrip";

    /// <summary>
    /// Displacement the probe kernel applies to every position. Passed to the
    /// shader rather than hardcoded on both sides, so there is one source of truth.
    /// </summary>
    public static readonly float2 ProbeOffset = new float2(0.125f, -0.25f);

    [Tooltip("Compute shader asset. Needs the ProbeRoundTrip kernel.")]
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

    // Reused between readbacks so the one unavoidable managed hop does not
    // allocate every frame. T-029 removes the readback entirely.
    private Particle2D[] readbackScratch;

    private int kernel = -1;
    private int boundaryCapacity;
    private bool ready;

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
    /// still missing the kernel (renamed in HLSL, stripped by a pragma typo), and
    /// that failure should surface at setup, not mid-simulation.
    /// </remarks>
    public bool Initialize(int capacity)
    {
        if (shader == null)
        {
            Debug.LogError("SPHCompute: no compute shader assigned.");
            return false;
        }

        if (!shader.HasKernel(ProbeKernel))
        {
            Debug.LogError($"SPHCompute: compute shader has no '{ProbeKernel}' kernel.");
            return false;
        }

        Release();

        particleCapacity = Mathf.Max(1, capacity);
        kernel = shader.FindKernel(ProbeKernel);

        particlesIn = new ComputeBuffer(particleCapacity, ParticleStride, ComputeBufferType.Structured);
        particlesOut = new ComputeBuffer(particleCapacity, ParticleStride, ComputeBufferType.Structured);

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

    /// <summary>Dispatches the probe kernel over the whole capacity.</summary>
    public void DispatchProbe()
    {
        if (!ready) return;

        shader.SetBuffer(kernel, "_ParticlesIn", particlesIn);
        shader.SetBuffer(kernel, "_ParticlesOut", particlesOut);
        shader.SetBuffer(kernel, "_BoundaryPositions", boundary);

        shader.SetInt("_ParticleCount", particleCapacity);
        shader.SetFloat("_BoundaryVolume", boundaryVolume);
        shader.SetVector("_ProbeOffset", new Vector4(ProbeOffset.x, ProbeOffset.y, 0f, 0f));

        int groups = (particleCapacity + ThreadGroupSize - 1) / ThreadGroupSize;
        shader.Dispatch(kernel, groups, 1, 1);
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

        particlesIn = null;
        particlesOut = null;
        boundary = null;
        readbackScratch = null;

        boundaryCapacity = 0;
        BoundaryCount = 0;
        kernel = -1;
        ready = false;
    }
}

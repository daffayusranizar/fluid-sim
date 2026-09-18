using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Owner of the GPU-side particle buffers and dispatch.
///
/// T-026 built the plumbing, T-027 added density and pressure, T-028 added forces
/// and integration. T-029 makes the pipeline able to run on its own: particle
/// state stays resident on the GPU and is ping-ponged between two buffers, so a
/// step never round-trips through the CPU.
///
/// The one exception is deliberate and small. An adaptive CFL step needs
/// max||v|| and max||a||, which is a CPU-side number by definition. Those are
/// reduced on the GPU into a two-uint buffer and read back asynchronously, one
/// frame late. That keeps T-021's adaptive step alive without a pipeline stall,
/// and it is GPU Port Decision #4 in docs/TASKS.md.
///
/// The GPU layout mirrors <see cref="ParticleState"/> on purpose: one structured
/// buffer holding the same 40-byte Particle2D the renderer already consumes.
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
    private const string ClearExtremesKernelName = "ClearExtremes";
    private const string ReduceExtremesKernelName = "ReduceExtremes";
    private const string CellKeysKernelName = "ComputeCellKeys";
    private const string ClearBucketsKernelName = "ClearBuckets";
    private const string CountBucketsKernelName = "CountBuckets";
    private const string PrefixSumKernelName = "PrefixSumBuckets";
    private const string ScatterKernelName = "ScatterParticles";

    /// <summary>How a cell coordinate is turned into a bucket index.</summary>
    public enum CellKeyMode
    {
        /// <summary>
        /// Unique index into a dense grid. Collision-free, and the right choice for
        /// this project: the domain is a bounded box, so a cell can never fall
        /// outside the grid, and there are only ~N/4.5 cells to index.
        /// </summary>
        DirectIndex = 0,

        /// <summary>
        /// Prime-multiplier hash. The portable form for an unbounded domain, where
        /// cell coordinates are arbitrary. Trades collisions for a fixed table.
        /// </summary>
        SpatialHash = 1,
    }

    /// <summary>
    /// Displacement the probe kernel applies to every position. Passed to the
    /// shader rather than hardcoded on both sides, so there is one source of truth.
    /// </summary>
    public static readonly float2 ProbeOffset = new float2(0.125f, -0.25f);

    [Tooltip("Compute shader asset. Needs every kernel in SPH2D.compute.")]
    public ComputeShader shader;

    [Tooltip("Particle capacity. Buffers are allocated to this by Initialize().")]
    public int particleCapacity = 5000;

    [Tooltip("Only used by the round-trip probe. The simulation takes the real " +
             "value from SolverParams, which is spawnSpacing squared (0.0048 at " +
             "5000 particles).")]
    public float boundaryVolume = 0.0048f;

    [Tooltip("Run the round-trip self test once in Start(). Turn this off once " +
             "SPHComputeSimulation is driving the pipeline.")]
    public bool verifyOnStart = true;

    [Tooltip("How a cell coordinate becomes a bucket index. DirectIndex is " +
             "collision-free and smaller for this bounded box; SpatialHash is the " +
             "portable form for an unbounded domain.")]
    public CellKeyMode keyMode = CellKeyMode.DirectIndex;

    [Tooltip("Hash bucket count when keyMode is SpatialHash. 0 derives 2N+1.")]
    public int hashTableSize = 0;

    // Ping-pong state. Two buffers, because Integrate reads the current particle
    // and writes the next one; with a single buffer every thread would be reading
    // a slot another thread is overwriting.
    private readonly ComputeBuffer[] particleBuffers = new ComputeBuffer[2];
    private int current;

    private ComputeBuffer boundary;

    // Density, pressure and force live outside the Particle struct so no pass
    // writes a field another thread reads. See the note in SPH2D.compute.
    private ComputeBuffer density;
    private ComputeBuffer nearDensity;
    private ComputeBuffer pressure;
    private ComputeBuffer nearPressure;
    private ComputeBuffer force;
    private ComputeBuffer viscosityForce;

    private ComputeBuffer extremes;

    private ComputeBuffer cellKeys;
    private ComputeBuffer cellCoords;

    // Count-sort output: bucket b owns sortedIndices[bucketStart[b] .. bucketStart[b+1]].
    private ComputeBuffer bucketStart;
    private ComputeBuffer bucketCursor;
    private ComputeBuffer sortedIndices;

    private int gridTotalCount;
    private int gridBucketCount;

    private int probeKernel = -1;
    private int densityKernel = -1;
    private int pressureKernel = -1;
    private int forceKernel = -1;
    private int viscosityKernel = -1;
    private int integrateKernel = -1;
    private int clearExtremesKernel = -1;
    private int reduceExtremesKernel = -1;
    private int cellKeysKernel = -1;
    private int clearBucketsKernel = -1;
    private int countBucketsKernel = -1;
    private int prefixSumKernel = -1;
    private int scatterKernel = -1;

    private int boundaryCapacity;
    private bool ready;

    private SolverParams parameters;
    private bool hasParameters;

    // The interactive disc the integrate kernel applies. Cached rather than
    // uploaded on set, because it goes out with the rest of the parameter block in
    // SetParameterUniforms -- one place that knows what the shader expects.
    private PointerState pointer;

    private bool extremesPending;
    private AsyncGPUReadbackRequest extremesRequest;

    // Reused between readbacks so the one unavoidable managed hop does not
    // allocate every frame.
    private Particle2D[] readbackScratch;
    private float[] scalarScratch;
    private float2[] vectorScratch;
    private uint[] extremesScratch;
    private uint[] keyScratch;
    private int2[] coordScratch;
    private uint[] bucketScratch;
    private uint[] indexScratch;

    public bool IsReady => ready;
    public int ParticleCapacity => particleCapacity;

    /// <summary>
    /// Smoothing length from the last uploaded parameters, or 0 before any are set.
    /// The renderer uses it to size impostors from the particle spacing, which is
    /// h divided by the solver's smoothingLengthInSpacing.
    /// </summary>
    public float SmoothingLength => hasParameters ? parameters.smoothingLength : 0f;

    /// <summary>
    /// The buffer holding the current particle state. Exposed so the renderer can
    /// bind it directly instead of reading state back to the CPU.
    /// </summary>
    public ComputeBuffer ParticleBuffer => ready ? ParticlesIn : null;
    public int BoundaryCount { get; private set; }

    /// <summary>
    /// max||v|| and max||a|| from the most recent completed asynchronous readback.
    /// These are one frame old by construction, which is the price of not stalling
    /// the pipeline; the CPU already sized dt from the previous step's state.
    /// </summary>
    public float LastMaxSpeed { get; private set; }
    public float LastMaxAccel { get; private set; }

    public bool HasPendingExtremesReadback => extremesPending;

    /// <summary>Grid geometry for the current step. Valid after DispatchCellKeys.</summary>
    public int GridCols { get; private set; }
    public int GridRows { get; private set; }
    public float CellSize { get; private set; }
    public float2 GridOrigin { get; private set; }

    /// <summary>Number of distinct buckets the current key mode can produce.</summary>
    public int BucketCount => keyMode == CellKeyMode.SpatialHash
        ? hashTableSize
        : GridCols * GridRows;

    /// <summary>Boundary particles actually participating in the grid.</summary>
    public int EffectiveBoundaryCount => (hasParameters && parameters.useBoundaryParticles)
        ? BoundaryCount
        : 0;

    /// <summary>Packed particle count: fluid first, then boundary.</summary>
    public int TotalCount => particleCapacity + EffectiveBoundaryCount;

    private ComputeBuffer ParticlesIn => particleBuffers[current];
    private ComputeBuffer ParticlesOut => particleBuffers[1 - current];

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
                 {
                     ProbeKernelName, DensityKernelName, PressureKernelName,
                     ForceKernelName, ViscosityKernelName, IntegrateKernelName,
                     ClearExtremesKernelName, ReduceExtremesKernelName,
                     CellKeysKernelName, ClearBucketsKernelName, CountBucketsKernelName,
                     PrefixSumKernelName, ScatterKernelName,
                 })
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
        clearExtremesKernel = shader.FindKernel(ClearExtremesKernelName);
        reduceExtremesKernel = shader.FindKernel(ReduceExtremesKernelName);
        cellKeysKernel = shader.FindKernel(CellKeysKernelName);
        clearBucketsKernel = shader.FindKernel(ClearBucketsKernelName);
        countBucketsKernel = shader.FindKernel(CountBucketsKernelName);
        prefixSumKernel = shader.FindKernel(PrefixSumKernelName);
        scatterKernel = shader.FindKernel(ScatterKernelName);

        particleBuffers[0] = new ComputeBuffer(particleCapacity, ParticleStride, ComputeBufferType.Structured);
        particleBuffers[1] = new ComputeBuffer(particleCapacity, ParticleStride, ComputeBufferType.Structured);
        current = 0;

        density = new ComputeBuffer(particleCapacity, sizeof(float), ComputeBufferType.Structured);
        nearDensity = new ComputeBuffer(particleCapacity, sizeof(float), ComputeBufferType.Structured);
        pressure = new ComputeBuffer(particleCapacity, sizeof(float), ComputeBufferType.Structured);
        nearPressure = new ComputeBuffer(particleCapacity, sizeof(float), ComputeBufferType.Structured);
        force = new ComputeBuffer(particleCapacity, sizeof(float) * 2, ComputeBufferType.Structured);
        viscosityForce = new ComputeBuffer(particleCapacity, sizeof(float) * 2, ComputeBufferType.Structured);

        extremes = new ComputeBuffer(2, sizeof(uint), ComputeBufferType.Structured);

        if (hashTableSize <= 0) hashTableSize = 2 * particleCapacity + 1;

        // Park a one-element boundary buffer immediately. The kernel reads element
        // zero unconditionally, so it must never be null, and a zero-length
        // ComputeBuffer cannot be allocated at all.
        boundary = new ComputeBuffer(1, sizeof(float) * 2, ComputeBufferType.Structured);
        boundary.SetData(new[] { new float2(1e9f, 1e9f) });
        boundaryCapacity = 1;
        BoundaryCount = 0;

        LastMaxSpeed = 0f;
        LastMaxAccel = 0f;

        ready = true;
        return true;
    }

    /// <summary>Uploads the fluid particle state. Called once at seed time.</summary>
    public void UploadParticles(NativeArray<Particle2D> particles)
    {
        if (!ready || particles.Length == 0) return;

        int count = Mathf.Min(particles.Length, particleCapacity);
        ParticlesIn.SetData(particles, 0, 0, count);
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
    /// the EOS coefficients mean.
    /// </remarks>
    public void UploadParameters(in SolverParams p)
    {
        parameters = p;
        hasParameters = true;
    }

    /// <summary>
    /// Publishes the interactive pointer, applied by the integrate kernel.
    /// </summary>
    /// <remarks>
    /// Called once per frame by <see cref="SPHComputeSimulation"/>, which is where
    /// <see cref="SPHInteractor2D"/> sends it. Cached here rather than uploaded on
    /// set because it is uploaded with the rest of the parameter block once per
    /// substep -- so a pointer costs no buffer, no dispatch and no extra bandwidth.
    /// </remarks>
    public void SetPointer(in PointerState value) => pointer = value;

    // ------------------------------------------------------------------
    // Full step (T-029)
    // ------------------------------------------------------------------

    /// <summary>
    /// Runs one complete simulation step and leaves the result in the other
    /// particle buffer.
    /// </summary>
    /// <remarks>
    /// Order matters at three points:
    ///
    ///   density -> pressure -> force -> viscosity -> reduce -> integrate
    ///
    /// - pressure needs density; viscosity reads _Force and adds to it.
    /// - the reduction runs after forces and before integration, so max||v|| and
    ///   max||a|| describe the same pre-integration state the CPU measures.
    /// - integration writes the other particle buffer, so the swap at the end is
    ///   what makes the new state the current one.
    ///
    /// Nothing here reads back to the CPU.
    /// </remarks>
    public void StepOnce(float dt)
    {
        if (!ready || !hasParameters) return;

        BuildGrid();
        DispatchDensityPressure();
        DispatchForces();
        DispatchReduceExtremes();
        DispatchIntegrate(dt);

        SwapParticles();
    }

    /// <summary>Makes the most recently written particle buffer the current one.</summary>
    public void SwapParticles()
    {
        current = 1 - current;
    }

    // ------------------------------------------------------------------
    // Dispatch
    // ------------------------------------------------------------------

    /// <summary>Dispatches the probe kernel over the whole capacity.</summary>
    public void DispatchProbe()
    {
        if (!ready) return;

        shader.SetBuffer(probeKernel, "_ParticlesIn", ParticlesIn);
        shader.SetBuffer(probeKernel, "_ParticlesOut", ParticlesOut);
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

        if (bucketStart == null)
        {
            Debug.LogError("SPHCompute: BuildGrid() must run before the query passes.");
            return;
        }

        SetParameterUniforms();

        shader.SetBuffer(densityKernel, "_ParticlesIn", ParticlesIn);
        shader.SetBuffer(densityKernel, "_BoundaryPositions", boundary);
        shader.SetBuffer(densityKernel, "_BucketStart", bucketStart);
        shader.SetBuffer(densityKernel, "_SortedIndices", sortedIndices);
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

        if (bucketStart == null)
        {
            Debug.LogError("SPHCompute: BuildGrid() must run before the query passes.");
            return;
        }

        SetParameterUniforms();

        shader.SetBuffer(forceKernel, "_ParticlesIn", ParticlesIn);
        shader.SetBuffer(forceKernel, "_BoundaryPositions", boundary);
        shader.SetBuffer(forceKernel, "_BucketStart", bucketStart);
        shader.SetBuffer(forceKernel, "_SortedIndices", sortedIndices);
        shader.SetBuffer(forceKernel, "_Density", density);
        shader.SetBuffer(forceKernel, "_NearDensity", nearDensity);
        shader.SetBuffer(forceKernel, "_Pressure", pressure);
        shader.SetBuffer(forceKernel, "_NearPressure", nearPressure);
        shader.SetBuffer(forceKernel, "_Force", force);
        shader.Dispatch(forceKernel, GroupCount(), 1, 1);

        shader.SetBuffer(viscosityKernel, "_ParticlesIn", ParticlesIn);
        shader.SetBuffer(viscosityKernel, "_BucketStart", bucketStart);
        shader.SetBuffer(viscosityKernel, "_SortedIndices", sortedIndices);
        shader.SetBuffer(viscosityKernel, "_ViscosityForce", viscosityForce);
        shader.SetBuffer(viscosityKernel, "_Force", force);
        shader.Dispatch(viscosityKernel, GroupCount(), 1, 1);
    }

    /// <summary>Reduces max speed and max acceleration into the two-slot buffer.</summary>
    public void DispatchReduceExtremes()
    {
        if (!ready || !hasParameters) return;

        shader.Dispatch(clearExtremesKernel, 1, 1, 1);

        shader.SetBuffer(reduceExtremesKernel, "_ParticlesIn", ParticlesIn);
        shader.SetBuffer(reduceExtremesKernel, "_Density", density);
        shader.SetBuffer(reduceExtremesKernel, "_Force", force);
        shader.SetBuffer(reduceExtremesKernel, "_Extremes", extremes);

        shader.SetInt("_ParticleCount", particleCapacity);
        shader.SetVector("_Gravity", new Vector4(parameters.gravity.x, parameters.gravity.y, 0f, 0f));

        shader.Dispatch(reduceExtremesKernel, GroupCount(), 1, 1);
    }

    /// <summary>
    /// Maps every particle to a cell coordinate and then to a bucket key. This is
    /// the input the T-031 count sort consumes.
    /// </summary>
    /// <summary>
    /// Maps every particle to a cell coordinate and a bucket key. Exposed
    /// separately from <see cref="BuildGrid"/> so the mapping can be verified on
    /// its own; the full build also clears, counts, scans and scatters.
    /// </summary>
    public void DispatchCellKeys()
    {
        if (!ready || !hasParameters) return;

        ComputeGridGeometry();
        EnsureGridBuffers();
        SetGridUniforms();

        shader.SetBuffer(cellKeysKernel, "_ParticlesIn", ParticlesIn);
        shader.SetBuffer(cellKeysKernel, "_BoundaryPositions", boundary);
        shader.SetBuffer(cellKeysKernel, "_CellKeys", cellKeys);
        shader.SetBuffer(cellKeysKernel, "_CellCoords", cellCoords);
        shader.Dispatch(cellKeysKernel, GroupCountFor(TotalCount), 1, 1);
    }

    /// <summary>
    /// Derives the grid geometry from the solver parameters.
    /// </summary>
    /// <remarks>
    /// The domain is the container plus one smoothing length of margin on every
    /// side, mirroring SPH2D.RebuildNeighbors, so that boundary particles sit
    /// inside the grid rather than being clamped into its edge cells. Cell size is
    /// exactly h, for the same reason as the CPU grid: the 3x3 query is only
    /// guaranteed complete when cell size is >= h.
    /// </remarks>
    /// <summary>
    /// Rebuilds the spatial structure for the current particle positions.
    /// </summary>
    /// <remarks>
    /// Five passes, the classic count sort:
    ///
    ///   keys -> clear -> histogram -> prefix sum -> scatter
    ///
    /// Rebuilt every step because fluid particles move. Boundary particles are
    /// static and are re-sorted along with them; at these sizes that is cheaper
    /// than the bookkeeping needed to keep them separate.
    /// </remarks>
    public void BuildGrid()
    {
        if (!ready || !hasParameters) return;

        ComputeGridGeometry();
        EnsureGridBuffers();
        SetGridUniforms();

        int particleGroups = GroupCountFor(TotalCount);

        // 1. keys: cell coordinate and bucket per packed particle
        shader.SetBuffer(cellKeysKernel, "_ParticlesIn", ParticlesIn);
        shader.SetBuffer(cellKeysKernel, "_BoundaryPositions", boundary);
        shader.SetBuffer(cellKeysKernel, "_CellKeys", cellKeys);
        shader.SetBuffer(cellKeysKernel, "_CellCoords", cellCoords);
        shader.Dispatch(cellKeysKernel, particleGroups, 1, 1);

        // 2. clear
        shader.SetBuffer(clearBucketsKernel, "_BucketStart", bucketStart);
        shader.Dispatch(clearBucketsKernel, GroupCountFor(gridBucketCount), 1, 1);

        // 3. histogram
        shader.SetBuffer(countBucketsKernel, "_CellKeys", cellKeys);
        shader.SetBuffer(countBucketsKernel, "_BucketStart", bucketStart);
        shader.Dispatch(countBucketsKernel, particleGroups, 1, 1);

        // 4. exclusive prefix sum, one workgroup tiled internally
        shader.SetBuffer(prefixSumKernel, "_BucketStart", bucketStart);
        shader.SetBuffer(prefixSumKernel, "_BucketCursor", bucketCursor);
        shader.Dispatch(prefixSumKernel, 1, 1, 1);

        // 5. scatter
        shader.SetBuffer(scatterKernel, "_CellKeys", cellKeys);
        shader.SetBuffer(scatterKernel, "_BucketStart", bucketStart);
        shader.SetBuffer(scatterKernel, "_BucketCursor", bucketCursor);
        shader.SetBuffer(scatterKernel, "_SortedIndices", sortedIndices);
        shader.Dispatch(scatterKernel, particleGroups, 1, 1);
    }

    private void EnsureGridBuffers()
    {
        int total = TotalCount;
        int buckets = BucketCount;

        if (cellKeys != null && gridTotalCount >= total && gridBucketCount == buckets) return;

        cellKeys?.Release();
        cellCoords?.Release();
        sortedIndices?.Release();
        bucketStart?.Release();
        bucketCursor?.Release();

        cellKeys = new ComputeBuffer(total, sizeof(uint), ComputeBufferType.Structured);
        cellCoords = new ComputeBuffer(total, sizeof(int) * 2, ComputeBufferType.Structured);
        sortedIndices = new ComputeBuffer(total, sizeof(uint), ComputeBufferType.Structured);

        // One extra slot holds the running total, so the last bucket's end needs
        // no special case in the query.
        bucketStart = new ComputeBuffer(buckets + 1, sizeof(uint), ComputeBufferType.Structured);
        bucketCursor = new ComputeBuffer(buckets, sizeof(uint), ComputeBufferType.Structured);

        gridTotalCount = total;
        gridBucketCount = buckets;
    }

    private void SetGridUniforms()
    {
        shader.SetInt("_ParticleCount", particleCapacity);
        shader.SetInt("_BoundaryCount", BoundaryCount);
        shader.SetInt("_TotalCount", TotalCount);
        shader.SetInt("_BucketCount", gridBucketCount);

        shader.SetFloat("_CellSize", CellSize);
        shader.SetVector("_GridOrigin", new Vector4(GridOrigin.x, GridOrigin.y, 0f, 0f));
        shader.SetInt("_GridCols", GridCols);
        shader.SetInt("_GridRows", GridRows);
        shader.SetInt("_HashTableSize", hashTableSize);
        shader.SetInt("_KeyMode", (int)keyMode);
    }

    private int GroupCountFor(int elementCount)
    {
        return Mathf.Max(1, (elementCount + ThreadGroupSize - 1) / ThreadGroupSize);
    }

    private void ComputeGridGeometry()
    {
        CellSize = math.max(0.0001f, parameters.smoothingLength);

        float margin = parameters.smoothingLength * 1.01f;
        float2 half = parameters.boxHalfSize + new float2(margin, margin);
        float2 size = half * 2f;

        GridOrigin = parameters.boxCenter - half;
        GridCols = math.max(1, (int)math.ceil(size.x / CellSize));
        GridRows = math.max(1, (int)math.ceil(size.y / CellSize));
    }

    /// <summary>
    /// Integrates one step of semi-implicit Euler and writes the new state into
    /// the other particle buffer.
    /// </summary>
    public void DispatchIntegrate(float dt)
    {
        if (!ready || !hasParameters) return;

        SetParameterUniforms();
        shader.SetFloat("_DeltaTime", dt);

        shader.SetBuffer(integrateKernel, "_ParticlesIn", ParticlesIn);
        shader.SetBuffer(integrateKernel, "_ParticlesOut", ParticlesOut);
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

        shader.SetVector("_PointerPosition", new Vector4(pointer.position.x, pointer.position.y, 0f, 0f));
        shader.SetVector("_PointerVelocity", new Vector4(pointer.velocity.x, pointer.velocity.y, 0f, 0f));
        shader.SetFloat("_PointerRadius", pointer.radius);
        shader.SetFloat("_PointerRestitution", pointer.restitution);
        shader.SetFloat("_PointerFriction", pointer.friction);
        shader.SetInt("_PointerActive", pointer.active ? 1 : 0);
    }

    // ------------------------------------------------------------------
    // Readback
    // ------------------------------------------------------------------

    /// <summary>
    /// Requests the extremes reduction asynchronously. Called once per frame, not
    /// once per step: a second request while one is in flight is skipped rather
    /// than queued, so the pipeline never builds a backlog of readbacks.
    /// </summary>
    /// <remarks>
    /// Poll-based rather than callback-based. A callback only fires while the
    /// player loop is pumping, which makes it untestable outside play mode; the
    /// poll can be driven explicitly, so the readback itself is covered by tests.
    /// </remarks>
    public void RequestExtremesReadback()
    {
        if (!ready || extremesPending) return;

        extremesPending = true;
        extremesRequest = AsyncGPUReadback.Request(extremes);
    }

    private void Update() => PollExtremesReadback();

    /// <summary>Consumes the pending readback once the GPU has finished with it.</summary>
    public void PollExtremesReadback()
    {
        if (!extremesPending || !extremesRequest.done) return;

        ConsumeExtremes();
    }

    /// <summary>
    /// Blocks until the pending readback completes, then consumes it. Test and
    /// diagnostic use only: the per-frame path is <see cref="PollExtremesReadback"/>,
    /// which never stalls.
    /// </summary>
    public bool TryConsumeExtremesBlocking(out float maxSpeed, out float maxAccel)
    {
        maxSpeed = 0f;
        maxAccel = 0f;

        if (!extremesPending) return false;

        extremesRequest.WaitForCompletion();
        ConsumeExtremes();

        maxSpeed = LastMaxSpeed;
        maxAccel = LastMaxAccel;
        return true;
    }

    private void ConsumeExtremes()
    {
        extremesPending = false;

        // On failure keep the previous estimate rather than falling back to zero,
        // which would silently produce a much larger dt.
        if (extremesRequest.hasError) return;

        NativeArray<uint> data = extremesRequest.GetData<uint>();
        if (data.Length < 2) return;

        LastMaxSpeed = math.asfloat(data[0]);
        LastMaxAccel = math.asfloat(data[1]);
    }

    /// <summary>
    /// Reads the extremes buffer synchronously. Test and diagnostic use only: the
    /// per-frame path is <see cref="RequestExtremesReadback"/>, which does not stall.
    /// </summary>
    public void ReadbackExtremes(out float maxSpeed, out float maxAccel)
    {
        maxSpeed = 0f;
        maxAccel = 0f;

        if (!ready) return;

        if (extremesScratch == null) extremesScratch = new uint[2];
        extremes.GetData(extremesScratch, 0, 0, 2);

        maxSpeed = math.asfloat(extremesScratch[0]);
        maxAccel = math.asfloat(extremesScratch[1]);
    }

    /// <summary>Copies the current particle state back into managed-visible memory.</summary>
    /// <remarks>
    /// The destination must hold the whole buffer. ComputeBuffer in this Unity
    /// version exposes GetData only for managed arrays, not for NativeArray, so
    /// the copy goes buffer -> managed scratch -> NativeArray. T-032 replaces this
    /// with rendering straight from the buffer.
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

        ParticlesIn.GetData(readbackScratch, 0, 0, particleCapacity);
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
    /// Reads back the per-particle cell keys and cell coordinates. Both
    /// destinations must hold the whole buffer.
    /// </summary>
    public void ReadbackCellKeys(NativeArray<uint> outKeys, NativeArray<int2> outCoords)
    {
        if (!ready) return;

        if (outKeys.Length < particleCapacity || outCoords.Length < particleCapacity)
        {
            Debug.LogError($"SPHCompute.ReadbackCellKeys: destinations hold " +
                           $"{outKeys.Length}/{outCoords.Length} slots but the buffer holds {particleCapacity}.");
            return;
        }

        if (keyScratch == null || keyScratch.Length != particleCapacity) keyScratch = new uint[particleCapacity];
        if (coordScratch == null || coordScratch.Length != particleCapacity) coordScratch = new int2[particleCapacity];

        cellKeys.GetData(keyScratch, 0, 0, particleCapacity);
        cellCoords.GetData(coordScratch, 0, 0, particleCapacity);

        NativeArray<uint>.Copy(keyScratch, outKeys, particleCapacity);
        NativeArray<int2>.Copy(coordScratch, outCoords, particleCapacity);
    }

    // ------------------------------------------------------------------
    /// <summary>
    /// Reads back the bucket offsets: <c>BucketCount + 1</c> entries, where entry
    /// <c>b</c> is the first slot of bucket <c>b</c>'s run and the final entry is
    /// the total. Test and diagnostic use only.
    /// </summary>
    public void ReadbackBucketStart(NativeArray<uint> outStarts)
    {
        if (!ready || bucketStart == null) return;

        int count = gridBucketCount + 1;

        if (outStarts.Length < count)
        {
            Debug.LogError($"SPHCompute.ReadbackBucketStart: destination has {outStarts.Length} " +
                           $"slots but the buffer holds {count}.");
            return;
        }

        if (bucketScratch == null || bucketScratch.Length < count) bucketScratch = new uint[count];

        bucketStart.GetData(bucketScratch, 0, 0, count);
        NativeArray<uint>.Copy(bucketScratch, outStarts, count);
    }

    /// <summary>Reads back the sorted packed particle indices. Test and diagnostic use only.</summary>
    public void ReadbackSortedIndices(NativeArray<uint> outIndices)
    {
        if (!ready || sortedIndices == null) return;

        int count = gridTotalCount;

        if (outIndices.Length < count)
        {
            Debug.LogError($"SPHCompute.ReadbackSortedIndices: destination has {outIndices.Length} " +
                           $"slots but the buffer holds {count}.");
            return;
        }

        if (indexScratch == null || indexScratch.Length < count) indexScratch = new uint[count];

        sortedIndices.GetData(indexScratch, 0, 0, count);
        NativeArray<uint>.Copy(indexScratch, outIndices, count);
    }

    /// <summary>
    /// Reads back cell keys and coordinates for every packed particle, fluid and
    /// boundary. Test and diagnostic use only.
    /// </summary>
    public void ReadbackCellKeysFull(NativeArray<uint> outKeys, NativeArray<int2> outCoords)
    {
        if (!ready || cellKeys == null) return;

        int count = gridTotalCount;

        if (outKeys.Length < count || outCoords.Length < count)
        {
            Debug.LogError($"SPHCompute.ReadbackCellKeysFull: destinations hold " +
                           $"{outKeys.Length}/{outCoords.Length} slots but the buffer holds {count}.");
            return;
        }

        if (keyScratch == null || keyScratch.Length < count) keyScratch = new uint[count];
        if (coordScratch == null || coordScratch.Length < count) coordScratch = new int2[count];

        cellKeys.GetData(keyScratch, 0, 0, count);
        cellCoords.GetData(coordScratch, 0, 0, count);

        NativeArray<uint>.Copy(keyScratch, outKeys, count);
        NativeArray<int2>.Copy(coordScratch, outCoords, count);
    }

    // Self test (T-026)
    // ------------------------------------------------------------------

    /// <summary>
    /// Pushes a known pattern through the GPU and checks what comes back.
    /// </summary>
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

            // The probe writes the other buffer, so make it current before reading.
            SwapParticles();
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
        for (int i = 0; i < particleBuffers.Length; i++)
        {
            particleBuffers[i]?.Release();
            particleBuffers[i] = null;
        }

        boundary?.Release();
        density?.Release();
        nearDensity?.Release();
        pressure?.Release();
        nearPressure?.Release();
        force?.Release();
        viscosityForce?.Release();
        extremes?.Release();
        cellKeys?.Release();
        cellCoords?.Release();
        sortedIndices?.Release();
        bucketStart?.Release();
        bucketCursor?.Release();

        boundary = null;
        density = null;
        nearDensity = null;
        pressure = null;
        nearPressure = null;
        force = null;
        viscosityForce = null;
        extremes = null;
        cellKeys = null;
        cellCoords = null;
        sortedIndices = null;
        bucketStart = null;
        bucketCursor = null;
        gridTotalCount = 0;
        gridBucketCount = 0;

        boundaryCapacity = 0;
        BoundaryCount = 0;
        current = 0;

        probeKernel = -1;
        densityKernel = -1;
        pressureKernel = -1;
        forceKernel = -1;
        viscosityKernel = -1;
        integrateKernel = -1;
        clearExtremesKernel = -1;
        reduceExtremesKernel = -1;
        cellKeysKernel = -1;
        clearBucketsKernel = -1;
        countBucketsKernel = -1;
        prefixSumKernel = -1;
        scatterKernel = -1;

        readbackScratch = null;
        scalarScratch = null;
        vectorScratch = null;
        extremesScratch = null;
        keyScratch = null;
        coordScratch = null;
        bucketScratch = null;
        indexScratch = null;

        extremesPending = false;
        hasParameters = false;
        ready = false;
    }
}

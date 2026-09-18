using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

/// <summary>
/// Drives the GPU pipeline for a whole session: seeds it once, then runs the
/// frame loop with the particle state never leaving the GPU.
///
/// This is the T-029 integration point. <see cref="SPHCompute"/> owns the buffers
/// and knows how to run one step; this component owns *time* — the frame
/// accumulator, substepping, and the adaptive step size.
///
/// The adaptive step is T-021's, kept deliberately. Its inputs are max||v|| and
/// max||a||, which the GPU reduces into two uints and returns asynchronously one
/// frame late. That is GPU Port Decision #4 in docs/TASKS.md: the alternative was
/// a fixed conservative dt, which would have quietly deleted the CFL behaviour
/// that T-021 added and that the Phase 1 invariants list as load-bearing.
///
/// The CPU still seeds the simulation (spawn and parameter derivation stay in
/// <see cref="SPH2D"/>) and can still read state back for drawing. Neither is
/// part of the step.
/// </summary>
[DefaultExecutionOrder(100)]
public class SPHComputeSimulation : MonoBehaviour
{
    [Tooltip("The CPU solver, used only to spawn particles and supply parameters. " +
             "Its own solver loop is switched off while this component is active.")]
    public SPH2D source;

    [Tooltip("The GPU pipeline. Defaults to a component on the same GameObject.")]
    public SPHCompute compute;

    [Header("Time stepping")]
    // Must match SPH2D's cap: 5000 particles need ~33 substeps per frame to keep
    // up with real time, so 32 would clamp and lose fluid time.
    public int maxSubSteps = 64;
    [Range(0.05f, 0.5f)] public float cflFactor = 0.25f;
    public float minTimeStep = 0.0002f;
    public float maxTimeStep = 0.02f;

    [Header("Visualisation")]
    [Tooltip("Draw the GPU particles as gizmos. Requires a periodic synchronous " +
             "readback, which is why it is off by default and why T-032 renders " +
             "straight from the buffer instead.")]
    public bool showGizmos = false;

    [Tooltip("Read back once every N frames when showGizmos is on.")]
    public int gizmoReadbackInterval = 15;

    [Tooltip("Gizmo sphere radius, world units.")]
    public float gizmoRadius = 0.08f;

    private float accumulator;
    private int frameCounter;
    private Particle2D[] snapshot;
    private int activeCount;

    private void Start()
    {
        if (source == null) source = GetComponent<SPH2D>();
        if (compute == null) compute = GetComponent<SPHCompute>();

        if (source == null || compute == null)
        {
            Debug.LogError("SPHComputeSimulation: needs both an SPH2D and an SPHCompute.");
            enabled = false;
            return;
        }

        // One owner for the particle state. SPH2D.Awake has already spawned and
        // built the parameters by the time Start runs.
        source.solveOnCpu = false;

        activeCount = source.particleCount;

        if (!compute.Initialize(activeCount))
        {
            enabled = false;
            return;
        }

        compute.UploadParticles(source.Particles);
        compute.UploadBoundary(source.BoundaryParticles);
        compute.UploadParameters(source.Parameters);
    }

    private void Update()
    {
        if (!compute.IsReady) return;

        // Frame time goes into a buffer rather than straight into the integrator,
        // exactly as the CPU solver does: a slow frame produces more steps instead
        // of one large unstable step.
        accumulator += Time.deltaTime;

        int steps = 0;

        // LastMaxSpeed / LastMaxAccel are from the previous frame's readback. That
        // one-frame lag is the whole price of not stalling the pipeline, and it is
        // harmless because dt was already sized from the previous step's state
        // before T-029.
        while (accumulator > 0f && steps < maxSubSteps)
        {
            float dt = Mathf.Clamp(
                StableTimeStep(compute.LastMaxSpeed, compute.LastMaxAccel),
                minTimeStep, maxTimeStep);

            dt = Mathf.Min(dt, accumulator);

            compute.StepOnce(dt);

            accumulator -= dt;
            steps++;
        }

        // Drop any backlog so a long hitch cannot start a spiral of death.
        accumulator = 0f;

        // One request per frame, not per step. A second request while one is in
        // flight is skipped inside SPHCompute, so readbacks cannot pile up.
        compute.RequestExtremesReadback();

        if (showGizmos && ++frameCounter >= Mathf.Max(1, gizmoReadbackInterval))
        {
            frameCounter = 0;
            SnapshotForGizmos();
        }
    }

    /// <summary>
    /// The same CFL limit <c>SPHSolver.ComputeStableTimeStep</c> applies, with its
    /// two inputs supplied by the GPU reduction instead of a CPU loop.
    /// </summary>
    private float StableTimeStep(float maxSpeed, float maxAccel)
    {
        SolverParams p = source.Parameters;
        float soundSpeed = Mathf.Sqrt(Mathf.Max(0f, p.stiffness));

        float dtCfl = cflFactor * p.smoothingLength / (maxSpeed + soundSpeed + 0.0001f);

        float dtAcc = maxAccel > 0.0001f
            ? cflFactor * Mathf.Sqrt(p.smoothingLength / maxAccel)
            : float.MaxValue;

        return Mathf.Min(dtCfl, dtAcc);
    }

    /// <summary>
    /// Copies GPU state into a managed array for the gizmo pass. This is the
    /// "read back for visualisation" allowance in T-029, and it is the only
    /// synchronous readback left in the loop.
    /// </summary>
    private void SnapshotForGizmos()
    {
        if (snapshot == null || snapshot.Length != activeCount)
        {
            snapshot = new Particle2D[activeCount];
        }

        // NativeArray(T[], Allocator) copies INTO native memory, so the readback
        // lands in the native buffer and has to be copied back out explicitly.
        var native = new NativeArray<Particle2D>(activeCount, Allocator.Temp);
        compute.Readback(native);
        native.CopyTo(snapshot);
        native.Dispose();
    }

    private void OnDrawGizmos()
    {
        if (!showGizmos || snapshot == null) return;

        Gizmos.color = Color.blue;
        Gizmos.DrawWireCube(transform.position, new Vector3(source.boxSize.x, source.boxSize.y, 0f));

        for (int i = 0; i < snapshot.Length; i++)
        {
            float speed = math.length(snapshot[i].velocity);
            Gizmos.color = Color.Lerp(new Color(0.1f, 0.3f, 0.7f), new Color(1f, 0.9f, 0.6f),
                                      Mathf.InverseLerp(0f, 8f, speed));

            float2 p = snapshot[i].position;
            Gizmos.DrawSphere(new Vector3(p.x, p.y, 0f), gizmoRadius);
        }
    }

    private void OnDestroy()
    {
        // Hand the state back to the CPU solver if the component is removed.
        if (source != null) source.solveOnCpu = true;
    }
}

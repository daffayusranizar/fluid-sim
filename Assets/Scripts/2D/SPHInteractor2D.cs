using Unity.Mathematics;
using UnityEngine;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// Turns the mouse into the solid disc the fluid feels.
///
/// Press and hold to make the disc solid, drag to scoop water with it, tap to poke
/// it, and press space to freeze the simulation. Releasing the button leaves the
/// disc tracking the cursor but intangible, so moving the mouse across the window
/// on the way to something else does not disturb the fluid.
///
/// This component owns input and pointer kinematics and nothing else. It never
/// touches particle data; it publishes a <see cref="PointerState"/> to whichever
/// solver is stepping and lets that solver decide what to do with it. That keeps
/// the input layer replaceable -- a gamepad, a touch, or a recorded track could
/// drive the same disc without the solver changing.
/// </summary>
/// <remarks>
/// Runs at <c>-100</c> so the pointer is published before both
/// <see cref="SPH2D"/> (0) and <see cref="SPHComputeSimulation"/> (100) consume it
/// in the same frame. Publish after the step and the fluid reacts a frame late.
/// </remarks>
[DefaultExecutionOrder(-100)]
public class SPHInteractor2D : MonoBehaviour
{
    [Header("Targets")]
    [Tooltip("The CPU solver. Set to null when nothing steps the Burst path.")]
    public SPH2D solver;

    [Tooltip("The GPU frame loop, which forwards the pointer to SPHCompute.")]
    public SPHComputeSimulation gpuSimulation;

    [Tooltip("Camera used to turn a screen position into a world position. Falls " +
             "back to Camera.main, which requires the camera to carry the " +
             "MainCamera tag.")]
    public Camera targetCamera;

    [Header("Pointer")]
    public bool enableInteraction = true;

    [Tooltip("Size the disc from the particle spacing rather than a fixed world " +
             "radius. Without this, one radius means 'a wide scoop' at 400 " +
             "particles and 'the whole container' at 50000, because the spacing " +
             "changes with the count.")]
    public bool radiusInSpacing = true;

    [Tooltip("Disc radius measured in particle spacings. At 5000 particles in the " +
             "default container one spacing is 0.069, so 9 gives a disc about 0.62 " +
             "world units across the radius.")]
    public float radiusInSpacings = 9f;

    [Tooltip("Disc radius in world units. Used when radiusInSpacing is off.")]
    public float radius = 0.6f;

    [Tooltip("Fastest the disc is allowed to move, in world units per second. This " +
             "is the most important value here: the disc is servoed toward the " +
             "cursor instead of snapping to it, so a flick across the window cannot " +
             "displace a particle by more than a real step's worth. Remove the cap " +
             "and a fast drag teleports the disc, which ejects the fluid inside it " +
             "all at once and blows the pressure solve up.")]
    public float maxPointerSpeed = 12f;

    [Range(0f, 1f)]
    [Tooltip("How much of the impact velocity is returned. 0 is a dead stop, 1 is " +
             "a perfect bounce. Bounces are the second way to inject energy, so " +
             "this stays low.")]
    public float restitution = 0.05f;

    [Range(0f, 1f)]
    [Tooltip("Tangential drag toward the disc's own velocity. This is the " +
             "difference between poking the water and scooping it: without any " +
             "friction a drag slides through and the fluid only moves radially out " +
             "of the way.")]
    public float friction = 0.35f;

    [Tooltip("How long the disc takes to inflate from nothing to its full radius " +
             "after a press. This is the tap strength control. A disc that appears at " +
             "full size moves every particle inside it out to the surface in a single " +
             "substep, up to a full radius of displacement at once, and the pressure " +
             "solve answers that with a splash. Inflating bounds it to one substep's " +
             "worth of growth, so the same tap reads as a push. Lower is a harder tap; " +
             "0 restores the instant disc.")]
    public float radiusGrowthTime = 0.15f;

    [Tooltip("A press stays solid for at least this long even if the button comes " +
             "up sooner, so a quick tap still lands. A true click can begin and end " +
             "inside one frame, and without the latch a tap would often do nothing " +
             "at all. Note this also caps how far a tap inflates the disc, so it " +
             "changes tap strength as well as tap reliability.")]
    public float minimumPressDuration = 0.12f;

    [Header("Time")]
    [Tooltip("Allow the space bar to freeze and resume the simulation.")]
    public bool allowPause = true;

    [Tooltip("Draw the disc in the Scene view. The Game view has no equivalent: a " +
             "ring there would need its own shader, and a shader that nothing " +
             "references is stripped from a build.")]
    public bool showPointerGizmo = true;

    // The disc is tracked as a servoed position rather than as the raw cursor, so
    // its velocity is always a real, bounded quantity that can be handed to the
    // fluid with confidence.
    private float2 pointerPosition;
    private float2 pointerVelocity;

    private float pressLatch;
    private bool paused;
    private bool positioned;
    private float growth;

    // 0 to 1, how far the disc has inflated toward its full radius. Driven only
    // while the disc is solid, and collapsed the instant it is not -- which costs
    // nothing, because an inactive disc constrains no particle.

    /// <summary>True while the simulation is frozen.</summary>
    public bool Paused => paused;

    /// <summary>Disc radius the solver is being given right now, after inflation.</summary>
    public float ResolvedRadius => ResolveRadius() * growth;

    /// <summary>Disc radius at full inflation, in world units.</summary>
    public float FullRadius => ResolveRadius();

    private void Start()
    {
        if (solver == null) solver = GetComponent<SPH2D>();
        if (gpuSimulation == null) gpuSimulation = GetComponent<SPHComputeSimulation>();
        if (targetCamera == null) targetCamera = Camera.main;
    }

    /// <summary>
    /// Leaves the solver running when this component is switched off. Otherwise
    /// disabling the interactor mid-pause would freeze the simulation with nothing
    /// left to unfreeze it.
    /// </summary>
    private void OnDisable()
    {
        paused = false;

        Publish(default(PointerState), false);
    }

    private void Update()
    {
        float dt = Time.deltaTime;

        if (allowPause && ReadPausePressed()) paused = !paused;

        float discRadius = ResolveRadius();

        // Hovering without holding still moves the disc, so it is always where the
        // user last pointed and a press does not first have to drag it there.
        float2 target = pointerPosition;

        if (enableInteraction && TryGetCursorWorld(out float2 cursorWorld))
        {
            target = cursorWorld;
        }

        // Keep the whole disc inside the container. A disc overlapping a wall would
        // push fluid into it on every substep and fight the wall clamp, which reads
        // as jitter rather than as a solid object. The cost is that the last radius
        // of fluid against a wall cannot be reached.
        target = ClampToContainer(target, discRadius);

        if (!positioned)
        {
            // Start where the cursor is rather than at the origin, so the first
            // frame does not sweep the disc across the fluid to catch up.
            pointerPosition = target;
            positioned = true;
        }

        Servo(target, dt);

        if (ReadHeld())
        {
            pressLatch = math.max(0f, minimumPressDuration);
        }
        else
        {
            pressLatch = math.max(0f, pressLatch - dt);
        }

        // Solid only while held, and never while frozen. A solid disc in a frozen
        // simulation could only teleport the particles inside it, which would read
        // as the fluid jumping the moment it resumes.
        bool solid = pressLatch > 0f && enableInteraction && !paused;

        // Inflating is what makes a tap a push rather than a splash. The disc still
        // ends up solid and still cannot be entered; the difference is that a
        // particle deep inside it is walked out over many substeps instead of being
        // thrown to the surface in one. Note the displacement is bounded but the
        // particle gains no velocity from it, so the pressure field sees a slow
        // expansion rather than a void appearing.
        growth = solid
            ? math.saturate(growth + dt / math.max(1e-4f, radiusGrowthTime))
            : 0f;

        var state = new PointerState
        {
            position = pointerPosition,

            // Zeroed while paused: the disc keeps following the cursor, and without
            // this it would arrive at resume time carrying however far the cursor
            // travelled during the pause.
            velocity = paused ? float2.zero : pointerVelocity,

            // Grows from the centre outward. A floor keeps the radius non-zero on the
            // very first frame of a press, where the resolver would otherwise have no
            // direction to work with.
            radius = discRadius * math.max(growth, 1e-4f),

            restitution = restitution,
            friction = friction,
            active = solid,
        };

        Publish(state, paused);
    }

    private void Publish(PointerState state, bool frozen)
    {
        if (solver != null)
        {
            solver.pointer = state;
            solver.SetPaused(frozen);
        }

        if (gpuSimulation != null)
        {
            gpuSimulation.SetPointer(state);
            gpuSimulation.SetPaused(frozen);
        }
    }

    /// <summary>
    /// Moves the disc toward the target at no more than
    /// <see cref="maxPointerSpeed"/>, and derives its velocity from where it
    /// actually went.
    /// </summary>
    /// <remarks>
    /// The velocity is measured from the disc's own displacement, not from the
    /// cursor's. That is what makes the speed cap meaningful: the fluid can be given
    /// at most the velocity the disc legitimately travelled at, so a cursor
    /// teleport cannot inject a teleport-sized velocity into whatever it lands on.
    /// </remarks>
    private void Servo(float2 target, float dt)
    {
        float2 delta = target - pointerPosition;
        float distance = math.length(delta);
        float maxStep = math.max(0f, maxPointerSpeed) * dt;

        float2 next = distance <= maxStep
            ? target
            : pointerPosition + delta / distance * maxStep;

        pointerVelocity = dt > 1e-6f ? (next - pointerPosition) / dt : float2.zero;
        pointerPosition = next;
    }

    private float2 ClampToContainer(float2 position, float discRadius)
    {
        if (solver == null) return position;

        float2 centre = new float2(solver.transform.position.x, solver.transform.position.y);
        float2 half = new float2(solver.boxSize.x, solver.boxSize.y) * 0.5f;

        // max with zero so a disc larger than the container collapses to the centre
        // instead of inverting the clamp range.
        float2 limit = math.max(float2.zero, half - discRadius);

        return math.clamp(position, centre - limit, centre + limit);
    }

    private float ResolveRadius()
    {
        if (!radiusInSpacing || solver == null) return math.max(0.01f, radius);

        // The solver derives h as smoothingLengthInSpacing * spacing, so spacing
        // comes back out of h. Taking it from the solver rather than from
        // particleCount keeps the disc the same size when the spawn region or the
        // smoothing ratio changes, not just when the count does.
        float spacings = math.max(0.01f, solver.smoothingLengthInSpacing);
        float spacing = solver.smoothingLength / spacings;

        return math.max(0.01f, spacing * radiusInSpacings);
    }

    private bool TryGetCursorWorld(out float2 world)
    {
        world = pointerPosition;

        Camera camera = targetCamera != null ? targetCamera : Camera.main;
        if (camera == null) return false;

        Vector2 screen;

#if ENABLE_INPUT_SYSTEM
        Mouse mouse = Mouse.current;
        if (mouse == null) return false;
        screen = mouse.position.ReadValue();
#else
        screen = Input.mousePosition;
#endif

        // The fluid lives on the z = 0 plane, which is also where the render shader
        // builds every quad. Under an orthographic camera the z argument only sets
        // the z of the result, so the plane distance does not affect x or y.
        Vector3 point = camera.ScreenToWorldPoint(new Vector3(screen.x, screen.y, 0f));
        world = new float2(point.x, point.y);
        return true;
    }

    // ------------------------------------------------------------------
    // Input
    // ------------------------------------------------------------------
    //
    // Guarded rather than using one system directly. This project currently has
    // Active Input Handling set to "Input System Package (New)", and in that
    // configuration UnityEngine.Input throws at runtime -- but the setting is one
    // dropdown away from changing, and an unguarded file then fails to compile.
    // ENABLE_INPUT_SYSTEM / ENABLE_LEGACY_INPUT_MANAGER are Unity's own defines for
    // exactly this. Both can be true at once ("Both" in Player Settings), in which
    // case the new system wins.

    private bool ReadHeld()
    {
#if ENABLE_INPUT_SYSTEM
        Mouse mouse = Mouse.current;
        return mouse != null && mouse.leftButton.isPressed;
#else
        return Input.GetMouseButton(0);
#endif
    }

    private bool ReadPausePressed()
    {
#if ENABLE_INPUT_SYSTEM
        Keyboard keyboard = Keyboard.current;
        return keyboard != null && keyboard.spaceKey.wasPressedThisFrame;
#else
        return Input.GetKeyDown(KeyCode.Space);
#endif
    }

    private void OnDrawGizmos()
    {
        if (!showPointerGizmo || !positioned) return;

        float fullRadius = ResolveRadius();
        float solidRadius = fullRadius * growth;

        // The full radius stays visible while the disc is growing, so the size it is
        // inflating toward can be judged while it happens.
        if (solidRadius < fullRadius - 1e-4f)
        {
            DrawCircle(pointerPosition, fullRadius, new Color(1f, 1f, 1f, 0.15f));
        }

        DrawCircle(
            pointerPosition,
            solidRadius,
            pressLatch > 0f ? new Color(1f, 1f, 1f, 0.9f) : new Color(1f, 1f, 1f, 0.3f));
    }

    /// <summary>
    /// Drawn in segments rather than as a wire sphere: the disc is a 2D object, and
    /// a wireframe sphere would add depth cues that do not exist.
    /// </summary>
    private static void DrawCircle(float2 centre, float radius, Color colour)
    {
        if (radius <= 1e-4f) return;

        Gizmos.color = colour;

        const int segments = 48;
        Vector3 origin = new Vector3(centre.x, centre.y, 0f);
        Vector3 previous = origin + new Vector3(radius, 0f, 0f);

        for (int i = 1; i <= segments; i++)
        {
            float angle = i / (float)segments * math.PI * 2f;
            Vector3 next = origin + new Vector3(math.cos(angle), math.sin(angle), 0f) * radius;

            Gizmos.DrawLine(previous, next);
            previous = next;
        }
    }
}

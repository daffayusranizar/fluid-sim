using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Renders particles as smooth instanced discs instead of gizmos.
///
/// One quad mesh, one draw call, one structured buffer holding the whole
/// Particle2D array. The shader reads that buffer by SV_InstanceID, so particle
/// positions and velocities never have to be pulled apart into separate arrays --
/// the C# struct layout is the buffer layout.
///
/// This runs alongside the solver rather than inside it: the solver stays
/// Burst-compiled and native-only, and this is a plain managed component that
/// uploads a snapshot once per frame.
/// </summary>
[RequireComponent(typeof(SPH2D))]
public class ParticleRenderer2D : MonoBehaviour
{
    /// <summary>
    /// Ready-made speed ramps. All of them run dark to bright rather than
    /// cold-to-hot: a fluid is mostly slow bulk with a few fast features, so
    /// darkening the bulk and lighting up the fast parts reads as a surface
    /// catching light instead of as a temperature map.
    /// </summary>
    public enum ColourPreset
    {
        /// <summary>Uses the <see cref="colourMap"/> gradient below.</summary>
        Custom,
        DeepWater,
        Molten,
        Neon,
        Monochrome,

        /// <summary>
        /// Full-spectrum speed ramp: blue at rest through cyan, green and yellow to
        /// red at the top. Ordered slow-to-fast, so it reads without a legend.
        /// </summary>
        /// <remarks>
        /// A three-stop blue -> green -> red lerp looks muddy, because green-to-red
        /// passes through olive and blue-to-green passes through teal: the midpoint
        /// is desaturated. Going the long way round the wheel keeps every stop
        /// saturated, which is what makes speed differences legible in fine detail.
        /// </remarks>
        SpeedRainbow,
    }

    [Header("Source")]
    [Tooltip("Optional GPU solver. When set, the renderer binds the solver's own " +
             "particle buffer instead of packing and uploading CPU state, so a GPU " +
             "simulation renders with no per-frame readback and no upload. Leave " +
             "empty to render the Burst CPU solver instead.")]
    public SPHCompute gpuSource;

    [Header("Particle Appearance")]
    [Tooltip("World-space radius of one particle disc. Ignored when " +
             "autoParticleRadius is on.")]
    public float particleRadius = 0.12f;

    [Tooltip("Size impostors from the particle spacing instead of a fixed radius. " +
             "Without this the same number means 'separate dots' at 400 particles " +
             "and 'one body' at 5000, because the spacing changes with the count.")]
    public bool autoParticleRadius = true;

    [Tooltip("Impostor radius per unit of particle spacing. The world-space disc " +
             "radius is HALF of this, so a value of 0.8 gives discs 0.8 x spacing " +
             "across. Below ~1.13 the discs leave gaps and read as separate " +
             "particles; at ~1.13 they just tile the area into one sheet; above " +
             "that they overlap into a solid mass.")]
    public float particleRadiusInSpacing = 0.8f;

    [Tooltip("Speed mapped to the top of the colour gradient. Particles at or " +
             "above this render as the gradient's final colour. Ignored when " +
             "autoVelocityMax is on.")]
    public float velocityDisplayMax = 6f;

    [Tooltip("Derive the top of the colour ramp from the simulation's own peak " +
             "speed instead of a fixed number. The peak is already reduced every " +
             "frame for the CFL time step, so this costs nothing, and it removes " +
             "the failure mode where the fluid outgrows a hand-set maximum and " +
             "every particle clamps to the same colour.")]
    public bool autoVelocityMax = true;

    [Tooltip("Peak speed is multiplied by this to leave headroom, so the fastest " +
             "particles are not pinned to the last gradient stop.")]
    // Roughly half the peak, because the peak is outlier-dominated: the fastest
    // particle in a dam break can be several times the median, so mapping the whole
    // ramp to it leaves the bulk of the fluid compressed into the dark end.
    [Range(0.2f, 2f)] public float velocityMaxPadding = 0.5f;

    [Tooltip("How quickly the auto range follows the peak, in e-folds per second. " +
             "Lower is steadier; 0 freezes it.")]
    public float velocityMaxSmoothing = 2f;

    [Tooltip("Floor for the auto range. Without one, a fluid at rest collapses the " +
             "ramp to zero and every particle samples the top stop.")]
    public float minVelocityRange = 1f;

    [Tooltip("Ready-made ramp. Pick Custom to use the gradient below instead. " +
             "SpeedRainbow is ordered slow-to-fast: blue, green, red.")]
    public ColourPreset preset = ColourPreset.SpeedRainbow;

    [Tooltip("Only used when preset is Custom. Slow at the left, fast at the right.")]
    public Gradient colourMap = DefaultGradient();

    [Tooltip("Width of the gradient texture. Higher gives a smoother ramp.")]
    public int gradientResolution = 128;

    [Header("Impostor Shading")]
    [Tooltip("Direction the impostor light comes from, in world space. It needs a " +
             "positive Z to light the face pointing at the camera, which is where " +
             "the hemisphere normals point.")]
    public Vector3 impostorLightDirection = new Vector3(-0.4f, 0.5f, 0.8f);

    [Tooltip("Fraction of the particle colour that survives unlit. 1 disables shading.")]
    [Range(0f, 1f)] public float impostorAmbient = 0.35f;

    // Off by default: a highlight per particle is a strong "separate spheres" cue,
    // which is the opposite of what the colour ramp is trying to say.
    [Range(0f, 1f)] public float impostorSpecular = 0f;
    public float impostorShininess = 32f;

    [Header("Screen-space surface")]
    [Tooltip("Render the surface implied by the particle depths instead of the " +
             "individual discs. Removes the per-disc silhouettes that make a cloud " +
             "of particles read as dots. Costs an offscreen depth pass, two blur " +
             "passes and a full-screen composite.")]
    public bool useScreenSpaceSurface = false;

    [Tooltip("FluidSim/FluidSurface shader. Found automatically when left empty.")]
    public Shader surfaceShader;

    [Tooltip("Camera the screen-space surface is built for. Falls back to " +
             "Camera.main, which requires the camera to carry the MainCamera tag.")]
    public Camera targetCamera;

    [Tooltip("Depth similarity tolerance for the bilateral blur, in world units. " +
             "Roughly the particle radius: larger blends across the silhouette, " +
             "smaller leaves disc seams.")]
    public float surfaceRangeSigma = 0.05f;

    [Range(0f, 1f)] public float surfaceAmbient = 0.35f;
    [Range(0f, 1f)] public float surfaceSpecular = 0.6f;
    public float surfaceShininess = 48f;

    [Tooltip("Higher softens the silhouette against the depth jump; 0 leaves it hard.")]
    public float surfaceEdgeSoftness = 1f;

    [Tooltip("Direction the surface light comes from. Negative Z lights the side " +
             "facing the camera, which is where the reconstructed normals point.")]
    public Vector3 surfaceLightDirection = new Vector3(-0.4f, 0.5f, -0.8f);

    /// <summary>Bytes per particle. Must match the shader's Particle struct.</summary>
    private const int ParticleStride = sizeof(float) * 10;

    private Material material;
    private Mesh quad;
    private ComputeBuffer argsBuffer;

    // Only used on the CPU path. On the GPU path the solver's buffer is bound
    // directly, so nothing is allocated or uploaded here.
    private ComputeBuffer ownParticleBuffer;

    private ComputeBuffer boundBuffer;
    private int boundCount;

    // Starts at the manual value so the first frames are not mapped 0..0.
    private float smoothedVelocityMax;

    // Screen-space surface resources. Depth is RFloat, not RHalf: the reconstructed
    // normals come from screen-space derivatives of depth, and half precision at a
    // camera distance of ~10 quantises in steps of ~0.01, which is a visible
    // fraction of a particle radius.
    private const float FarDepth = 512f;

    private RenderTexture depthTarget;
    private RenderTexture blurTargetA;
    private RenderTexture blurTargetB;
    private RenderTexture colourTarget;

    private Material blurMaterial;
    private Material compositeMaterial;
    private CommandBuffer surfaceCommands;

    private CommandBuffer SurfaceCommands => surfaceCommands ??= new CommandBuffer { name = "Fluid surface" };

    private Texture2D gradientTexture;

    private SPH2D sim;
    private bool needsSettingsUpdate = true;

    /// <summary>Fallback used when preset is Custom and the gradient is empty.</summary>
    private static Gradient DefaultGradient()
    {
        return BuildGradient(
            new Color(0.10f, 0.10f, 0.12f),
            new Color(0.95f, 0.96f, 1.00f));
    }

    /// <summary>The gradient for a preset, or null for Custom.</summary>
    private static Gradient GradientForPreset(ColourPreset preset)
    {
        switch (preset)
        {
            case ColourPreset.DeepWater:
                return BuildGradient(
                    new Color(0.02f, 0.06f, 0.14f),
                    new Color(0.05f, 0.28f, 0.48f),
                    new Color(0.15f, 0.65f, 0.78f),
                    new Color(0.80f, 0.98f, 1.00f));

            case ColourPreset.Molten:
                return BuildGradient(
                    new Color(0.06f, 0.02f, 0.02f),
                    new Color(0.45f, 0.08f, 0.03f),
                    new Color(0.90f, 0.40f, 0.05f),
                    new Color(1.00f, 0.95f, 0.70f));

            case ColourPreset.Neon:
                return BuildGradient(
                    new Color(0.05f, 0.02f, 0.12f),
                    new Color(0.35f, 0.05f, 0.55f),
                    new Color(0.85f, 0.15f, 0.65f),
                    new Color(1.00f, 0.85f, 0.95f));

            case ColourPreset.Monochrome:
                return BuildGradient(
                    new Color(0.08f, 0.08f, 0.10f),
                    new Color(0.35f, 0.36f, 0.40f),
                    new Color(0.70f, 0.72f, 0.78f),
                    new Color(0.97f, 0.98f, 1.00f));

            case ColourPreset.SpeedRainbow:
                return BuildGradient(
                    new Color(0.10f, 0.25f, 0.90f),   // slow: blue
                    new Color(0.15f, 0.75f, 0.95f),   // cyan
                    new Color(0.20f, 0.90f, 0.35f),   // mid: green
                    new Color(0.95f, 0.90f, 0.20f),   // yellow
                    new Color(1.00f, 0.55f, 0.10f),   // orange
                    new Color(0.95f, 0.12f, 0.10f));  // fast: red

            default:
                return null;
        }
    }

    /// <summary>Builds a gradient with evenly spaced colour keys.</summary>
    private static Gradient BuildGradient(params Color[] colours)
    {
        int last = colours.Length - 1;
        var keys = new GradientColorKey[colours.Length];

        for (int i = 0; i < colours.Length; i++)
        {
            keys[i] = new GradientColorKey(colours[i], last > 0 ? i / (float)last : 0f);
        }

        var gradient = new Gradient();
        gradient.SetKeys(keys, new[]
        {
            new GradientAlphaKey(1f, 0f),
            new GradientAlphaKey(1f, 1f),
        });

        return gradient;
    }

    private void Awake()
    {
        // The renderer pulls from the solver rather than the solver pushing to the
        // renderer, so SPH2D stays pure simulation and knows nothing about drawing.
        sim = GetComponent<SPH2D>();

        // Auto-wire the GPU source when both live on the same object.
        if (gpuSource == null) gpuSource = GetComponent<SPHCompute>();

        Shader shader = Shader.Find("FluidSim/Particle2D");

        if (shader == null)
        {
            Debug.LogError("ParticleRenderer2D: shader 'FluidSim/Particle2D' not found. " +
                           "Is Assets/Shaders/2D/Particle2D.shader imported?");
            enabled = false;
            return;
        }

        material = new Material(shader);
        quad = CreateQuadMesh();

        if (surfaceShader == null) surfaceShader = Shader.Find("FluidSim/FluidSurface");

        if (surfaceShader != null)
        {
            blurMaterial = new Material(surfaceShader) { hideFlags = HideFlags.HideAndDontSave };
            compositeMaterial = new Material(surfaceShader) { hideFlags = HideFlags.HideAndDontSave };
            compositeMaterial.renderQueue = (int)RenderQueue.Transparent + 100;
        }
    }

    /// <summary>
    /// Renders the impostors offscreen, blurs their depth, and composites the
    /// implied surface over the camera.
    /// </summary>
    /// <remarks>
    /// The discs are never drawn to the camera in this mode. They exist only to fill
    /// the depth and colour targets; what reaches the screen is one full-screen
    /// quad shaded from the smoothed depth, which is what removes the per-disc
    /// silhouettes. The full-screen quad is a scene-space rectangle sized to the
    /// frustum rather than a URP render feature, so the whole effect stays inside
    /// this component and needs no renderer-asset changes.
    /// </remarks>
    private void RenderSurface(ComputeBuffer buffer, int count)
    {
        Camera cam = targetCamera != null ? targetCamera : Camera.main;
        if (cam == null) return;

        if (!EnsureBound(buffer, count)) return;
        if (needsSettingsUpdate) { needsSettingsUpdate = false; UpdateSettings(); }

        UpdateVelocityRange();
        UpdateParticleRadius();

        EnsureTargets(cam.pixelWidth, cam.pixelHeight);

        float far = FarDepth;
        float texelW = 1f / Mathf.Max(1, depthTarget.width);
        float texelH = 1f / Mathf.Max(1, depthTarget.height);
        var texelSize = new Vector4(texelW, texelH, depthTarget.width, depthTarget.height);

        // 1. impostors -> depth (min-reduced) and colour, offscreen.
        // The view/projection matrices are set explicitly: an offscreen command
        // buffer has no camera context, and the impostor shader derives view depth
        // from UNITY_MATRIX_V.
        if (quad == null || material == null || argsBuffer == null)
        {
            Debug.LogError($"ParticleRenderer2D.RenderSurface: missing resources " +
                           $"(quad={quad != null} material={material != null} args={argsBuffer != null}).");
            return;
        }

        CommandBuffer commands = SurfaceCommands;
        commands.Clear();
        commands.SetViewProjectionMatrices(cam.worldToCameraMatrix, cam.projectionMatrix);

        commands.SetRenderTarget(depthTarget);
        commands.ClearRenderTarget(false, true, new Color(far, 0f, 0f, 0f));
        commands.DrawMeshInstancedIndirect(quad, 0, material, 1, argsBuffer);

        commands.SetRenderTarget(colourTarget);
        commands.ClearRenderTarget(false, true, new Color(0f, 0f, 0f, 0f));
        commands.DrawMeshInstancedIndirect(quad, 0, material, 0, argsBuffer);
        Graphics.ExecuteCommandBuffer(commands);

        // 2. bilateral blur, one axis per pass
        blurMaterial.SetFloat("_FarDepth", far);
        blurMaterial.SetFloat("_RangeSigma", surfaceRangeSigma);
        blurMaterial.SetVector("_FluidDepth_TexelSize", texelSize);

        // Explicit pass index: Blit's default (-1) runs more than one pass, which
        // would composite the surface into the depth target.
        const int BlurPass = 1;

        blurMaterial.SetTexture("_FluidDepth", depthTarget);
        blurMaterial.SetVector("_BlurDir", new Vector2(1f, 0f));
        Graphics.Blit(depthTarget, blurTargetA, blurMaterial, BlurPass);

        blurMaterial.SetTexture("_FluidDepth", blurTargetA);
        blurMaterial.SetVector("_BlurDir", new Vector2(0f, 1f));
        Graphics.Blit(blurTargetA, blurTargetB, blurMaterial, BlurPass);

        // 3. composite the surface onto the camera as a frustum-sized quad
        float distance = cam.nearClipPlane + 0.1f;
        float halfHeight = cam.orthographic
            ? cam.orthographicSize
            : Mathf.Tan(cam.fieldOfView * 0.5f * Mathf.Deg2Rad) * distance;
        float halfWidth = halfHeight * cam.aspect;

        compositeMaterial.SetFloat("_FarDepth", far);
        compositeMaterial.SetVector("_FluidDepth_TexelSize", texelSize);
        compositeMaterial.SetTexture("_FluidDepth", blurTargetB);
        compositeMaterial.SetTexture("_FluidColour", colourTarget);
        compositeMaterial.SetVector("_ViewSize", new Vector4(halfWidth * 2f, halfHeight * 2f, 0f, 0f));
        compositeMaterial.SetVector("_SurfaceLightDir", surfaceLightDirection);
        compositeMaterial.SetFloat("_SurfaceAmbient", surfaceAmbient);
        compositeMaterial.SetFloat("_SurfaceSpecular", surfaceSpecular);
        compositeMaterial.SetFloat("_SurfaceShininess", surfaceShininess);
        compositeMaterial.SetFloat("_EdgeSoftness", surfaceEdgeSoftness);

        Matrix4x4 trs = Matrix4x4.TRS(
            cam.transform.position + cam.transform.forward * distance,
            cam.transform.rotation,
            new Vector3(halfWidth * 2f, halfHeight * 2f, 1f));

        Graphics.DrawMesh(quad, trs, compositeMaterial, 0);
    }

    private void EnsureTargets(int width, int height)
    {
        width = Mathf.Max(1, width);
        height = Mathf.Max(1, height);

        if (depthTarget != null && depthTarget.width == width && depthTarget.height == height) return;

        ReleaseTargets();

        depthTarget = new RenderTexture(width, height, 0, RenderTextureFormat.RFloat) { name = "FluidDepth" };
        blurTargetA = new RenderTexture(width, height, 0, RenderTextureFormat.RFloat) { name = "FluidDepthBlurA" };
        blurTargetB = new RenderTexture(width, height, 0, RenderTextureFormat.RFloat) { name = "FluidDepthBlurB" };
        colourTarget = new RenderTexture(width, height, 0, RenderTextureFormat.ARGB32) { name = "FluidColour" };

        foreach (var rt in new[] { depthTarget, blurTargetA, blurTargetB, colourTarget })
        {
            rt.filterMode = FilterMode.Bilinear;
            rt.wrapMode = TextureWrapMode.Clamp;
            rt.Create();
        }
    }

    private void ReleaseTargets()
    {
        foreach (var rt in new[] { depthTarget, blurTargetA, blurTargetB, colourTarget })
        {
            if (rt == null) continue;
            rt.Release();
            if (Application.isPlaying) Destroy(rt); else DestroyImmediate(rt);
        }

        depthTarget = blurTargetA = blurTargetB = colourTarget = null;
    }

    private void OnDestroy()
    {
        surfaceCommands?.Release();
        surfaceCommands = null;

        ReleaseTargets();

        if (blurMaterial != null)
        {
            if (Application.isPlaying) Destroy(blurMaterial); else DestroyImmediate(blurMaterial);
        }

        if (compositeMaterial != null)
        {
            if (Application.isPlaying) Destroy(compositeMaterial); else DestroyImmediate(compositeMaterial);
        }

        ReleaseBuffer(ref argsBuffer);
        ReleaseBuffer(ref ownParticleBuffer);

        if (material != null)
        {
            if (Application.isPlaying) Destroy(material);
            else DestroyImmediate(material);
        }

        if (quad != null)
        {
            if (Application.isPlaying) Destroy(quad);
            else DestroyImmediate(quad);
        }

        if (gradientTexture != null)
        {
            if (Application.isPlaying) Destroy(gradientTexture);
            else DestroyImmediate(gradientTexture);
        }
    }

    private void OnValidate()
    {
        // Inspector edits should be picked up without re-entering play mode
        needsSettingsUpdate = true;
    }

    private void LateUpdate()
    {
        if (material == null) return;

        // GPU path: bind the solver's own buffer. Nothing is copied, nothing is
        // uploaded, and the instance count is the solver's capacity.
        if (gpuSource != null && gpuSource.IsReady)
        {
            if (useScreenSpaceSurface && blurMaterial != null && compositeMaterial != null)
            {
                RenderSurface(gpuSource.ParticleBuffer, gpuSource.ParticleCapacity);
            }
            else
            {
                Render(gpuSource.ParticleBuffer, gpuSource.ParticleCapacity);
            }
            return;
        }

        // CPU path: pack from SPH2D and upload. Still supported so the two solvers
        // can be compared side by side.
        if (sim == null) return;

        NativeArray<Particle2D> particles = sim.Particles;

        if (!particles.IsCreated || particles.Length == 0) return;

        int count = particles.Length;

        if (ownParticleBuffer == null || ownParticleBuffer.count != count)
        {
            ReleaseBuffer(ref ownParticleBuffer);
            ownParticleBuffer = new ComputeBuffer(count, ParticleStride);
            boundBuffer = null;
        }

        ownParticleBuffer.SetData(particles);
        Render(ownParticleBuffer, count);
    }

    /// <summary>
    /// Points the colour ramp at the range the fluid actually occupies.
    /// </summary>
    /// <remarks>
    /// A fixed top-of-ramp value saturates the moment the fluid moves faster than
    /// it: every particle above the maximum samples the same final stop, and a
    /// multi-colour ramp renders as one flat colour. The peak speed is already
    /// available from SPHCompute's per-frame reduction, so the ramp can follow the
    /// simulation instead of being guessed.
    /// </remarks>
    /// <summary>
    /// Sizes the impostors relative to the particle spacing rather than to a fixed
    /// world radius.
    /// </summary>
    /// <remarks>
    /// The spacing is h / smoothingLengthInSpacing, so it can be recovered from the
    /// smoothing length the solver already publishes. This matters for more than
    /// convenience: whether the fluid reads as a body or as a cloud of dots depends
    /// on the ratio of disc size to spacing, so a fixed radius silently changes
    /// meaning every time the particle count changes.
    /// </remarks>
    private void UpdateParticleRadius()
    {
        float radius = particleRadius;

        if (autoParticleRadius)
        {
            float h = gpuSource != null ? gpuSource.SmoothingLength : 0f;
            if (h <= 0f && sim != null) h = sim.Parameters.smoothingLength;

            if (h > 0f)
            {
                float ratio = sim != null ? Mathf.Max(0.01f, sim.smoothingLengthInSpacing) : 2.2f;
                radius = (h / ratio) * particleRadiusInSpacing;
            }
        }

        material.SetFloat("_ParticleRadius", Mathf.Max(0.0001f, radius));
    }

    private void UpdateVelocityRange()
    {
        float target = velocityDisplayMax;

        if (autoVelocityMax && gpuSource != null && gpuSource.LastMaxSpeed > 1e-4f)
        {
            target = gpuSource.LastMaxSpeed * velocityMaxPadding;
        }

        target = Mathf.Max(Mathf.Max(0.0001f, minVelocityRange), target);

        if (smoothedVelocityMax <= 0f || target > smoothedVelocityMax * 2f || target < smoothedVelocityMax * 0.5f)
        {
            // First frame, or a change too large to ease into: snap. Otherwise a
            // fluid that starts moving fast spends seconds fading up from the
            // resting range, and nothing is readable in the meantime.
            smoothedVelocityMax = target;
        }
        else if (velocityMaxSmoothing > 0f)
        {
            float k = 1f - Mathf.Exp(-velocityMaxSmoothing * Mathf.Max(0f, Time.deltaTime));
            smoothedVelocityMax = Mathf.Lerp(smoothedVelocityMax, target, k);
        }

        material.SetFloat("_VelocityMax", Mathf.Max(0.0001f, smoothedVelocityMax));
    }

    /// <summary>
    /// Binds <paramref name="buffer"/> and issues one instanced draw.
    /// </summary>
    /// <remarks>
    /// The material buffer and the args buffer are only rebuilt when the source or
    /// the count changes, so a steady frame is a single draw call with no buffer
    /// traffic. The instance count lives inside the args buffer, which is why it
    /// has to be tracked alongside the binding rather than queried back.
    /// </remarks>
    /// <summary>
    /// Points the material at <paramref name="buffer"/> and sizes the args buffer.
    /// Split out of <see cref="Render"/> so the screen-space path can bind the same
    /// way without issuing the disc draw.
    /// </summary>
    private bool EnsureBound(ComputeBuffer buffer, int count)
    {
        if (buffer == null || count <= 0) return false;

        if (boundBuffer != buffer || boundCount != count)
        {
            material.SetBuffer("Particles", buffer);

            ReleaseBuffer(ref argsBuffer);
            argsBuffer = CreateArgsBuffer(quad, count);

            boundBuffer = buffer;
            boundCount = count;
        }

        return true;
    }

    private void Render(ComputeBuffer buffer, int count)
    {
        if (!EnsureBound(buffer, count)) return;

        if (needsSettingsUpdate)
        {
            needsSettingsUpdate = false;
            UpdateSettings();
        }

        UpdateVelocityRange();
        UpdateParticleRadius();

        // Bounds are deliberately huge: the geometry is built in world space from
        // the buffer, so Unity has no way to cull it correctly from the mesh alone.
        var bounds = new Bounds(Vector3.zero, Vector3.one * 10000f);
        Graphics.DrawMeshInstancedIndirect(quad, 0, material, bounds, argsBuffer);
    }

    private void UpdateSettings()
    {
        // _ParticleRadius and _VelocityMax are owned by the per-frame updates, since
        // both depend on state that only exists once the solver has been seeded.

        material.SetVector("_LightDir", impostorLightDirection);
        material.SetFloat("_Ambient", impostorAmbient);
        material.SetFloat("_Specular", impostorSpecular);
        material.SetFloat("_Shininess", impostorShininess);

        Gradient ramp = GradientForPreset(preset) ?? colourMap;
        TextureFromGradient(ref gradientTexture, gradientResolution, ramp);
        material.SetTexture("ColourMap", gradientTexture);
    }

    /// <summary>
    /// Bakes a Gradient into a 1xN texture so the shader can sample it by speed.
    /// </summary>
    public static void TextureFromGradient(
        ref Texture2D texture, int width, Gradient gradient, FilterMode filterMode = FilterMode.Bilinear)
    {
        if (width != texture?.width)
        {
            if (texture != null)
            {
                if (Application.isPlaying) Destroy(texture);
                else DestroyImmediate(texture);
            }

            texture = new Texture2D(width, 1);
        }

        gradient ??= new Gradient();

        texture.wrapMode = TextureWrapMode.Clamp;
        texture.filterMode = filterMode;

        var colours = new Color[width];

        for (int i = 0; i < width; i++)
        {
            colours[i] = gradient.Evaluate(i / (width - 1f));
        }

        texture.SetPixels(colours);
        texture.Apply();
    }

    /// <summary>A unit quad centred on the origin, with UVs spanning 0..1.</summary>
    private static Mesh CreateQuadMesh()
    {
        var mesh = new Mesh { name = "ParticleQuad2D" };

        mesh.vertices = new[]
        {
            new Vector3(-0.5f, -0.5f, 0f),
            new Vector3(0.5f, -0.5f, 0f),
            new Vector3(0.5f, 0.5f, 0f),
            new Vector3(-0.5f, 0.5f, 0f),
        };

        mesh.uv = new[]
        {
            new Vector2(0f, 0f),
            new Vector2(1f, 0f),
            new Vector2(1f, 1f),
            new Vector2(0f, 1f),
        };

        mesh.triangles = new[] { 0, 2, 1, 0, 3, 2 };
        mesh.RecalculateBounds();

        return mesh;
    }

    private static ComputeBuffer CreateArgsBuffer(Mesh mesh, int instanceCount)
    {
        const int subMesh = 0;

        var args = new uint[5];
        args[0] = mesh.GetIndexCount(subMesh);
        args[1] = (uint)instanceCount;
        args[2] = mesh.GetIndexStart(subMesh);
        args[3] = mesh.GetBaseVertex(subMesh);
        args[4] = 0;

        var buffer = new ComputeBuffer(1, 5 * sizeof(uint), ComputeBufferType.IndirectArguments);
        buffer.SetData(args);

        return buffer;
    }

    private static void ReleaseBuffer(ref ComputeBuffer buffer)
    {
        buffer?.Release();
        buffer = null;
    }
}

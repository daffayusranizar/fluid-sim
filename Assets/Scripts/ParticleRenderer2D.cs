using Unity.Collections;
using UnityEngine;

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
    }

    [Header("Particle Appearance")]
    [Tooltip("World-space radius of one particle disc.")]
    public float particleRadius = 0.12f;

    [Tooltip("Speed mapped to the top of the colour gradient. Particles at or " +
             "above this render as the gradient's final colour.")]
    public float velocityDisplayMax = 6f;

    [Tooltip("Ready-made ramp. Pick Custom to use the gradient below instead.")]
    public ColourPreset preset = ColourPreset.DeepWater;

    [Tooltip("Only used when preset is Custom. Slow at the left, fast at the right.")]
    public Gradient colourMap = DefaultGradient();

    [Tooltip("Width of the gradient texture. Higher gives a smoother ramp.")]
    public int gradientResolution = 128;

    private Material material;
    private Mesh quad;
    private ComputeBuffer argsBuffer;
    private ComputeBuffer particleBuffer;
    private Texture2D gradientTexture;

    private SPH2D sim;
    private int particleCapacity;
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

        Shader shader = Shader.Find("FluidSim/Particle2D");

        if (shader == null)
        {
            Debug.LogError("ParticleRenderer2D: shader 'FluidSim/Particle2D' not found. " +
                           "Is Assets/Shaders/Particle2D.shader imported?");
            enabled = false;
            return;
        }

        material = new Material(shader);
        quad = CreateQuadMesh();
    }

    private void OnDestroy()
    {
        ReleaseBuffer(ref argsBuffer);
        ReleaseBuffer(ref particleBuffer);

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
        if (material == null || sim == null) return;

        NativeArray<Particle2D> particles = sim.Particles;

        if (!particles.IsCreated || particles.Length == 0) return;

        EnsureBufferCapacity(particles.Length);
        particleBuffer.SetData(particles);

        if (needsSettingsUpdate)
        {
            needsSettingsUpdate = false;
            UpdateSettings();
        }

        // Bounds are deliberately huge: the geometry is built in world space from
        // the buffer, so Unity has no way to cull it correctly from the mesh alone.
        var bounds = new Bounds(Vector3.zero, Vector3.one * 10000f);
        Graphics.DrawMeshInstancedIndirect(quad, 0, material, bounds, argsBuffer);
    }

    private void EnsureBufferCapacity(int count)
    {
        if (particleBuffer != null && particleCapacity == count) return;

        ReleaseBuffer(ref particleBuffer);

        // 40 bytes matches the Particle2D layout the shader declares
        const int stride = sizeof(float) * 10;

        particleBuffer = new ComputeBuffer(count, stride);
        particleCapacity = count;

        material.SetBuffer("Particles", particleBuffer);

        ReleaseBuffer(ref argsBuffer);
        argsBuffer = CreateArgsBuffer(quad, count);
    }

    private void UpdateSettings()
    {
        material.SetFloat("_ParticleRadius", Mathf.Max(0.0001f, particleRadius));
        material.SetFloat("_VelocityMax", Mathf.Max(0.0001f, velocityDisplayMax));

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

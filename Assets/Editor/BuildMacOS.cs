using System.IO;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEditor.OSXStandalone;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// The macOS export, as three commands instead of three windows.
///
/// The point of doing this from a script is that a GUI build is unrepeatable: the
/// architecture, the window mode and the bundle identifier live in project
/// settings, so a build six months from now depends on what someone happened to
/// leave in a dropdown. Here they are applied by the build that depends on them.
///
/// Run any of the three from the Editor's Fluid menu, or headless:
///
///   Unity -batchmode -quit -projectPath . -executeMethod FluidBuild.WireScene
///   Unity -batchmode -quit -projectPath . -executeMethod FluidBuild.BuildMacOS
///
/// Unity refuses to open a project that another Unity instance has open, so the
/// Editor has to be closed before any of the headless forms will run.
/// </summary>
public static class FluidBuild
{
    private const string ScenePath = "Assets/Scenes/SampleScene.unity";
    private const string ParticleShaderPath = "Assets/Shaders/2D/Particle2D.shader";
    private const string OutputPath = "Builds/FluidSim.app";

    // Derived from the project's existing companyName rather than invented, so the
    // identifier is at least traceable back to a setting someone chose.
    private const string BundleIdentifier = "com.defaultcompany.fluidsim";
    private const string ProductName = "FluidSim";

    /// <summary>
    /// Puts the scene into a state the build path can rely on.
    /// </summary>
    /// <remarks>
    /// Two things, both of which are silently wrong rather than loudly broken if
    /// they are missing:
    ///
    /// 1. <c>ParticleRenderer2D.particleShader</c> gets a real reference. The
    ///    renderer used to reach its shader only through <c>Shader.Find</c>, and a
    ///    shader that nothing references is stripped from a build. The Editor keeps
    ///    rendering, the built app renders nothing, and the only evidence is a line
    ///    in the Player log.
    ///
    /// 2. The interactor component is added. Without it the build runs, and the
    ///    mouse does nothing at all.
    ///
    /// Written to file rather than patched by hand because the alternative is
    /// hand-writing a guid into scene YAML and hoping it matches the .meta Unity
    /// generates.
    /// </remarks>
    [MenuItem("Fluid/Wire Scene")]
    public static void WireScene()
    {
        Scene scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);

        var solver = Object.FindAnyObjectByType<SPH2D>();
        var renderer = Object.FindAnyObjectByType<ParticleRenderer2D>();

        if (solver == null || renderer == null)
        {
            Fail($"expected an SPH2D and a ParticleRenderer2D in {ScenePath} " +
                 $"(found solver={solver != null}, renderer={renderer != null}).");
            return;
        }

        Shader particleShader = AssetDatabase.LoadAssetAtPath<Shader>(ParticleShaderPath);

        if (particleShader == null)
        {
            Fail($"could not load {ParticleShaderPath}.");
            return;
        }

        var serialised = new SerializedObject(renderer);
        serialised.FindProperty("particleShader").objectReferenceValue = particleShader;
        serialised.ApplyModifiedPropertiesWithoutUndo();

        bool addedInteractor = false;

        if (solver.GetComponent<SPHInteractor2D>() == null)
        {
            solver.gameObject.AddComponent<SPHInteractor2D>();
            addedInteractor = true;
        }

        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene, ScenePath, false);
        AssetDatabase.SaveAssets();

        Debug.Log($"[FluidBuild] wired {ScenePath}: " +
                  $"particleShader={particleShader.name}, interactorAdded={addedInteractor}");
    }

    /// <summary>
    /// Applies the settings the macOS build depends on.
    /// </summary>
    [MenuItem("Fluid/Apply macOS Player Settings")]
    public static void ApplyMacOSPlayerSettings()
    {
        PlayerSettings.productName = ProductName;
        PlayerSettings.SetApplicationIdentifier(NamedBuildTarget.Standalone, BundleIdentifier);

        // Windowed rather than the fullscreen-at-native-resolution the template
        // ships. The whole point of this build is dragging a cursor around inside
        // the container, and a fullscreen window on a laptop gives the fluid a lot
        // of empty space either side of a square box.
        PlayerSettings.fullScreenMode = FullScreenMode.Windowed;
        PlayerSettings.defaultScreenWidth = 1280;
        PlayerSettings.defaultScreenHeight = 800;
        PlayerSettings.resizableWindow = true;

        // The fluid is only interesting while the cursor is over the window, and the
        // solver is happy to spend a core or two on an idle one.
        PlayerSettings.runInBackground = false;

        // Apple silicon only.
        //
        // NOT PlayerSettings.SetArchitecture: its own documentation limits it to
        // iOS, tvOS and visionOS, so setting it here would look like it worked and
        // silently leave the default architecture in place. macOS is reached through
        // OSXStandalone, which is also what BuildTarget.StandaloneOSX points at.
        UserBuildSettings.architecture = OSArchitecture.ARM64;

        // Mono, not IL2CPP. IL2CPP's advantages are smaller binaries, faster code
        // and no JIT, which matter when shipping to other people; they cost minutes
        // per build, which is paid every time during development. Burst-compiled
        // solver passes are unaffected either way. Switch to IL2CPP for the build
        // that leaves this machine.
        PlayerSettings.SetScriptingBackend(NamedBuildTarget.Standalone, ScriptingImplementation.Mono2x);

        AssetDatabase.SaveAssets();

        Debug.Log($"[FluidBuild] macOS player settings: product={ProductName} " +
                  $"id={BundleIdentifier} window={PlayerSettings.defaultScreenWidth}x" +
                  $"{PlayerSettings.defaultScreenHeight} " +
                  $"arch={UserBuildSettings.architecture} backend=Mono2x");
    }

    /// <summary>
    /// Builds the standalone app, refusing to do so if the scene is not wired.
    /// </summary>
    [MenuItem("Fluid/Build macOS")]
    public static void BuildMacOS()
    {
        if (EditorUserBuildSettings.activeBuildTarget != BuildTarget.StandaloneOSX &&
            !EditorUserBuildSettings.SwitchActiveBuildTarget(NamedBuildTarget.Standalone, BuildTarget.StandaloneOSX))
        {
            Fail("could not switch the active build target to macOS.");
            return;
        }

        ApplyMacOSPlayerSettings();

        // Open and check the scene before building rather than after. A build that
        // produces a black window is worse than a build that refuses to run: the
        // failure surfaces at the far end, in an app, as "nothing is drawn".
        EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);

        if (!VerifySceneWiring(out string problem))
        {
            Fail($"refusing to build. {problem} Run Fluid > Wire Scene first.");
            return;
        }

        string output = Path.GetFullPath(OutputPath);
        Directory.CreateDirectory(Path.GetDirectoryName(output));

        var options = new BuildPlayerOptions
        {
            scenes = new[] { ScenePath },
            locationPathName = output,
            target = BuildTarget.StandaloneOSX,
            targetGroup = BuildTargetGroup.Standalone,
            options = BuildOptions.None,
        };

        BuildReport report = BuildPipeline.BuildPlayer(options);
        BuildSummary summary = report.summary;

        if (summary.result != BuildResult.Succeeded)
        {
            Fail($"build {summary.result} after {summary.totalTime}, " +
                 $"{summary.totalErrors} error(s).");
            return;
        }

        Debug.Log($"[FluidBuild] built {output} " +
                  $"({summary.totalSize / (1024 * 1024)} MB, {summary.totalTime}). " +
                  $"Verify with: lipo -archs \"{output}/Contents/MacOS/{ProductName}\"");
    }

    /// <summary>
    /// Reports the first thing in the scene that would make the built app run but
    /// appear to do nothing.
    /// </summary>
    /// <remarks>
    /// Every check here is for a failure that the Editor cannot reproduce, which is
    /// the whole reason a separate check exists: the Editor resolves shaders by name
    /// and a build does not, so a missing reference is invisible until the app runs
    /// somewhere with no Editor to explain it.
    /// </remarks>
    private static bool VerifySceneWiring(out string problem)
    {
        var renderer = Object.FindAnyObjectByType<ParticleRenderer2D>();

        if (renderer == null)
        {
            problem = "No ParticleRenderer2D in the scene.";
            return false;
        }

        if (renderer.particleShader == null)
        {
            problem = "ParticleRenderer2D.particleShader is unassigned, so the " +
                      "shader would be stripped from the build and the app would " +
                      "render a black window.";
            return false;
        }

        if (Object.FindAnyObjectByType<SPHInteractor2D>() == null)
        {
            problem = "No SPHInteractor2D in the scene, so the mouse would do nothing.";
            return false;
        }

        problem = null;
        return true;
    }

    private static void Fail(string message)
    {
        Debug.LogError($"[FluidBuild] {message}");

        // Non-zero exit so a headless build fails the shell command that ran it.
        // Without this, -quit exits 0 whatever happened, and a broken build looks
        // like a successful one to anything scripting it.
        if (Application.isBatchMode) EditorApplication.Exit(1);
    }
}

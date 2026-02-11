#if UNITY_EDITOR

using System;
using System.Reflection;
using UnityEditor;
using UnityEngine;

public class FullScreenHotkeyHandler : MonoBehaviour
{
    bool makeFullscreenAtStart = true;
	
	// Enable fullscreen when starting game
    void Start() { if (makeFullscreenAtStart) { FullscreenGameView.Toggle(); } }

    void Update()
    {
		// Toggle fullscreen when hotkey pressed
        if (Input.GetKeyDown(KeyCode.Backslash))
        {
            FullscreenGameView.Toggle();
        }
        // Toggle game view orientation (portrait/landscape) when hotkey pressed
        if (Input.GetKeyDown(KeyCode.Slash))
        {
            GameViewOrientation.Toggle();
        }
    }
}

// Below code from: https://gist.github.com/fnuecke/d4275087cc7969257eae0f939fac3d2f
// My Improvement: Fixed bug where stuck in fullscreen after re-compiling
public static class FullscreenGameView
{
    static readonly Type GameViewType = Type.GetType("UnityEditor.GameView,UnityEditor");
    static readonly PropertyInfo ShowToolbarProperty = GameViewType.GetProperty("showToolbar", BindingFlags.Instance | BindingFlags.NonPublic);
    static readonly object False = false; // Only box once. This is a matter of principle.

    static EditorWindow instance;

    // Exit fullscreen when re-compiling game during Game session (to fix bug where can't leave fullscreen)
    static FullscreenGameView() { AssemblyReloadEvents.beforeAssemblyReload += OnBeforeAssemblyReload; }
    private static void OnBeforeAssemblyReload() { if (instance != null) { instance.Close(); instance = null; } }
    
    [MenuItem("Window/General/Game (Fullscreen) %#&2", priority = 2)]
    public static void Toggle()
    {
        if (GameViewType == null)
        {
            Debug.LogError("GameView type not found.");
            return;
        }

        if (ShowToolbarProperty == null)
        {
            Debug.LogWarning("GameView.showToolbar property not found.");
        }

        if (instance != null)
        {
            instance.Close();
            instance = null;
        }
        else
        {
            instance = (EditorWindow) ScriptableObject.CreateInstance(GameViewType);

            ShowToolbarProperty?.SetValue(instance, False);

            var desktopResolution = new Vector2(Screen.currentResolution.width, Screen.currentResolution.height);
            Debug.Log($"[FullscreenGameView] Setting Game View to fullscreen with resolution {desktopResolution.x}x{desktopResolution.y}");
            var fullscreenRect = new Rect(Vector2.zero, desktopResolution);
            instance.ShowPopup();
            instance.position = fullscreenRect;
            instance.Focus();
        }
    }
}

public static class GameViewOrientation
{
    static readonly Assembly EditorAssembly = typeof(Editor).Assembly;
    static readonly Type GameViewType = EditorAssembly.GetType("UnityEditor.GameView");
    static readonly Type GameViewSizesType = EditorAssembly.GetType("UnityEditor.GameViewSizes");
    static readonly Type GameViewSizeType = EditorAssembly.GetType("UnityEditor.GameViewSize");
    static readonly Type GameViewSizeTypeEnum = EditorAssembly.GetType("UnityEditor.GameViewSizeType");

    static object SizesInstance
    {
        get
        {
            var singletonType = typeof(ScriptableSingleton<>).MakeGenericType(GameViewSizesType);
            return singletonType.GetProperty("instance").GetValue(null);
        }
    }

    public static void Toggle()
    {
        if (GameViewType == null)
        {
            Debug.LogError("[GameViewOrientation] GameView type not found.");
            return;
        }

        var gameView = EditorWindow.GetWindow(GameViewType);
        if (gameView == null) return;

        // Get the current rendered size
        var targetSizeProp = GameViewType.GetProperty("targetSize", BindingFlags.NonPublic | BindingFlags.Instance);
        if (targetSizeProp == null)
        {
            Debug.LogError("[GameViewOrientation] targetSize property not found.");
            return;
        }

        Vector2 currentSize = (Vector2)targetSizeProp.GetValue(gameView);
        int newWidth = Mathf.RoundToInt(currentSize.y);
        int newHeight = Mathf.RoundToInt(currentSize.x);

        int sizeIndex = FindOrAddSize(newWidth, newHeight);
        if (sizeIndex < 0) return;

        // Apply the new size
        var selectedSizeIndexProp = GameViewType.GetProperty("selectedSizeIndex", BindingFlags.NonPublic | BindingFlags.Instance);
        if (selectedSizeIndexProp == null)
        {
            Debug.LogError("[GameViewOrientation] selectedSizeIndex property not found.");
            return;
        }

        selectedSizeIndexProp.SetValue(gameView, sizeIndex);
        gameView.Repaint();

        Debug.Log($"[GameViewOrientation] Toggled to {newWidth}x{newHeight}");
    }

    static object GetCurrentGroup()
    {
        var currentGroupTypeProp = GameViewSizesType.GetProperty("currentGroupType");
        var groupType = currentGroupTypeProp.GetValue(SizesInstance);
        var getGroupMethod = GameViewSizesType.GetMethod("GetGroup");
        return getGroupMethod.Invoke(SizesInstance, new object[] { (int)groupType });
    }

    static int FindOrAddSize(int width, int height)
    {
        var group = GetCurrentGroup();
        if (group == null) return -1;

        var groupObj = group.GetType();
        var getTotalCount = groupObj.GetMethod("GetTotalCount");
        var getGameViewSize = groupObj.GetMethod("GetGameViewSize");
        int totalCount = (int)getTotalCount.Invoke(group, null);

        // Search all existing sizes (built-in + custom)
        for (int i = 0; i < totalCount; i++)
        {
            var size = getGameViewSize.Invoke(group, new object[] { i });
            int w = (int)size.GetType().GetProperty("width").GetValue(size);
            int h = (int)size.GetType().GetProperty("height").GetValue(size);
            if (w == width && h == height)
                return i;
        }

        // Not found — add a new custom size
        var fixedResolution = Enum.Parse(GameViewSizeTypeEnum, "FixedResolution");
        var ctor = GameViewSizeType.GetConstructor(new Type[] { GameViewSizeTypeEnum, typeof(int), typeof(int), typeof(string) });
        var newSize = ctor.Invoke(new object[] { fixedResolution, width, height, $"{width}x{height}" });

        groupObj.GetMethod("AddCustomSize").Invoke(group, new object[] { newSize });

        return (int)getTotalCount.Invoke(group, null) - 1;
    }
}

#endif
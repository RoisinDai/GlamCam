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
        // Toggle Windows display orientation (landscape/portrait) when hotkey pressed
        if (Input.GetKeyDown(KeyCode.Slash))
        {
            WindowsDisplayOrientation.Toggle();
            FullscreenGameView.Toggle(); // close
            FullscreenGameView.Toggle(); // reopen at new resolution
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

public static class WindowsDisplayOrientation
{
    // Windows API constants
    const int DMDO_DEFAULT = 0;  // Landscape
    const int DMDO_90 = 1;       // Portrait (rotated 90° CCW)
    const int DM_DISPLAYORIENTATION = 0x00000080;
    const int DM_PELSWIDTH = 0x00080000;
    const int DM_PELSHEIGHT = 0x00100000;
    const int ENUM_CURRENT_SETTINGS = -1;
    const int CDS_RESET = 0x40000000;

    [System.Runtime.InteropServices.StructLayout(System.Runtime.InteropServices.LayoutKind.Sequential, CharSet = System.Runtime.InteropServices.CharSet.Ansi)]
    struct DEVMODE
    {
        [System.Runtime.InteropServices.MarshalAs(System.Runtime.InteropServices.UnmanagedType.ByValTStr, SizeConst = 32)]
        public string dmDeviceName;
        public short dmSpecVersion;
        public short dmDriverVersion;
        public short dmSize;
        public short dmDriverExtra;
        public int dmFields;
        public int dmPositionX;
        public int dmPositionY;
        public int dmDisplayOrientation;
        public int dmDisplayFixedOutput;
        public short dmColor;
        public short dmDuplex;
        public short dmYResolution;
        public short dmTTOption;
        public short dmCollate;
        [System.Runtime.InteropServices.MarshalAs(System.Runtime.InteropServices.UnmanagedType.ByValTStr, SizeConst = 32)]
        public string dmFormName;
        public short dmLogPixels;
        public int dmBitsPerPel;
        public int dmPelsWidth;
        public int dmPelsHeight;
        public int dmDisplayFlags;
        public int dmDisplayFrequency;
        public int dmICMMethod;
        public int dmICMIntent;
        public int dmMediaType;
        public int dmDitherType;
        public int dmReserved1;
        public int dmReserved2;
        public int dmPanningWidth;
        public int dmPanningHeight;
    }

    [System.Runtime.InteropServices.DllImport("user32.dll")]
    static extern bool EnumDisplaySettings(string deviceName, int modeNum, ref DEVMODE devMode);

    [System.Runtime.InteropServices.DllImport("user32.dll")]
    static extern int ChangeDisplaySettings(ref DEVMODE devMode, int flags);

    public static void Toggle()
    {
        DEVMODE dm = new DEVMODE();
        dm.dmSize = (short)System.Runtime.InteropServices.Marshal.SizeOf(typeof(DEVMODE));

        if (!EnumDisplaySettings(null, ENUM_CURRENT_SETTINGS, ref dm))
        {
            Debug.LogError("[WindowsDisplayOrientation] Failed to get current display settings.");
            return;
        }

        // Toggle between landscape (0°) and portrait (90°)
        if (dm.dmDisplayOrientation == DMDO_DEFAULT)
        {
            dm.dmDisplayOrientation = DMDO_90;
            // Swap width and height for portrait
            int temp = dm.dmPelsWidth;
            dm.dmPelsWidth = dm.dmPelsHeight;
            dm.dmPelsHeight = temp;
        }
        else
        {
            dm.dmDisplayOrientation = DMDO_DEFAULT;
            // Swap width and height for landscape
            int temp = dm.dmPelsWidth;
            dm.dmPelsWidth = dm.dmPelsHeight;
            dm.dmPelsHeight = temp;
        }

        dm.dmFields = DM_DISPLAYORIENTATION | DM_PELSWIDTH | DM_PELSHEIGHT;

        int result = ChangeDisplaySettings(ref dm, CDS_RESET);
        if (result == 0)
        {
            Debug.Log($"[WindowsDisplayOrientation] Toggled to {(dm.dmDisplayOrientation == DMDO_DEFAULT ? "Landscape" : "Portrait")} ({dm.dmPelsWidth}x{dm.dmPelsHeight})");
        }
        else
        {
            Debug.LogError($"[WindowsDisplayOrientation] ChangeDisplaySettings failed with code {result}");
        }
    }
}

#endif
using UnityEngine;

public class CameraController : MonoBehaviour
{
    [SerializeField] private Camera avatarCamera;
    [SerializeField] private Camera debugCamera;
    
    private bool isAvatarCameraActive = true;
    
    // Static property so other scripts can check if debug camera is active
    public static bool IsDebugCameraActive { get; private set; } = false;

    void Start()
    {
        // Initialize with AvatarCamera active
        if (avatarCamera != null && debugCamera != null)
        {
            avatarCamera.enabled = true;
            debugCamera.enabled = false;
        }
        else
        {
            Debug.LogWarning("CameraController: Please assign both cameras in the Inspector!");
        }
    }

    void Update()
    {
        // Check for 'C' key press
        if (Input.GetKeyDown(KeyCode.C))
        {
            ToggleCamera();
        }
    }

    private void ToggleCamera()
    {
        if (avatarCamera == null || debugCamera == null)
        {
            Debug.LogWarning("CameraController: One or both cameras are not assigned!");
            return;
        }

        isAvatarCameraActive = !isAvatarCameraActive;
        IsDebugCameraActive = !isAvatarCameraActive;

        avatarCamera.enabled = isAvatarCameraActive;
        debugCamera.enabled = !isAvatarCameraActive;

        Debug.Log($"Switched to {(isAvatarCameraActive ? "AvatarCamera" : "DebugCamera")}");
    }
}

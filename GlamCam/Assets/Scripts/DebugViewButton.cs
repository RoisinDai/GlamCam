using UnityEngine;

public class DebugViewButton : MonoBehaviour
{
    [Header("Interaction")]
    public float cooldownSeconds = 0.5f;

    [Header("Raycast Settings")]
    public Camera avatarCamera;
    public float maxRayDistance = 30f;
    public LayerMask uiLayerMask;
    private float _nextAllowedTime = 0f;
    private Collider _collider;
    [SerializeField] private GameObject handCursorObj;
    [SerializeField] private GameObject handCursorLeftObj;
    private HandCursorFollower _handCursor;
    private HandCursorFollower _handCursorLeft;
    private HoverScale _hoverScale;
    private bool _wasFistInside = false;
    private bool _wasFistInsideLeft = false;
    
    [Header("DebugViewButton specific variables")]
    [SerializeField] private Camera debugCamera;
    public static bool isAvatarCameraActive = true;

    void Start()
    {
        uiLayerMask = LayerMask.GetMask("UI3D");
        _collider = GetComponent<Collider>();
        if (_collider == null)
        {
            Debug.LogWarning("[DebugViewButton] No collider found.");
            return;
        }

        _hoverScale = GetComponent<HoverScale>();

        if (handCursorObj != null)
            _handCursor = handCursorObj.GetComponent<HandCursorFollower>();

        if (_handCursor == null)
            Debug.LogWarning("[DebugViewButton] HandCursorFollower not found (tag: HandCursor).");

        if (handCursorLeftObj != null)
            _handCursorLeft = handCursorLeftObj.GetComponent<HandCursorFollower>();

        // Auto-find AvatarCamera if not assigned
        if (avatarCamera == null)
        {
            GameObject camObj = GameObject.Find("AvatarCamera");
            if (camObj != null)
                avatarCamera = camObj.GetComponent<Camera>();
            if (avatarCamera == null)
                avatarCamera = Camera.main;
        }

        // Auto-find DebugCamera if not assigned
        if (debugCamera == null)
        {
            GameObject debugCamObj = GameObject.Find("DebugCamera");
            if (debugCamObj != null)
                debugCamera = debugCamObj.GetComponent<Camera>();
        }

        if (debugCamera == null)
        {
            Debug.LogWarning("[DebugViewButton] DebugCamera not found!");
        }

        // Initialize with AvatarCamera active
        if (avatarCamera != null && debugCamera != null)
        {
            avatarCamera.enabled = true;
            debugCamera.enabled = false;
        }
    }

    void Update()
    {
        if (_collider == null) return;
        if (_handCursor == null && _handCursorLeft == null) return;

        bool isPointingRight = false;
        bool isPointingLeft = false;

        // Check right hand cursor
        if (_handCursor != null)
        {
            isPointingRight = CheckHandPointing(_handCursor);
        }

        // Check left hand cursor
        if (_handCursorLeft != null)
        {
            isPointingLeft = CheckHandPointing(_handCursorLeft);
        }

        bool isPointingAtThisButton = isPointingRight || isPointingLeft;

        // Hover grow
        if (_hoverScale != null)
            _hoverScale.SetHover(isPointingAtThisButton);

        // Click logic for right hand
        if (_handCursor != null)
        {
            bool isFistClosed = _handCursor.GetIsFistClosed();
            bool isFistInsideNow = isPointingRight && isFistClosed;

            if (isFistInsideNow && !_wasFistInside && Time.time >= _nextAllowedTime)
            {
                _nextAllowedTime = Time.time + cooldownSeconds;
                ToggleCamera();
            }
            _wasFistInside = isFistInsideNow;
        }

        // Click logic for left hand
        if (_handCursorLeft != null)
        {
            bool isFistClosedLeft = _handCursorLeft.GetIsFistClosed();
            bool isFistInsideNowLeft = isPointingLeft && isFistClosedLeft;

            if (isFistInsideNowLeft && !_wasFistInsideLeft && Time.time >= _nextAllowedTime)
            {
                _nextAllowedTime = Time.time + cooldownSeconds;
                ToggleCamera();
            }
            _wasFistInsideLeft = isFistInsideNowLeft;
        }
    }

    private bool CheckHandPointing(HandCursorFollower handCursor)
    {
        Vector3 origin = handCursor.transform.position;
        Vector3 direction = Vector3.zero;
        if (avatarCamera != null)
        {
            direction = (avatarCamera.transform.position - origin).normalized;
        }

        if (direction == Vector3.zero) return false;

        RaycastHit hit;
        bool didHit = Physics.Raycast(origin, direction, out hit, maxRayDistance, uiLayerMask);
        bool isPointing = didHit && hit.collider == _collider;

        Debug.DrawRay(origin, direction * maxRayDistance, isPointing ? Color.green : Color.red);
        return isPointing;
    }

    private void ToggleCamera()
    {
        if (avatarCamera == null || debugCamera == null)
        {
            Debug.LogWarning("[DebugViewButton] One or both cameras are not assigned!");
            return;
        }

        isAvatarCameraActive = !isAvatarCameraActive;

        avatarCamera.enabled = isAvatarCameraActive;
        debugCamera.enabled = !isAvatarCameraActive;

        Debug.Log($"Switched to {(isAvatarCameraActive ? "AvatarCamera" : "DebugCamera")}");
    }
}

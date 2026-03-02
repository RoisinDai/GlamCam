using UnityEngine;

/// <summary>
/// Button used to toggle the Kinect skeleton rendering in the BodySourceView.
/// Behaviour parallels ViewBaseAvatarButton: fist-selection, hover grow, etc.
/// </summary>
public class ViewSkeletonButton : MonoBehaviour
{
    [Header("Interaction")]
    public float cooldownSeconds = 0.5f;

    [Header("Raycast Settings")]
    public Camera avatarCamera;
    public float maxRayDistance = 30f;
    public LayerMask uiLayerMask;

    [SerializeField] private GameObject handCursorObj;
    [SerializeField] private GameObject handCursorLeftObj;

    private float _nextAllowedTime = 0f;
    private Collider _collider;

    private HandCursorFollower _handCursor;
    private HandCursorFollower _handCursorLeft;
    private HoverScale _hoverScale;

    private bool _wasFistInside = false;
    private bool _wasFistInsideLeft = false;

    [Header("Skeleton Control")]
    [Tooltip("Assign BodyView here (recommended). Works even if BodyView starts inactive.")]
    [SerializeField] private GameObject bodyViewObj;

    [Header("Canvas Control")]
    [Tooltip("Assign the Canvas GameObject to hide when skeleton is shown. Auto-finds 'Canvas' if not set.")]
    [SerializeField] private GameObject canvasObj;

    [Header("Mutual Exclusion")]
    [Tooltip("Reference to the ViewBaseAvatarButton so skeleton and base avatar don't show at the same time.")]
    [SerializeField] private ViewBaseAvatarButton baseAvatarButton;

    [Header("Clothed Avatar Mesh")]
    [Tooltip("Assign clothed_avatarMesh (child of clothed_avatar) here. Auto-found if not set.")]
    [SerializeField] private GameObject clothedAvatarMesh;

    // Backwards compat with your original private field name
    private GameObject _bodyViewObj;
    private GameObject _canvasObj;
    private GameObject _clothedAvatarMesh;

    void Start()
    {
        uiLayerMask = LayerMask.GetMask("UI3D");
        _collider = GetComponent<Collider>();
        if (_collider == null)
        {
            Debug.LogWarning("[ViewSkeletonButton] No collider found.");
            return;
        }

        _hoverScale = GetComponent<HoverScale>();

        if (handCursorObj != null)
            _handCursor = handCursorObj.GetComponent<HandCursorFollower>();

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

        // Prefer inspector assignment
        _bodyViewObj = bodyViewObj;

        // If not assigned, find BodyView even if inactive
        if (_bodyViewObj == null)
        {
            _bodyViewObj = FindInSceneIncludingInactive("BodyView");
        }

        if (_bodyViewObj == null)
        {
            Debug.LogWarning("[ViewSkeletonButton] BodyView not found. (Tip: assign it in Inspector.)");
        }
        else
        {
            Debug.Log($"[ViewSkeletonButton] Found BodyView (activeSelf={_bodyViewObj.activeSelf}).");
        }

        // Resolve Canvas reference
        _canvasObj = canvasObj;
        if (_canvasObj == null)
        {
            _canvasObj = FindInSceneIncludingInactive("Canvas");
        }

        if (_canvasObj == null)
        {
            Debug.LogWarning("[ViewSkeletonButton] Canvas not found. (Tip: assign it in Inspector.)");
        }
        else
        {
            Debug.Log($"[ViewSkeletonButton] Found Canvas (activeSelf={_canvasObj.activeSelf}).");
        }

        // Auto-find ViewBaseAvatarButton if not assigned
        if (baseAvatarButton == null)
            baseAvatarButton = FindObjectOfType<ViewBaseAvatarButton>();

        // Resolve clothed_avatarMesh
        _clothedAvatarMesh = clothedAvatarMesh;
        if (_clothedAvatarMesh == null)
        {
            GameObject clothedAvatar = GameObject.Find("clothed_avatar");
            if (clothedAvatar != null)
            {
                Transform t = clothedAvatar.transform.Find("clothed_avatarMesh");
                if (t != null) _clothedAvatarMesh = t.gameObject;
            }
        }

        if (_clothedAvatarMesh == null)
            Debug.LogWarning("[ViewSkeletonButton] clothed_avatarMesh not found. (Tip: assign it in Inspector.)");
        else
            Debug.Log($"[ViewSkeletonButton] Found clothed_avatarMesh (activeSelf={_clothedAvatarMesh.activeSelf}).");
    }

    void Update()
    {
        if (_collider == null) return;
        if (_handCursor == null && _handCursorLeft == null) return;

        bool isPointingRight = false;
        bool isPointingLeft = false;

        if (_handCursor != null)
            isPointingRight = CheckHandPointing(_handCursor);

        if (_handCursorLeft != null)
            isPointingLeft = CheckHandPointing(_handCursorLeft);

        bool isPointingAtThisButton = isPointingRight || isPointingLeft;
        if (_hoverScale != null)
            _hoverScale.SetHover(isPointingAtThisButton);

        if (_handCursor != null)
        {
            bool isFistClosed = _handCursor.GetIsFistClosed();
            bool isFistInsideNow = isPointingRight && isFistClosed;

            if (isFistInsideNow && !_wasFistInside && Time.time >= _nextAllowedTime)
            {
                _nextAllowedTime = Time.time + cooldownSeconds;
                ToggleSkeleton();
            }
            _wasFistInside = isFistInsideNow;
        }

        if (_handCursorLeft != null)
        {
            bool isFistClosedLeft = _handCursorLeft.GetIsFistClosed();
            bool isFistInsideNowLeft = isPointingLeft && isFistClosedLeft;

            if (isFistInsideNowLeft && !_wasFistInsideLeft && Time.time >= _nextAllowedTime)
            {
                _nextAllowedTime = Time.time + cooldownSeconds;
                ToggleSkeleton();
            }
            _wasFistInsideLeft = isFistInsideNowLeft;
        }
    }

    private bool CheckHandPointing(HandCursorFollower handCursor)
    {
        Vector3 origin = handCursor.transform.position;

        Vector3 direction = Vector3.zero;
        if (avatarCamera != null)
            direction = (avatarCamera.transform.position - origin).normalized;

        if (direction == Vector3.zero) return false;

        RaycastHit hit;
        bool didHit = Physics.Raycast(origin, direction, out hit, maxRayDistance, uiLayerMask);
        bool isPointing = didHit && hit.collider == _collider;

        Debug.DrawRay(origin, direction * maxRayDistance, isPointing ? Color.green : Color.red);
        return isPointing;
    }

    private void ToggleSkeleton()
    {
        // Re-try resolving if needed (covers spawned-late or missing in Start)
        if (_bodyViewObj == null)
        {
            _bodyViewObj = bodyViewObj != null ? bodyViewObj : FindInSceneIncludingInactive("BodyView");
        }

        if (_bodyViewObj == null)
        {
            Debug.LogWarning("[ViewSkeletonButton] BodyView still not found; cannot toggle.");
            return;
        }

        bool willBeActive = !_bodyViewObj.activeSelf;

        // If we're about to turn skeleton ON, reset the base avatar first
        if (willBeActive && baseAvatarButton != null)
            baseAvatarButton.ResetToDefault();

        _bodyViewObj.SetActive(willBeActive);
        Debug.Log($"[ViewSkeletonButton] BodyView active? {_bodyViewObj.activeSelf}");

        // Hide clothed_avatarMesh when skeleton is shown, restore when skeleton is hidden
        if (_clothedAvatarMesh != null)
        {
            _clothedAvatarMesh.SetActive(!willBeActive);
            Debug.Log($"[ViewSkeletonButton] clothed_avatarMesh active? {_clothedAvatarMesh.activeSelf}");
        }

        // Hide Canvas (video feed) when skeleton is shown, restore when skeleton is hidden
        if (_canvasObj == null)
            _canvasObj = canvasObj != null ? canvasObj : FindInSceneIncludingInactive("Canvas");

        if (_canvasObj != null)
        {
            _canvasObj.SetActive(!willBeActive);
            Debug.Log($"[ViewSkeletonButton] Canvas active? {_canvasObj.activeSelf}");
        }
    }

    /// <summary>
    /// Resets skeleton to off and restores canvas. Called by other buttons for mutual exclusion.
    /// </summary>
    public void ResetToDefault()
    {
        if (_bodyViewObj == null)
            _bodyViewObj = bodyViewObj != null ? bodyViewObj : FindInSceneIncludingInactive("BodyView");

        if (_bodyViewObj != null && _bodyViewObj.activeSelf)
        {
            _bodyViewObj.SetActive(false);
            Debug.Log("[ViewSkeletonButton] ResetToDefault: BodyView deactivated.");
        }

        // Restore clothed_avatarMesh
        if (_clothedAvatarMesh != null && !_clothedAvatarMesh.activeSelf)
        {
            _clothedAvatarMesh.SetActive(true);
            Debug.Log("[ViewSkeletonButton] ResetToDefault: clothed_avatarMesh restored.");
        }

        // Restore canvas
        if (_canvasObj == null)
            _canvasObj = canvasObj != null ? canvasObj : FindInSceneIncludingInactive("Canvas");

        if (_canvasObj != null)
            _canvasObj.SetActive(true);
    }

    /// <summary>
    /// Finds a GameObject by name in the loaded scenes, INCLUDING inactive objects.
    /// </summary>
    private static GameObject FindInSceneIncludingInactive(string targetName)
    {
#if UNITY_2023_1_OR_NEWER
        // Includes inactive objects in loaded scenes
        Transform[] all = GameObject.FindObjectsByType<Transform>(FindObjectsInactive.Include, FindObjectsSortMode.None);
        foreach (var t in all)
        {
            if (t.name == targetName)
                return t.gameObject;
        }
        return null;
#else
        // Works on older Unity versions; includes inactive objects.
        // Filter out non-scene objects (like assets) by requiring a valid scene.
        Transform[] all = Resources.FindObjectsOfTypeAll<Transform>();
        foreach (var t in all)
        {
            if (t.name != targetName) continue;
            if (!t.gameObject.scene.IsValid()) continue; // ignore prefab assets
            return t.gameObject;
        }
        return null;
#endif
    }
}
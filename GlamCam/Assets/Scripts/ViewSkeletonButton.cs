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

    // Backwards compat with your original private field name
    private GameObject _bodyViewObj;

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

        _bodyViewObj.SetActive(!_bodyViewObj.activeSelf);
        Debug.Log($"[ViewSkeletonButton] BodyView active? {_bodyViewObj.activeSelf}");
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
using UnityEngine;

public class ViewBaseAvatarButton : MonoBehaviour
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
    
    [Header("ViewBaseAvatarButton specific variables")]
    private bool isInvisibleMaskActive = true;
    [SerializeField] private GameObject baseAvatarMesh;
    private Renderer baseAvatarRenderer;

    [SerializeField] private Material invisibleMaskMaterial;
    [SerializeField] private Material greenMaterial;
    


    void Start()
    {
        uiLayerMask = LayerMask.GetMask("UI3D");
        _collider = GetComponent<Collider>();
        if (_collider == null)
        {
            Debug.LogWarning("[ViewBaseAvatarButton] No collider found.");
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

        // Find the renderer for the base avatar if not assigned
        if (baseAvatarMesh != null)
        {
            baseAvatarRenderer = baseAvatarMesh?.GetComponent<Renderer>();
            if (baseAvatarRenderer == null)
            {
                Debug.LogWarning("[ViewBaseAvatarButton] No Renderer found on this GameObject!");
            }
        } else
        {
            Debug.LogWarning("[ViewBaseAvatarButton] Base Avatar Mesh not assigned!");
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
                ToggleBaseAvatarMaterial();
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
                ToggleBaseAvatarMaterial();
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

    private void ToggleBaseAvatarMaterial()
    {
        if (baseAvatarRenderer == null || invisibleMaskMaterial == null || greenMaterial == null)
        {
            Debug.LogWarning("[ViewBaseAvatarButton] Renderer or materials not properly set up!");
            return;
        }

        isInvisibleMaskActive = !isInvisibleMaskActive;

        baseAvatarRenderer.material = isInvisibleMaskActive ? invisibleMaskMaterial : greenMaterial;

        Debug.Log($"Switched to {(isInvisibleMaskActive ? "InvisibleMask" : "Green")} material");
    }
}
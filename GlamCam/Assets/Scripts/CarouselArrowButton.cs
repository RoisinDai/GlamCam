using UnityEngine;

public class CarouselArrowButton : MonoBehaviour
{
    public CarouselSelector selector;   // Drag TopSelector (with CarouselSelector) here
    public bool isNext = true;          // RightArrow=true, LeftArrow=false
    public float cooldownSeconds = 0.35f;

    [Header("Raycast Settings")]
    public Camera avatarCamera;
    public float maxRayDistance = 30f;
    public LayerMask uiLayerMask = LayerMask.GetMask("UI3D");

    private float _nextAllowedTime = 0f;
    private Collider _collider;
    private HandCursorFollower _handCursor;
    private bool _wasFistInside = false;
    private HoverScale _hoverScale;

    void Start()
    {
        _collider = GetComponent<Collider>();
        if (_collider == null)
        {
            Debug.LogWarning("[CarouselArrowButton] No collider found on this GameObject.");
        }

        GameObject handCursorObj = GameObject.FindWithTag("HandCursor");
        if (handCursorObj != null)
        {
            _handCursor = handCursorObj.GetComponent<HandCursorFollower>();
        }

        if (_handCursor == null)
        {
            Debug.LogWarning("[CarouselArrowButton] HandCursorFollower not found.");
        }

        _hoverScale = GetComponent<HoverScale>();

        // Auto-find AvatarCamera if not assigned
        if (avatarCamera == null)
        {
            GameObject camObj = GameObject.Find("AvatarCamera");
            if (camObj != null)
                avatarCamera = camObj.GetComponent<Camera>();
            if (avatarCamera == null)
                avatarCamera = Camera.main;
        }
    }

    void Update()
    {
        if (_handCursor == null || _collider == null || selector == null) return;

        // Ray origin = hand cursor position
        Vector3 origin = _handCursor.transform.position;

        // Ray direction = from hand toward camera/UI
        Vector3 direction = Vector3.zero;
        if (avatarCamera != null)
        {
            direction = (avatarCamera.transform.position - origin).normalized;
        }

        // Cast the ray
        bool isPointingAtThisButton = false;
        if (direction != Vector3.zero)
        {
            RaycastHit hit;
            bool didHit = Physics.Raycast(origin, direction, out hit, maxRayDistance, uiLayerMask);

            // Check if this ray hit OUR button's collider
            isPointingAtThisButton = didHit && hit.collider == _collider;

            // Debug: visualize the ray in Scene view
            Debug.DrawRay(origin, direction * maxRayDistance, isPointingAtThisButton ? Color.green : Color.red);
        }

        // Hover scale feedback
        if (_hoverScale != null)
            _hoverScale.SetHover(isPointingAtThisButton);

        // Click logic (fist while pointing at button)
        bool isFistClosed = _handCursor.GetIsFistClosed();
        bool isFistInsideNow = isPointingAtThisButton && isFistClosed;

        if (isFistInsideNow && !_wasFistInside)
        {
            if (Time.time >= _nextAllowedTime)
            {
                _nextAllowedTime = Time.time + cooldownSeconds;
                if (isNext) selector.Next();
                else selector.Prev();
            }
        }

        _wasFistInside = isFistInsideNow;
    }
}
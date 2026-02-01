using UnityEngine;

public class ClearAllClothingButton : MonoBehaviour
{
    [Header("Managers to clear")]
    public CarouselClothingManager tops;
    public CarouselClothingManager bottoms;
    public CarouselClothingManager dresses;
    public CarouselClothingManager fullbodies;
    public CarouselClothingManager hats;
    public CarouselClothingManager accessories;

    [Header("Interaction")]
    public float cooldownSeconds = 0.5f;

    [Header("Raycast Settings")]
    public Camera avatarCamera;
    public float maxRayDistance = 30f;
    public LayerMask uiLayerMask = LayerMask.GetMask("UI3D");

    private float _nextAllowedTime = 0f;
    private Collider _collider;
    private HandCursorFollower _handCursor;
    private HandCursorFollower _handCursorLeft;
    private HoverScale _hoverScale;
    private bool _wasFistInside = false;
    private bool _wasFistInsideLeft = false;

    void Start()
    {
        _collider = GetComponent<Collider>();
        if (_collider == null)
        {
            Debug.LogWarning("[ClearAllClothingButton] No collider found.");
            return;
        }

        _hoverScale = GetComponent<HoverScale>();

        var handObj = GameObject.FindWithTag("HandCursor");
        if (handObj != null)
            _handCursor = handObj.GetComponent<HandCursorFollower>();

        if (_handCursor == null)
            Debug.LogWarning("[ClearAllClothingButton] HandCursorFollower not found (tag: HandCursor).");

        var handObjLeft = GameObject.FindWithTag("HandCursorLeft");
        if (handObjLeft != null)
            _handCursorLeft = handObjLeft.GetComponent<HandCursorFollower>();

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
                ClearAll();
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
                ClearAll();
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

    private void ClearAll()
    {
        tops?.ClearCategoryAndResetUI();
        bottoms?.ClearCategoryAndResetUI();
        dresses?.ClearCategoryAndResetUI();
        fullbodies?.ClearCategoryAndResetUI();
        hats?.ClearCategoryAndResetUI();
        accessories?.ClearCategoryAndResetUI();

        Debug.Log("[ClearAllClothingButton] Cleared all clothing and reset UI to None.");
    }
}
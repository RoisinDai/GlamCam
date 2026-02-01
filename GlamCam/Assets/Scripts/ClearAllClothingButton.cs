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
    private HoverScale _hoverScale;
    private bool _wasFistInside = false;

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
        if (_handCursor == null || _collider == null) return;

        // Raycast from hand toward camera
        Vector3 origin = _handCursor.transform.position;
        Vector3 direction = Vector3.zero;
        if (avatarCamera != null)
        {
            direction = (avatarCamera.transform.position - origin).normalized;
        }

        bool isPointingAtThisButton = false;
        if (direction != Vector3.zero)
        {
            RaycastHit hit;
            bool didHit = Physics.Raycast(origin, direction, out hit, maxRayDistance, uiLayerMask);
            isPointingAtThisButton = didHit && hit.collider == _collider;

            Debug.DrawRay(origin, direction * maxRayDistance, isPointingAtThisButton ? Color.green : Color.red);
        }

        // Hover grow
        if (_hoverScale != null)
            _hoverScale.SetHover(isPointingAtThisButton);

        // Fist press detection
        bool isFistClosed = _handCursor.GetIsFistClosed();
        bool isFistInsideNow = isPointingAtThisButton && isFistClosed;

        // Trigger once per press
        if (isFistInsideNow && !_wasFistInside && Time.time >= _nextAllowedTime)
        {
            _nextAllowedTime = Time.time + cooldownSeconds;
            ClearAll();
        }

        _wasFistInside = isFistInsideNow;
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
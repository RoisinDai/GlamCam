using UnityEngine;

public class ResetButton : MonoBehaviour
{
    [Header("References")]
    public InitializationStateMachine stateMachine;

    [Header("Avatar")]
    public string avatarObjectName = "clothed_avatar";

    [Header("Input")]
    public float cooldownSeconds = 0.5f;

    [Header("Raycast Settings")]
    public Camera avatarCamera;
    public float maxRayDistance = 30f;
    public LayerMask uiLayerMask = -1;

    [Header("View Buttons (for reset)")]
    [SerializeField] private ViewSkeletonButton skeletonButton;
    [SerializeField] private ViewBaseAvatarButton baseAvatarButton;

    private Collider _collider;
    private HandCursorFollower _handCursor;
    private HandCursorFollower _handCursorLeft;
    private bool _wasFistInside = false;
    private bool _wasFistInsideLeft = false;
    private float _nextAllowedTime = 0f;
    private HoverScale _hoverScale;

    void Start()
    {
        _collider = GetComponent<Collider>();
        if (_collider == null)
        {
            Debug.LogWarning("[ResetButton] No collider found on this GameObject.");
        }

        GameObject handCursorObj = GameObject.FindWithTag("HandCursor");
        if (handCursorObj != null)
        {
            _handCursor = handCursorObj.GetComponent<HandCursorFollower>();
        }

        if (_handCursor == null)
        {
            Debug.LogWarning("[ResetButton] HandCursorFollower not found. Tag the hand cursor with 'HandCursor'.");
        }

        GameObject handCursorLeftObj = GameObject.FindWithTag("HandCursorLeft");
        if (handCursorLeftObj != null)
        {
            _handCursorLeft = handCursorLeftObj.GetComponent<HandCursorFollower>();
        }

        _hoverScale = GetComponent<HoverScale>();

        // Auto-find view buttons if not assigned
        if (skeletonButton == null)
            skeletonButton = FindObjectOfType<ViewSkeletonButton>();
        if (baseAvatarButton == null)
            baseAvatarButton = FindObjectOfType<ViewBaseAvatarButton>();

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

        // Hover scale feedback
        if (_hoverScale != null)
            _hoverScale.SetHover(isPointingAtThisButton);

        // Click logic for right hand
        if (_handCursor != null)
        {
            bool isFistClosed = _handCursor.GetIsFistClosed();
            bool isFistInsideNow = isPointingRight && isFistClosed;

            if (isFistInsideNow && !_wasFistInside)
            {
                if (Time.time >= _nextAllowedTime)
                {
                    _nextAllowedTime = Time.time + cooldownSeconds;
                    OnResetPressed();
                }
            }
            _wasFistInside = isFistInsideNow;
        }

        // Click logic for left hand
        if (_handCursorLeft != null)
        {
            bool isFistClosedLeft = _handCursorLeft.GetIsFistClosed();
            bool isFistInsideNowLeft = isPointingLeft && isFistClosedLeft;

            if (isFistInsideNowLeft && !_wasFistInsideLeft)
            {
                if (Time.time >= _nextAllowedTime)
                {
                    _nextAllowedTime = Time.time + cooldownSeconds;
                    OnResetPressed();
                }
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

    public void OnResetPressed()
    {
        Debug.Log("[ResetButton] Reset pressed!");

        // 0) Reset view buttons (skeleton / base avatar) so Canvas comes back
        if (skeletonButton != null)
            skeletonButton.ResetToDefault();
        if (baseAvatarButton != null)
            baseAvatarButton.ResetToDefault();

        // 1) Reset UI / phase state
        if (stateMachine != null)
        {
            stateMachine.ResetToInit();
        }
        else
        {
            Debug.LogWarning("[ResetButton] InitializationStateMachine not assigned.");
        }

        // 2) Reset avatar
        GameObject avatarRoot = GameObject.Find(avatarObjectName);
        if (avatarRoot == null)
        {
            Debug.LogWarning($"[ResetButton] '{avatarObjectName}' not found.");
            return;
        }

        AvatarController avatarController =
            avatarRoot.GetComponentInChildren<AvatarController>();

        if (avatarController != null)
        {
            avatarController.Restart();
        }
        else
        {
            Debug.LogWarning("[ResetButton] AvatarController not found on clothed_avatar.");
        }
    }
}
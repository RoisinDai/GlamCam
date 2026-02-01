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

    private Collider _collider;
    private HandCursorFollower _handCursor;
    private bool _wasFistInside = false;
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

        // Hover scale feedback
        if (_hoverScale != null)
            _hoverScale.SetHover(isPointingAtThisButton);

        // Click logic (fist)
        bool isFistClosed = _handCursor.GetIsFistClosed();
        bool isFistInsideNow = isPointingAtThisButton && isFistClosed;

        // Trigger on fist-close (rising edge)
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

    public void OnResetPressed()
    {
        Debug.Log("[ResetButton] Reset pressed!");

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
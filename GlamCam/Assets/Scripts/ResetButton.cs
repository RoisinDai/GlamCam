using UnityEngine;

public class ResetButton : MonoBehaviour
{
    [Header("References")]
    public InitializationStateMachine stateMachine;

    [Header("Avatar")]
    public string avatarObjectName = "clothed_avatar";

    [Header("Input")]
    public float cooldownSeconds = 0.5f;

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
    }

    void Update()
    {
        if (_handCursor == null || _collider == null) return;

        Vector3 handPos = _handCursor.transform.position;

        // Check if cursor is inside collider
        Vector3 closest = _collider.ClosestPoint(handPos);
        float dist = Vector3.Distance(closest, handPos);
        bool isCursorInside = dist <= 0.02f;

        // Hover scale feedback
        if (_hoverScale != null)
            _hoverScale.SetHover(isCursorInside);

        // Click logic (fist)
        bool isFistClosed = _handCursor.GetIsFistClosed();
        bool isFistInsideNow = isCursorInside && isFistClosed;

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

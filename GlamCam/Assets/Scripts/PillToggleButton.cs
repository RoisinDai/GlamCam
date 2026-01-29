using UnityEngine;

public class PillToggleButton : MonoBehaviour
{
    public SelectorVisibilityController controller;
    public bool isClothesButton = true;
    public float cooldownSeconds = 0.5f;

    private float _nextAllowedTime;
    private Collider _collider;
    private HandCursorFollower _handCursor;
    private bool _wasFistInside = false;

    void Start()
    {
        _collider = GetComponent<Collider>();
        if (_collider == null)
        {
            Debug.LogWarning("[PillToggleButton] No collider found on this GameObject.");
        }

        GameObject handCursorObj = GameObject.FindWithTag("HandCursor");
        if (handCursorObj != null)
        {
            _handCursor = handCursorObj.GetComponent<HandCursorFollower>();
        }

        if (_handCursor == null)
        {
            Debug.LogWarning("[PillToggleButton] HandCursorFollower not found.");
        }
    }

    void Update()
    {
        if (_handCursor == null || _collider == null || controller == null) return;

        // Check if cursor is inside this button's collider
        Vector3 handPos = _handCursor.transform.position;
        bool isCursorInside = _collider.bounds.Contains(handPos);

        // Check if hand is closed (fist)
        bool isFistClosed = _handCursor.GetIsFistClosed();

        bool isFistInsideNow = isCursorInside && isFistClosed;

        // Fire selection on transition from not-fist to fist (while inside)
        if (isFistInsideNow && !_wasFistInside)
        {
            if (Time.time >= _nextAllowedTime)
            {
                _nextAllowedTime = Time.time + cooldownSeconds;

                if (isClothesButton)
                    controller.ShowClothes();
                else
                    controller.ShowAccessories();
            }
        }

        _wasFistInside = isFistInsideNow;
    }
}
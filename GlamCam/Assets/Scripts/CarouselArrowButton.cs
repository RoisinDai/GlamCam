using UnityEngine;

public class CarouselArrowButton : MonoBehaviour
{
    public CarouselSelector selector;   // Drag TopSelector (with CarouselSelector) here
    public bool isNext = true;          // RightArrow=true, LeftArrow=false
    public float cooldownSeconds = 0.35f;

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
    }

    void Update()
    {
        if (_handCursor == null || _collider == null || selector == null) return;

        Vector3 handPos = _handCursor.transform.position;

        // Reliable "inside" check
        Vector3 closest = _collider.ClosestPoint(handPos);
        float dist = Vector3.Distance(closest, handPos);
        bool isCursorInside = dist <= 0.02f; // <-- tune this

        // Hover scale
        if (_hoverScale != null)
            _hoverScale.SetHover(isCursorInside);

        // Click logic (fist)
        bool isFistClosed = _handCursor.GetIsFistClosed();
        bool isFistInsideNow = isCursorInside && isFistClosed;

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
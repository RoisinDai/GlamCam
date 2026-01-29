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
    public float pressDistance = 0.04f;
    public float cooldownSeconds = 0.5f;

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
    }

    void Update()
    {
        if (_handCursor == null || _collider == null) return;

        Vector3 handPos = _handCursor.transform.position;

        // Reliable inside check for sphere/box/etc.
        Vector3 closest = _collider.ClosestPoint(handPos);
        float dist = Vector3.Distance(closest, handPos);
        bool isInside = dist <= pressDistance;

        // Hover grow
        if (_hoverScale != null)
            _hoverScale.SetHover(isInside);

        // Fist press detection
        bool isFistClosed = _handCursor.GetIsFistClosed();
        bool isFistInsideNow = isInside && isFistClosed;

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
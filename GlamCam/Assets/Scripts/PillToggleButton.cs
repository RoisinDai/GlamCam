using UnityEngine;

public class PillToggleButton : MonoBehaviour
{
    public SelectorVisibilityController controller;
    public bool isClothesButton = true;
    public float cooldownSeconds = 0.5f;

    [Header("Selection Visuals")]
    public MeshRenderer clothesIconRenderer;
    public Material clothesNormalMat;
    public Material clothesGlowMat;

    public MeshRenderer accessoriesIconRenderer;
    public Material accessoriesNormalMat;
    public Material accessoriesGlowMat;

    [Header("Default Selected")]
    public bool defaultClothesSelected = true;

    [Header("Raycast Settings")]
    public Camera avatarCamera;
    public float maxRayDistance = 30f;
    public LayerMask uiLayerMask;

    private float _nextAllowedTime;
    private Collider _collider;
    private HandCursorFollower _handCursor;
    private HandCursorFollower _handCursorLeft;
    private bool _wasFistInside = false;
    private bool _wasFistInsideLeft = false;
    private HoverScale _hoverScale;

    void Start()
    {
        uiLayerMask = LayerMask.GetMask("UI3D");
        _collider = GetComponent<Collider>();
        if (_collider == null)
            Debug.LogWarning("[PillToggleButton] No collider found on this GameObject.");

        GameObject handCursorObj = GameObject.FindWithTag("HandCursor");
        if (handCursorObj != null)
            _handCursor = handCursorObj.GetComponent<HandCursorFollower>();

        if (_handCursor == null)
            Debug.LogWarning("[PillToggleButton] HandCursorFollower not found.");

        GameObject handCursorLeftObj = GameObject.FindWithTag("HandCursorLeft");
        if (handCursorLeftObj != null)
            _handCursorLeft = handCursorLeftObj.GetComponent<HandCursorFollower>();

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

        // Apply default selection visuals ONCE
        ApplySelectionVisuals(defaultClothesSelected);
    }

    void Update()
    {
        if (_collider == null || controller == null) return;
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
                    TriggerButton();
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
                    TriggerButton();
                }
            }
            _wasFistInsideLeft = isFistInsideNowLeft;
        }
    }

    private void TriggerButton()
    {
        if (isClothesButton)
        {
            controller.ShowClothes();
            ApplySelectionVisuals(true);
        }
        else
        {
            controller.ShowAccessories();
            ApplySelectionVisuals(false);
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

    private void ApplySelectionVisuals(bool clothesSelected)
    {
        if (clothesIconRenderer != null && clothesNormalMat != null && clothesGlowMat != null)
            clothesIconRenderer.material = clothesSelected ? clothesGlowMat : clothesNormalMat;

        if (accessoriesIconRenderer != null && accessoriesNormalMat != null && accessoriesGlowMat != null)
            accessoriesIconRenderer.material = clothesSelected ? accessoriesNormalMat : accessoriesGlowMat;
    }

    /// <summary>
    /// Resets the pill to show Clothes selected. Called on reset.
    /// </summary>
    public void ResetToClothes()
    {
        ApplySelectionVisuals(true);
        if (controller != null)
            controller.ShowClothes();
    }
}
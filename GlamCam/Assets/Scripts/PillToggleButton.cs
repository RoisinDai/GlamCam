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
    public LayerMask uiLayerMask = LayerMask.GetMask("UI3D");

    private float _nextAllowedTime;
    private Collider _collider;
    private HandCursorFollower _handCursor;
    private bool _wasFistInside = false;
    private HoverScale _hoverScale;

    void Start()
    {
        _collider = GetComponent<Collider>();
        if (_collider == null)
            Debug.LogWarning("[PillToggleButton] No collider found on this GameObject.");

        GameObject handCursorObj = GameObject.FindWithTag("HandCursor");
        if (handCursorObj != null)
            _handCursor = handCursorObj.GetComponent<HandCursorFollower>();

        if (_handCursor == null)
            Debug.LogWarning("[PillToggleButton] HandCursorFollower not found.");

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
        if (_handCursor == null || _collider == null || controller == null) return;

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

        bool isFistClosed = _handCursor.GetIsFistClosed();
        bool isFistInsideNow = isPointingAtThisButton && isFistClosed;

        if (isFistInsideNow && !_wasFistInside)
        {
            if (Time.time >= _nextAllowedTime)
            {
                _nextAllowedTime = Time.time + cooldownSeconds;

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
        }

        _wasFistInside = isFistInsideNow;
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
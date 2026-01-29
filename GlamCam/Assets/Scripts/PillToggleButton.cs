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

    private float _nextAllowedTime;
    private Collider _collider;
    private HandCursorFollower _handCursor;
    private bool _wasFistInside = false;

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

        // Apply default selection visuals ONCE
        ApplySelectionVisuals(defaultClothesSelected);
    }

    void Update()
    {
        if (_handCursor == null || _collider == null || controller == null) return;

        Vector3 handPos = _handCursor.transform.position;
        bool isCursorInside = _collider.bounds.Contains(handPos);

        bool isFistClosed = _handCursor.GetIsFistClosed();
        bool isFistInsideNow = isCursorInside && isFistClosed;

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
}
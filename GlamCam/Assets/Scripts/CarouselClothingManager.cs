using System.Collections.Generic;
using UnityEngine;

public class CarouselClothingManager : MonoBehaviour
{
    [Header("Carousel")]
    public CarouselSelector carousel;

    [Header("Avatar Root")]
    public GameObject clothedAvatar;

    [Header("Category")]
    // [Tooltip("Must match the child GameObject name under clothed_avatar")]
    public string categoryName = "Tops";

    public enum ClothingCategoryType
    {
        Mixable,
        Exclusive   // for future use (e.g. Dresses / Skirts)
    }

    [Header("Category Behavior")]
    public ClothingCategoryType categoryType = ClothingCategoryType.Mixable;

    // Internal
    private readonly Dictionary<string, GameObject> garmentsByName = new();
    private readonly List<GameObject> categoryGarments = new();

    void Start()
    {
        // Auto-find clothed_avatar if not assigned
        if (clothedAvatar == null)
        {
            clothedAvatar = GameObject.Find("clothed_avatar");
            if (clothedAvatar == null)
            {
                Debug.LogError("[CarouselClothingManager] clothed_avatar not found in scene.");
                return;
            }
        }

        BuildCategoryLookup();

        if (carousel != null)
        {
            carousel.SelectionChanged += OnCarouselSelectionChanged;

            // Apply initial selection immediately
            OnCarouselSelectionChanged(
                carousel.GetIndex(),
                carousel.GetCurrentItemName()
            );
        }
        else
        {
            Debug.LogWarning("[CarouselClothingManager] CarouselSelector not assigned.");
        }
    }

    private void BuildCategoryLookup()
    {
        garmentsByName.Clear();
        categoryGarments.Clear();

        if (clothedAvatar == null) return;

        Transform root;

        // If no category folder exists yet, use clothed_avatar directly
        if (string.IsNullOrEmpty(categoryName))
        {
            root = clothedAvatar.transform;
            Debug.Log("[CarouselClothingManager] No categoryName set. Using clothed_avatar root.");
        }
        else
        {
            root = clothedAvatar.transform.Find(categoryName);
            if (root == null)
            {
                Debug.LogWarning(
                    $"[CarouselClothingManager] Category '{categoryName}' not found. Falling back to clothed_avatar root."
                );
                root = clothedAvatar.transform;
            }
        }

        foreach (Transform child in root)
        {
            if (child == null) continue;

            garmentsByName[child.name] = child.gameObject;
            categoryGarments.Add(child.gameObject);
        }

        Debug.Log(
            $"[CarouselClothingManager] Loaded {categoryGarments.Count} garments from '{root.name}'."
        );
    }

    private void OnCarouselSelectionChanged(int index, string itemName)
    {
        if (string.IsNullOrEmpty(itemName))
            return;

        // Always turn OFF everything in this managed set first
        foreach (var g in categoryGarments)
        {
            if (g != null) g.SetActive(false);
        }

        // If user selected "None" enter deselected state
        if (string.Equals(itemName, "None", System.StringComparison.OrdinalIgnoreCase))
        {
            Debug.Log($"[CarouselClothingManager] [{categoryName}] Deselected (None).");
            return;
        }

        // Otherwise turn ON the selected garment if it exists
        if (garmentsByName.TryGetValue(itemName, out var selectedGarment) && selectedGarment != null)
        {
            selectedGarment.SetActive(true);
            Debug.Log($"[CarouselClothingManager] [{categoryName}] Activated '{itemName}' (index {index}).");
        }
        else
        {
            Debug.LogWarning($"[CarouselClothingManager] [{categoryName}] Garment '{itemName}' not found.");
        }
    }

    void OnDestroy()
    {
        if (carousel != null)
        {
            carousel.SelectionChanged -= OnCarouselSelectionChanged;
        }
    }
}
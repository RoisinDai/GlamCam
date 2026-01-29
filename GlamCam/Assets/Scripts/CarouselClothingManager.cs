using System.Collections.Generic;
using UnityEngine;

public class CarouselClothingManager : MonoBehaviour
{
    [Header("Carousel")]
    public CarouselSelector carousel;

    [Header("Avatar Root")]
    public GameObject clothedAvatar;

    [Header("Category (Logs Only)")]
    public string categoryName = "";

    public enum ClothingCategoryType
    {
        Mixable,
        Exclusive   // for future use
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

        var loadedNames = carousel.GetLoadedItemNames(includeNone: false);
        if (loadedNames.Count == 0)
        {
            Debug.LogWarning($"[CarouselClothingManager] [{GetCategoryLabel()}] Carousel has 0 loaded names. Check resourcesFolder or execution order.");
        }

        garmentsByName.Clear();
        categoryGarments.Clear();

        if (clothedAvatar == null) return;

        if (carousel == null)
        {
            Debug.LogError("[CarouselClothingManager] CarouselSelector missing; cannot build lookup.");
            return;
        }

        // Only manage items that this carousel actually loads
        var wantedNames = new HashSet<string>(
            carousel.GetLoadedItemNames(includeNone: false),
            System.StringComparer.OrdinalIgnoreCase
        );

        Transform root = clothedAvatar.transform;
        System.Text.StringBuilder sb = new System.Text.StringBuilder();
        sb.AppendLine($"[CarouselClothingManager] [{GetCategoryLabel()}] Matching garments under '{root.name}':");

        foreach (Transform child in root)
        {
            if (child == null) continue;

            if (!wantedNames.Contains(child.name))
                continue;

            garmentsByName[child.name] = child.gameObject;
            categoryGarments.Add(child.gameObject);
            sb.AppendLine($"- {child.name}");
        }

        Debug.Log(sb.ToString());
        Debug.Log($"[CarouselClothingManager] [{GetCategoryLabel()}] Loaded {categoryGarments.Count} garments.");
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
            Debug.Log($"[CarouselClothingManager] [{GetCategoryLabel()}] Deselected (None).");
            ClothingRulesCoordinator.NotifyChanged(this, itemName);
            return;
        }

        // Otherwise turn ON the selected garment if it exists
        if (garmentsByName.TryGetValue(itemName, out var selectedGarment) && selectedGarment != null)
        {
            selectedGarment.SetActive(true);
            Debug.Log($"[CarouselClothingManager] [{GetCategoryLabel()}] Activated '{itemName}' (index {index}).");
        }
        else
        {
            Debug.LogWarning($"[CarouselClothingManager] [{GetCategoryLabel()}] Garment '{itemName}' not found.");
        }

        // enforce cross-category rules (mix/exclusive/overlay)
        ClothingRulesCoordinator.NotifyChanged(this, itemName);
    }

    void OnDestroy()
    {
        if (carousel != null)
        {
            carousel.SelectionChanged -= OnCarouselSelectionChanged;
        }
    }

    // used by rules coordinator
    public void DeactivateAll()
    {
        foreach (var g in categoryGarments)
        {
            if (g != null) g.SetActive(false);
        }
    }

    // optional helper
    public bool HasAnyActive()
    {
        foreach (var g in categoryGarments)
        {
            if (g != null && g.activeSelf) return true;
        }
        return false;
    }

    private string GetCategoryLabel()
    {
        if (!string.IsNullOrEmpty(categoryName)) return categoryName;
        if (carousel != null && !string.IsNullOrEmpty(carousel.resourcesFolder))
            return carousel.resourcesFolder;
        return "Unknown";
    }
}

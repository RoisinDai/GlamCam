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

    [Header("Mixing Rules")]
    public bool isBaseLayer = false;

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
    private static readonly List<CarouselClothingManager> Instances = new();

    void OnEnable()
    {
        if (!Instances.Contains(this)) Instances.Add(this);
    }

    void OnDisable()
    {
        Instances.Remove(this);
    }

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

        Transform root = clothedAvatar.transform;
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

        // Enforce mixing rules before applying selection
        if (!IsNone(itemName))
        {
            if (categoryType == ClothingCategoryType.Exclusive)
            {
                ClearBaseLayers();
            }
            else if (isBaseLayer)
            {
                ClearExclusiveLayers();
            }
        }

        // Always turn OFF everything in this managed set first
        foreach (var g in categoryGarments)
        {
            if (g != null) g.SetActive(false);
        }

        // If user selected "None" enter deselected state
        if (IsNone(itemName))
        {
            Debug.Log($"[CarouselClothingManager] [{GetCategoryLabel()}] Deselected (None).");
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
    }

    void OnDestroy()
    {
        if (carousel != null)
        {
            carousel.SelectionChanged -= OnCarouselSelectionChanged;
        }
    }

    private static bool IsNone(string itemName)
    {
        return string.Equals(itemName, "None", System.StringComparison.OrdinalIgnoreCase);
    }

    private void ClearBaseLayers()
    {
        foreach (var mgr in Instances)
        {
            if (mgr == null || mgr == this) continue;
            if (!mgr.isBaseLayer) continue;
            mgr.ResetToNone();
        }
    }

    private void ClearExclusiveLayers()
    {
        foreach (var mgr in Instances)
        {
            if (mgr == null || mgr == this) continue;
            if (mgr.categoryType != ClothingCategoryType.Exclusive) continue;
            mgr.ResetToNone();
        }
    }

    private void ResetToNone()
    {
        if (carousel == null) return;
        if (IsNone(carousel.GetCurrentItemName())) return;
        if (!carousel.SetItemByName("None"))
        {
            Debug.LogWarning($"[CarouselClothingManager] [{GetCategoryLabel()}] Unable to set to None.");
        }
    }

    private string GetCategoryLabel()
    {
        if (!string.IsNullOrEmpty(categoryName)) return categoryName;
        if (carousel != null && !string.IsNullOrEmpty(carousel.resourcesFolder))
            return carousel.resourcesFolder;
        return "Unknown";
    }
}
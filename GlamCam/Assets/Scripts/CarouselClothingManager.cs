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

    public enum LayerGroup
    {
        Base,       // Tops, Bottoms
        Exclusive,  // Dresses, Fullbodies
        Overlay     // Hats, Accessories
    }

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
        if (carousel == null)
        {
            Debug.LogError("[CarouselClothingManager] carousel not assigned.");
            return;
        }

        var allowed = new HashSet<string>(
            carousel.GetItemNames(includeNone: false),
            System.StringComparer.OrdinalIgnoreCase
        );

        foreach (Transform child in clothedAvatar.transform)
        {
            if (child == null) continue;

            if (allowed.Contains(child.name))
            {
                garmentsByName[child.name] = child.gameObject;
                categoryGarments.Add(child.gameObject);
            }
        }

        Debug.Log($"[CarouselClothingManager] [{GetCategoryLabel()}] Managing {categoryGarments.Count} garments.");
    }

    private void OnCarouselSelectionChanged(int index, string itemName)
    {
        if (string.IsNullOrEmpty(itemName))
            return;

        // Enforce mixing rules before applying selection
        if (!IsNone(itemName))
        {
            var group = GetGroup();
            if (group == LayerGroup.Exclusive)
            {
                ClearByGroup(LayerGroup.Base);
            }
            else if (group == LayerGroup.Base)
            {
                ClearByGroup(LayerGroup.Exclusive);
            }
            // Overlay clears nothing
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

    private void ClearByGroup(LayerGroup target)
    {
        foreach (var mgr in Instances)
        {
            if (mgr == null || mgr == this) continue;
            if (mgr.carousel == null) continue;
            if (mgr.GetGroup() == target)
            {
                mgr.ResetToNone();
            }
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

    private LayerGroup GetGroup()
    {
        string folder = carousel != null ? carousel.resourcesFolder : "";
        if (string.IsNullOrEmpty(folder)) return LayerGroup.Overlay;

        if (folder.Equals("Tops", System.StringComparison.OrdinalIgnoreCase)) return LayerGroup.Base;
        if (folder.Equals("Bottoms", System.StringComparison.OrdinalIgnoreCase)) return LayerGroup.Base;

        if (folder.Equals("Dresses", System.StringComparison.OrdinalIgnoreCase)) return LayerGroup.Exclusive;
        if (folder.Equals("Fullbodies", System.StringComparison.OrdinalIgnoreCase)) return LayerGroup.Exclusive;

        if (folder.Equals("Hats", System.StringComparison.OrdinalIgnoreCase)) return LayerGroup.Overlay;
        if (folder.Equals("Accessories", System.StringComparison.OrdinalIgnoreCase)) return LayerGroup.Overlay;

        return LayerGroup.Overlay;
    }
}
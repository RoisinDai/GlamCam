using System.Collections.Generic;
using UnityEngine;

public class ClothingRulesCoordinator : MonoBehaviour
{
    public CarouselClothingManager tops;
    public CarouselClothingManager bottoms;
    public CarouselClothingManager dresses;
    public CarouselClothingManager fullbodies;
    public CarouselClothingManager hats;
    public CarouselClothingManager accessories;

    [Header("Pill Nav (for reset)")]
    public PillToggleButton clothesPillButton;
    public SelectorVisibilityController selectorVisibility;

    private static ClothingRulesCoordinator _instance;

    // used for temporarily saving current selections when a warning appears
    private Dictionary<CarouselClothingManager, string> _savedSelections;

    void Awake()
    {
        _instance = this;
    }

    public static void NotifyChanged(CarouselClothingManager source, string itemName)
    {
        if (_instance == null || source == null) return;
        _instance.ApplyRules(source, itemName);
    }

    private void ApplyRules(CarouselClothingManager source, string itemName)
    {
        if (string.IsNullOrEmpty(itemName)) return;

        bool isNone = string.Equals(itemName, "None", System.StringComparison.OrdinalIgnoreCase);

        bool sourceIsDress = (source == dresses);
        bool sourceIsFullbody = (source == fullbodies);
        bool sourceIsTop = (source == tops);
        bool sourceIsBottom = (source == bottoms);
        bool sourceIsHat = (source == hats);
        bool sourceIsAccessory = (source == accessories);

        // Hats / Accessories never remove anything
        if (sourceIsHat || sourceIsAccessory)
            return;

        // Dresses: exclusive
        if (sourceIsDress && !isNone)
        {
            tops?.ClearCategoryAndResetUI();
            bottoms?.ClearCategoryAndResetUI();
            fullbodies?.ClearCategoryAndResetUI();
            return;
        }

        // Fullbodies: exclusive
        if (sourceIsFullbody && !isNone)
        {
            tops?.ClearCategoryAndResetUI();
            bottoms?.ClearCategoryAndResetUI();
            dresses?.ClearCategoryAndResetUI();
            return;
        }

        // Tops/Bottoms: mixable, but selecting them should clear exclusives
        if ((sourceIsTop || sourceIsBottom) && !isNone)
        {
            dresses?.ClearCategoryAndResetUI();
            fullbodies?.ClearCategoryAndResetUI();
            return;
        }

        // If something becomes None, do nothing special.
    }

    /// <summary>
    /// Clears all clothing across all categories and resets UI to "None".
    /// Also resets the pill nav to Clothes view.
    /// Call this when resetting to init state.
    /// </summary>
    /// <summary>
    /// Clears all clothing across all categories and resets UI to "None".
    /// Also resets the pill nav to Clothes view.
    /// Call this when resetting to init state.
    /// </summary>
    public static void ClearAllClothing()
    {
        if (_instance == null) return;

        _instance.tops?.ClearCategoryAndResetUI();
        _instance.bottoms?.ClearCategoryAndResetUI();
        _instance.dresses?.ClearCategoryAndResetUI();
        _instance.fullbodies?.ClearCategoryAndResetUI();
        _instance.hats?.ClearCategoryAndResetUI();
        _instance.accessories?.ClearCategoryAndResetUI();

        // Reset pill nav to Clothes view (both visuals and selector visibility)
        _instance.clothesPillButton?.ResetToClothes();
        _instance.selectorVisibility?.ShowClothes();

        Debug.Log("[ClothingRulesCoordinator] All clothing cleared and pill reset to Clothes.");
    }

    /// <summary>
    /// Save current carousel selections for all categories. Stored internally until
    /// <see cref="RestoreSavedSelections"/> is called. Does nothing if already
    /// saved.
    /// </summary>
    public void SaveCurrentSelections()
    {
        if (_savedSelections != null) return; // already have a snapshot

        _savedSelections = new Dictionary<CarouselClothingManager, string>();
        foreach (var mgr in new[] { tops, bottoms, dresses, fullbodies, hats, accessories })
        {
            if (mgr != null && mgr.carousel != null)
            {
                _savedSelections[mgr] = mgr.carousel.GetCurrentItemName();
            }
        }
        Debug.Log("[ClothingRulesCoordinator] Saved current clothing selections.");
    }

    /// <summary>
    /// Restores the selections previously saved by <see cref="SaveCurrentSelections"/>.
    /// After restoring, the snapshot is discarded. Has no effect if nothing was saved.
    /// </summary>
    public void RestoreSavedSelections()
    {
        if (_savedSelections == null) return;

        foreach (var kvp in _savedSelections)
        {
            var mgr = kvp.Key;
            var name = kvp.Value;
            if (mgr == null || mgr.carousel == null) continue;

            if (string.IsNullOrEmpty(name) || string.Equals(name, "None", System.StringComparison.OrdinalIgnoreCase))
            {
                mgr.ResetToNoneUIOnly();
            }
            else
            {
                mgr.carousel.TrySelectByName(name, notify: true);
            }
        }

        _savedSelections = null;
        Debug.Log("[ClothingRulesCoordinator] Restored saved clothing selections.");
    }
}
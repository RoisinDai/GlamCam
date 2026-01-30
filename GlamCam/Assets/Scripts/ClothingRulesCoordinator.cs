using UnityEngine;

public class ClothingRulesCoordinator : MonoBehaviour
{
    public CarouselClothingManager tops;
    public CarouselClothingManager bottoms;
    public CarouselClothingManager dresses;
    public CarouselClothingManager fullbodies;
    public CarouselClothingManager hats;
    public CarouselClothingManager accessories;

    private static ClothingRulesCoordinator _instance;

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
}
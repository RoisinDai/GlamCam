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
            tops?.DeactivateAll();
            bottoms?.DeactivateAll();
            fullbodies?.DeactivateAll();
            return;
        }

        // Fullbodies: exclusive
        if (sourceIsFullbody && !isNone)
        {
            tops?.DeactivateAll();
            bottoms?.DeactivateAll();
            dresses?.DeactivateAll();
            return;
        }

        // Tops/Bottoms: mixable, but selecting them should clear exclusives
        if ((sourceIsTop || sourceIsBottom) && !isNone)
        {
            dresses?.DeactivateAll();
            fullbodies?.DeactivateAll();
            return;
        }

        // If something becomes None, do nothing special.
    }
}
using UnityEngine;

public class SelectorVisibilityController : MonoBehaviour
{
    [Header("Clothing selectors (children of UIRig)")]
    public GameObject[] clothingSelectors;

    [Header("Accessory selectors (children of UIRig)")]
    public GameObject[] accessorySelectors;

    void Start()
    {
        // Default state on Play
        ShowClothes();
    }

    public void ShowClothes()
    {
        SetActive(clothingSelectors, true);
        SetActive(accessorySelectors, false);
    }

    public void ShowAccessories()
    {
        SetActive(clothingSelectors, false);
        SetActive(accessorySelectors, true);
    }

    private void SetActive(GameObject[] list, bool state)
    {
        foreach (var go in list)
        {
            if (go != null)
                go.SetActive(state);
        }
    }
}
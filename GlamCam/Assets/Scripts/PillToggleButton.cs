using UnityEngine;

public class PillToggleButton : MonoBehaviour
{
    public SelectorVisibilityController controller;
    public bool isClothesButton = true;
    public float cooldownSeconds = 0.5f;

    private float nextAllowedTime;

    private void OnTriggerEnter(Collider other)
    {
        if (!other.CompareTag("HandCursor")) return;
        if (Time.time < nextAllowedTime) return;

        nextAllowedTime = Time.time + cooldownSeconds;

        if (controller == null) return;

        if (isClothesButton)
            controller.ShowClothes();
        else
            controller.ShowAccessories();
    }
}
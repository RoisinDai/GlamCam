using UnityEngine;

public class CarouselArrowButton : MonoBehaviour
{
    public CarouselSelector selector;   // Drag TopSelector (with CarouselSelector) here
    public bool isNext = true;          // RightArrow=true, LeftArrow=false
    public float cooldownSeconds = 0.35f;

    private float _nextAllowedTime = 0f;

    private void OnTriggerEnter(Collider other)
    {
        if (!other.CompareTag("HandCursor")) return;
        if (selector == null) return;

        if (Time.time < _nextAllowedTime) return;
        _nextAllowedTime = Time.time + cooldownSeconds;

        if (isNext) selector.Next();
        else selector.Prev();
    }
}
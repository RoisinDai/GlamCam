using UnityEngine;

public class HoverScale : MonoBehaviour
{
    [Header("What scales")]
    public Transform target;              // drag Icon here

    [Header("Scale")]
    public float hoverScale = 1.3f;      // how big on hover
    public float speed = 12f;             // how snappy the animation feels

    private Vector3 _baseScale;
    private bool _hovering;

    void Awake()
    {
        if (target == null) target = transform;
        _baseScale = target.localScale;
    }

    void Update()
    {
        var desired = _hovering ? _baseScale * hoverScale : _baseScale;
        target.localScale = Vector3.Lerp(target.localScale, desired, Time.deltaTime * speed);
    }

    public void SetHover(bool hovering)
    {
        _hovering = hovering;
    }
}
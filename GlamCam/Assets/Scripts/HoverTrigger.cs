using UnityEngine;

public class HoverTrigger : MonoBehaviour
{
    public string handCursorName = "HandCursor";

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.name != handCursorName) return;
        Debug.Log($"[HoverTrigger] ENTER with {gameObject.name}");
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.gameObject.name != handCursorName) return;
        Debug.Log($"[HoverTrigger] EXIT {gameObject.name}");
    }
}
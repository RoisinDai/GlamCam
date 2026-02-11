using UnityEngine;

public class BaseAvatarMaterialToggler : MonoBehaviour
{
    [SerializeField] private Material invisibleMaskMaterial;
    [SerializeField] private Material greenMaterial;
    
    private Renderer avatarRenderer;
    private bool isInvisibleMaskActive = true;

    void Start()
    {
        // Get the Renderer component
        avatarRenderer = GetComponent<Renderer>();
        
        if (avatarRenderer == null)
        {
            Debug.LogError("BaseAvatarMaterialToggler: No Renderer found on this GameObject!");
            return;
        }

        // Initialize with InvisibleMask material
        if (invisibleMaskMaterial != null)
        {
            avatarRenderer.material = invisibleMaskMaterial;
        }
        else
        {
            Debug.LogWarning("BaseAvatarMaterialToggler: InvisibleMask material not assigned!");
        }
    }

    void Update()
    {
        // Check for 'V' key press
        if (Input.GetKeyDown(KeyCode.V))
        {
            ToggleMaterial();
        }
    }

    private void ToggleMaterial()
    {
        if (avatarRenderer == null || invisibleMaskMaterial == null || greenMaterial == null)
        {
            Debug.LogWarning("BaseAvatarMaterialToggler: Renderer or materials not properly set up!");
            return;
        }

        isInvisibleMaskActive = !isInvisibleMaskActive;

        avatarRenderer.material = isInvisibleMaskActive ? invisibleMaskMaterial : greenMaterial;

        Debug.Log($"Switched to {(isInvisibleMaskActive ? "InvisibleMask" : "Green")} material");
    }
}

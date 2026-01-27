using UnityEngine;
using UnityEngine.UI; // Required for UI components

public class ColorSourceBackground : MonoBehaviour
{
    public ColorSourceManager colorSourceManager;
    private RawImage _backgroundImage;

    void Start()
    {
        _backgroundImage = GetComponent<RawImage>();
        
        // Flip the image horizontally to act like a mirror (Selfie view)
        // Standard UV is (0, 0, 1, 1).
        // Y = 1, Height = -1 ---> Flips Vertically (Fixes the upside down issue)
        _backgroundImage.uvRect = new Rect(0, 1, 1, -1);

        // 2. FORCE RENDER ORDER:
        // We create a new material instance based on the default UI shader.
        // We set its RenderQueue to 1000 (Background).
        // This ensures the video draws FIRST, filling the screen with color.
        // The Invisible Mask will draw LATER (Queue 2000), writing depth on top of the video.
        Material videoMat = new Material(Shader.Find("UI/Default"));
        videoMat.renderQueue = 1000; 
        _backgroundImage.material = videoMat;
    }

    void Update()
    {
        if (colorSourceManager == null)
        {
            return;
        }

        Texture2D texture = colorSourceManager.GetColorTexture();
        
        // Only assign if the texture exists and hasn't been assigned yet to avoid redundant calls
        if (texture != null && _backgroundImage.texture == null)
        {
            _backgroundImage.texture = texture;
            
            // Optional: Ensure the image isn't transparent
            _backgroundImage.color = Color.white; 
        }
    }
}
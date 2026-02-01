using UnityEngine;

// this is the CHILD of the HandCursor GameObject! The collision logic is not determined by the child. 
// purely for visuals!!!
public class HandCursorChild : MonoBehaviour
{
    [Header("Fixed Depth")]
    public float fixedZ = 1.9f;   // slightly in front of UI (which is at Z=2)
    public Camera avatarCamera;   // the camera we're projecting toward

    [Header("Visual Feedback")]
    public Material defaultMaterial;    // open hand (yellow)
    public Material fistClosedMaterial; // closed fist (green)

    private Transform _parent;
    private HandCursorFollower _parentFollower;
    private Renderer _renderer;
    private bool _lastFistState = false;

    void Awake()
    {
        _parent = transform.parent;
        if (_parent != null)
        {
            _parentFollower = _parent.GetComponent<HandCursorFollower>();
        }

        _renderer = GetComponent<Renderer>();

        // Auto-find AvatarCamera if not assigned
        if (avatarCamera == null)
        {
            GameObject camObj = GameObject.Find("AvatarCamera");
            if (camObj != null)
                avatarCamera = camObj.GetComponent<Camera>();
            if (avatarCamera == null)
                avatarCamera = Camera.main;
        }
    }

    void LateUpdate()
    {
        // Project parent's position onto the fixedZ plane using raycast direction
        if (_parent != null && avatarCamera != null)
        {
            Vector3 handPos = _parent.position;
            Vector3 cameraPos = avatarCamera.transform.position;
            Vector3 rayDirection = (cameraPos - handPos).normalized;

            // Calculate where the ray from hand toward camera intersects the Z=fixedZ plane
            // Ray equation: P = handPos + t * rayDirection
            // We want P.z = fixedZ, so: handPos.z + t * rayDirection.z = fixedZ
            // Therefore: t = (fixedZ - handPos.z) / rayDirection.z

            if (Mathf.Abs(rayDirection.z) > 0.001f) // avoid divide by zero
            {
                float t = (fixedZ - handPos.z) / rayDirection.z;
                Vector3 projectedPos = handPos + t * rayDirection;
                transform.position = projectedPos;
            }
            else
            {
                // Fallback if ray is parallel to Z plane
                Vector3 p = handPos;
                p.z = fixedZ;
                transform.position = p;
            }
        }

        // Update material based on fist state
        if (_parentFollower != null && _renderer != null)
        {
            bool isFist = _parentFollower.GetIsFistClosed();
            if (isFist != _lastFistState)
            {
                _lastFistState = isFist;
                Material matToUse = isFist ? fistClosedMaterial : defaultMaterial;
                if (matToUse != null)
                {
                    _renderer.material = matToUse;
                }
            }
        }
    }
}
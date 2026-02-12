using UnityEngine;
using Kinect = Windows.Kinect;

public class HandCursorFollower : MonoBehaviour
{
    public enum HandSide { Right, Left }

    [Header("Hand Selection")]
    public HandSide handToTrack = HandSide.Right;

    public GameObject bodySourceManager;

    public bool useClosestTrackedBody = true;
    public bool smoothMovement = true;

    [Range(0.01f, 1f)]
    public float smoothing = 0.4f;

    public bool hideWhenNotTracked = true;

    private BodySourceManager _bodyManager;
    private bool _wasVisible = true;
    private Renderer[] _renderers;
    private CanvasGroup _canvasGroup;

    [Header("Fist Detection")]
    [Range(0.05f, 0.5f)]
    public float fistDebounceTime = 0.15f;
    private bool _isFistClosed = false;
    private bool _rawFistState = false;
    private float _fistDebounceTimer = 0f;
    
    [Header("Debug Raycast Visualization")]
    public Color rayColorDefault = Color.red;
    public Color rayColorHit = Color.green;
    public float rayLength = 30f;
    private LineRenderer _debugLineRenderer;
    private Camera _avatarCamera;

    public bool GetIsFistClosed() => _isFistClosed;

    void Start()
    {
        if (bodySourceManager == null)
        {
            bodySourceManager = GameObject.Find("BodySourceManager");
        }

        if (bodySourceManager != null)
        {
            _bodyManager = bodySourceManager.GetComponent<BodySourceManager>();
        }

        if (_bodyManager == null)
        {
            Debug.LogWarning("[HandCursorFollower] BodySourceManager not found. Assign it in the inspector.");
        }
        
        // Setup debug ray visualization
        SetupDebugLineRenderer();
        
        // Find AvatarCamera for ray direction
        var camObj = GameObject.Find("AvatarCamera");
        if (camObj != null)
            _avatarCamera = camObj.GetComponent<Camera>();
        if (_avatarCamera == null)
            _avatarCamera = Camera.main;
    }
    
    private void SetupDebugLineRenderer()
    {
        _debugLineRenderer = gameObject.GetComponent<LineRenderer>();
        if (_debugLineRenderer == null)
        {
            _debugLineRenderer = gameObject.AddComponent<LineRenderer>();
        }
        
        _debugLineRenderer.startWidth = 0.005f;
        _debugLineRenderer.endWidth = 0.005f;
        _debugLineRenderer.positionCount = 2;
        _debugLineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        _debugLineRenderer.enabled = false;

        _renderers = GetComponentsInChildren<Renderer>(true);
        _canvasGroup = GetComponentInChildren<CanvasGroup>(true);
    }

    void Update()
    {
        if (_bodyManager == null) return;

        var data = _bodyManager.GetData();
        if (data == null) return;

        Kinect.Body body = useClosestTrackedBody ? GetClosestTrackedBody(data) : GetFirstTrackedBody(data);
        if (body == null)
        {
            SetCursorActive(false);
            return;
        }

        var jointType = handToTrack == HandSide.Right ? Kinect.JointType.HandRight : Kinect.JointType.HandLeft;
        var handJoint = body.Joints[jointType];
        if (handJoint.TrackingState == Kinect.TrackingState.NotTracked)
        {
            SetCursorActive(false);
            return;
        }

        SetCursorActive(true);

        var handState = handToTrack == HandSide.Right ? body.HandRightState : body.HandLeftState;
        bool isFist = (handState == Kinect.HandState.Closed);
        UpdateFistState(isFist);

        // Use the same coordinate transformation as BodySourceView
        Vector3 targetPos = BodySourceView.GetVector3FromJoint(handJoint);

        if (smoothMovement)
        {
            float t = 1f - Mathf.Pow(1f - smoothing, Time.deltaTime * 60f);
            transform.position = Vector3.Lerp(transform.position, targetPos, t);
        }
        else
        {
            transform.position = targetPos;
        }
    }
    
    void LateUpdate()
    {
        UpdateDebugRay();
    }
    
    private void UpdateDebugRay()
    {
        if (_debugLineRenderer == null) return;
        
        // Only show ray when debug camera is active
        if (DebugViewButton.isAvatarCameraActive || _avatarCamera == null)
        {
            _debugLineRenderer.enabled = false;
            return;
        }
        
        _debugLineRenderer.enabled = true;
        
        Vector3 origin = transform.position;
        Vector3 direction = (_avatarCamera.transform.position - origin).normalized;
        Vector3 endPoint = origin + direction * rayLength;
        
        // Check if ray hits something on UI3D layer
        LayerMask uiLayerMask = LayerMask.GetMask("UI3D");
        RaycastHit hit;
        bool didHit = Physics.Raycast(origin, direction, out hit, rayLength, uiLayerMask);
        
        // Set line positions
        _debugLineRenderer.SetPosition(0, origin);
        _debugLineRenderer.SetPosition(1, didHit ? hit.point : endPoint);
        
        // Set color based on hit
        Color rayColor = didHit ? rayColorHit : rayColorDefault;
        _debugLineRenderer.startColor = rayColor;
        _debugLineRenderer.endColor = rayColor;
    }

    private void SetCursorActive(bool visible)
    {
        if (!hideWhenNotTracked) return;
        if (_wasVisible == visible) return;
        _wasVisible = visible;

        if (_renderers != null)
        {
            foreach (var r in _renderers)
            {
                if (r != null) r.enabled = visible;
            }
        }

        if (_canvasGroup != null)
        {
            _canvasGroup.alpha = visible ? 1f : 0f;
            _canvasGroup.blocksRaycasts = visible;
            _canvasGroup.interactable = visible;
        }
    }

    private void UpdateFistState(bool isFist)
    {
        if (isFist)
        {
            // Closed detected – commit immediately
            _fistDebounceTimer = 0f;
            if (_isFistClosed) return;
            _isFistClosed = true;
            return;
        }

        // Open detected – only commit after debounce
        if (!_isFistClosed) { _fistDebounceTimer = 0f; return; }

        _fistDebounceTimer += Time.deltaTime;
        if (_fistDebounceTimer < fistDebounceTime) return;

        _isFistClosed = false;
        _fistDebounceTimer = 0f;
    }

    private Kinect.Body GetFirstTrackedBody(Kinect.Body[] bodies)
    {
        foreach (var body in bodies)
        {
            if (body != null && body.IsTracked)
            {
                return body;
            }
        }
        return null;
    }

    private Kinect.Body GetClosestTrackedBody(Kinect.Body[] bodies)
    {
        Kinect.Body closestBody = null;
        float closestZ = float.MaxValue;

        foreach (var body in bodies)
        {
            if (body == null || !body.IsTracked) continue;

            var joints = body.Joints;
            float avgZ = 0f;
            int validJointCount = 0;

            var torsoJoints = new[]
            {
                Kinect.JointType.SpineBase,
                Kinect.JointType.SpineMid,
                Kinect.JointType.SpineShoulder
            };

            foreach (var jointType in torsoJoints)
            {
                var joint = joints[jointType];
                if (joint.TrackingState != Kinect.TrackingState.NotTracked)
                {
                    avgZ += joint.Position.Z;
                    validJointCount++;
                }
            }

            if (validJointCount == 0) continue;

            avgZ /= validJointCount;
            if (avgZ < closestZ)
            {
                closestZ = avgZ;
                closestBody = body;
            }
        }

        return closestBody;
    }
}

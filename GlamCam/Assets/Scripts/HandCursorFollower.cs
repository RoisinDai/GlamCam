using UnityEngine;
using Kinect = Windows.Kinect;

public class HandCursorFollower : MonoBehaviour
{
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

        var handJoint = body.Joints[Kinect.JointType.HandRight];
        if (handJoint.TrackingState == Kinect.TrackingState.NotTracked)
        {
            SetCursorActive(false);
            return;
        }

        SetCursorActive(true);

        var handRightState = body.HandRightState;
        bool isFist = (handRightState == Kinect.HandState.Closed);
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

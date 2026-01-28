using UnityEngine;
using Kinect = Windows.Kinect;

public class HandCursorFollower : MonoBehaviour
{
    public GameObject bodySourceManager;

    public bool useClosestTrackedBody = true;

    public Vector3 positionOffset = Vector3.zero;

    public Camera avatarCamera;

    public float depthFromCamera = 2f;

    public bool smoothMovement = true;

    [Range(0.01f, 1f)]
    public float smoothing = 0.4f;

    public bool hideWhenNotTracked = true;

    private BodySourceManager _bodyManager;
    private bool _wasVisible = true;
    private Renderer[] _renderers;
    private CanvasGroup _canvasGroup;
    private Kinect.KinectSensor _sensor;
    private Kinect.CoordinateMapper _coordinateMapper;
    private int _colorWidth = 1920;
    private int _colorHeight = 1080;

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

        if (avatarCamera == null)
        {
            avatarCamera = Camera.main;
        }

        _sensor = Kinect.KinectSensor.GetDefault();
        if (_sensor != null)
        {
            _coordinateMapper = _sensor.CoordinateMapper;
            var desc = _sensor.ColorFrameSource?.FrameDescription;
            if (desc != null)
            {
                _colorWidth = desc.Width;
                _colorHeight = desc.Height;
            }
        }
        else
        {
            Debug.LogWarning("[HandCursorFollower] Kinect sensor not found.");
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

        if (_coordinateMapper == null || avatarCamera == null)
        {
            SetCursorActive(false);
            return;
        }

        var cameraPoint = new Kinect.CameraSpacePoint
        {
            X = handJoint.Position.X,
            Y = handJoint.Position.Y,
            Z = handJoint.Position.Z
        };

        Kinect.ColorSpacePoint colorPoint = _coordinateMapper.MapCameraPointToColorSpace(cameraPoint);
        if (float.IsNaN(colorPoint.X) || float.IsNaN(colorPoint.Y) ||
            float.IsInfinity(colorPoint.X) || float.IsInfinity(colorPoint.Y))
        {
            SetCursorActive(false);
            return;
        }

        if (colorPoint.X < 0 || colorPoint.Y < 0 || colorPoint.X > _colorWidth || colorPoint.Y > _colorHeight)
        {
            SetCursorActive(false);
            return;
        }

        float screenX = (colorPoint.X / _colorWidth) * Screen.width;
        float screenY = (1f - (colorPoint.Y / _colorHeight)) * Screen.height;
        Vector3 screenPos = new Vector3(screenX, screenY, depthFromCamera);

        Vector3 targetPos = avatarCamera.ScreenToWorldPoint(screenPos) + positionOffset;

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

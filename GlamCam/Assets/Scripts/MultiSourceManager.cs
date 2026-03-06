using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Windows.Kinect;

public class MultiSourceManager : MonoBehaviour {
    public int ColorWidth { get; private set; }
    public int ColorHeight { get; private set; }

    // Measurement status (polled by AvatarController and UI)
    public bool IsMeasured { get; private set; } = false;
    public float MeasuredSpineMidWidth { get; private set; } = -1f;
    public float MeasuredHeight { get; private set; } = -1f;

    // T-pose bone length measurements (raw meters, averaged left/right)
    public float MeasuredUpperArmLength { get; private set; } = -1f; // Shoulder → Elbow
    public float MeasuredLowerArmLength { get; private set; } = -1f; // Elbow → Wrist
    public float MeasuredUpperLegLength { get; private set; } = -1f; // Hip → Knee
    public float MeasuredLowerLegLength { get; private set; } = -1f; // Knee → Foot
    public float MeasuredShoulderDist { get; private set; } = -1f; // Avg(SpineShoulder → ShoulderLeft/Right)

    private KinectSensor _Sensor;
    private MultiSourceFrameReader _Reader;
    private CoordinateMapper _Mapper;
    private Texture2D _ColorTexture;
    private ushort[] _DepthData;
    private byte[] _ColorData;
    private byte[] _BodyIndexData;
    
    // Body tracking
    private Body[] _BodyData;
    private Body _TrackedBody;

    // T-pose detection state
    private float _TPoseHoldTimer = 0f;
    private const float TPOSE_HOLD_DURATION = 1.0f;
    private const float TPOSE_ARM_EXTENSION_MIN = 0.3f; // minimum horizontal arm spread (meters)
    private const float TPOSE_ANGLE_RATIO_MAX = 0.4f;   // max |deltaY/deltaX| (~22° from horizontal)
    
    // Multi-sample measurement for robustness (reduces noise/fluctuation)
    private const int MEASUREMENT_SAMPLE_COUNT = 20;
    private List<float> _HeightSamples = new List<float>();
    private List<float> _WidthSamples = new List<float>();
    private List<float> _UpperArmSamples = new List<float>();
    private List<float> _LowerArmSamples = new List<float>();
    private List<float> _UpperLegSamples = new List<float>();
    private List<float> _LowerLegSamples = new List<float>();
    private List<float> _ShoulderDistSamples = new List<float>();
    private bool _IsSampling = false; // True after TPOSE_HOLD_DURATION, collecting samples
    
    // For color-to-depth mapping (Body Mask)
    private DepthSpacePoint[] _ColorMappedToDepthPoints;
    
    private const int DEPTH_WIDTH = 512;
    private const int DEPTH_HEIGHT = 424;
    private const float DEPTH_HORIZONTAL_FOV = 70.6f; // Kinect v2 depth camera horizontal FOV

    public Texture2D GetColorTexture()
    {
        return _ColorTexture;
    }
    
    public ushort[] GetDepthData()
    {
        return _DepthData;
    }

    public byte[] GetBodyIndexData()
    {
        return _BodyIndexData;
    }
    
    public CoordinateMapper GetCoordinateMapper()
    {
        return _Mapper;
    }

    void Start () 
    {
        _Sensor = KinectSensor.GetDefault();
        
        if (_Sensor != null) 
        {
            // Include Body in the frame reader
            _Reader = _Sensor.OpenMultiSourceFrameReader(
                FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.BodyIndex | FrameSourceTypes.Body);
            
            var colorFrameDesc = _Sensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Rgba);
            ColorWidth = colorFrameDesc.Width;
            ColorHeight = colorFrameDesc.Height;
            
            _ColorTexture = new Texture2D(colorFrameDesc.Width, colorFrameDesc.Height, TextureFormat.RGBA32, false);
            _ColorData = new byte[colorFrameDesc.BytesPerPixel * colorFrameDesc.LengthInPixels];
            
            var depthFrameDesc = _Sensor.DepthFrameSource.FrameDescription;
            _DepthData = new ushort[depthFrameDesc.LengthInPixels];
            _BodyIndexData = new byte[depthFrameDesc.LengthInPixels]; // 512 x 424
            
            // Initialize body array
            _BodyData = new Body[_Sensor.BodyFrameSource.BodyCount];
            
            _Mapper = _Sensor.CoordinateMapper;
            
            // For Body Mask - mapping color pixels to depth space
            _ColorMappedToDepthPoints = new DepthSpacePoint[colorFrameDesc.Width * colorFrameDesc.Height];

            if (!_Sensor.IsOpen)
            {
                _Sensor.Open();
            }
            
            Debug.Log("MultiSourceManager: Started successfully. Press M or click button to capture.");
        }
        else
        {
            Debug.LogError("MultiSourceManager: Kinect sensor not found!");
        }
    }
    
    void Update ()
    {
        // Nothing to do once measured — AvatarController uses BodySourceManager for tracking
        if (IsMeasured) return;

        if (_Reader != null)
        {
            var frame = _Reader.AcquireLatestFrame();
            if (frame != null)
            {
                var colorFrame = frame.ColorFrameReference.AcquireFrame();
                if (colorFrame != null)
                {
                    var depthFrame = frame.DepthFrameReference.AcquireFrame();
                    if (depthFrame != null)
                    {
                        var bodyIndexFrame = frame.BodyIndexFrameReference.AcquireFrame();
                        if (bodyIndexFrame != null)
                        {
                            var bodyFrame = frame.BodyFrameReference.AcquireFrame();
                            if (bodyFrame != null)
                            {
                                colorFrame.CopyConvertedFrameDataToArray(_ColorData, ColorImageFormat.Rgba);
                                _ColorTexture.LoadRawTextureData(_ColorData);
                                _ColorTexture.Apply();
                                
                                depthFrame.CopyFrameDataToArray(_DepthData);
                                bodyIndexFrame.CopyFrameDataToArray(_BodyIndexData);
                                bodyFrame.GetAndRefreshBodyData(_BodyData);
                                
                                // Get the closest tracked body to the camera
                                _TrackedBody = GetClosestTrackedBody(_BodyData);
                                
                                bodyFrame.Dispose();
                                bodyFrame = null;
                            }
                            
                            bodyIndexFrame.Dispose();
                            bodyIndexFrame = null;
                        }
                        
                        depthFrame.Dispose();
                        depthFrame = null;
                    }
                
                    colorFrame.Dispose();
                    colorFrame = null;
                }
                
                frame = null;
            }
        }
        
        // T-pose detection and multi-sample measurement for robustness
        if (!IsMeasured && _TrackedBody != null && _TrackedBody.IsTracked)
        {
            if (DetectTPose())
            {
                _TPoseHoldTimer += Time.deltaTime;

                // After holding T-pose for required duration, start collecting samples
                if (_TPoseHoldTimer >= TPOSE_HOLD_DURATION)
                {
                    if (!_IsSampling)
                    {
                        _IsSampling = true;
                        Debug.Log($"[MultiSourceManager] T-pose held for {TPOSE_HOLD_DURATION}s. Starting to collect {MEASUREMENT_SAMPLE_COUNT} samples...");
                    }

                    // Collect one sample per frame
                    CollectMeasurementSample();

                    // Check if we have enough samples
                    if (_HeightSamples.Count >= MEASUREMENT_SAMPLE_COUNT)
                    {
                        // Compute mean of all samples
                        MeasuredHeight = _HeightSamples.Average();
                        MeasuredSpineMidWidth = _WidthSamples.Average();
                        MeasuredUpperArmLength = _UpperArmSamples.Average();
                        MeasuredLowerArmLength = _LowerArmSamples.Average();
                        MeasuredUpperLegLength = _UpperLegSamples.Average();
                        MeasuredLowerLegLength = _LowerLegSamples.Average();
                        MeasuredShoulderDist = _ShoulderDistSamples.Average();

                        IsMeasured = true;
                        _IsSampling = false;

                        // Divide by 10 since the measured height is in decimeters
                        float heightMeters = MeasuredHeight / 10f;
                        float widthMeters = MeasuredSpineMidWidth / 10f;
                        Debug.Log($"[MultiSourceManager] Measurement complete ({MEASUREMENT_SAMPLE_COUNT} samples averaged). Height={MeasuredHeight:F3} units ({heightMeters:F3}m) Width={MeasuredSpineMidWidth:F3} units ({widthMeters:F3}m)");
                        Debug.Log($"[MultiSourceManager] Bone lengths (m): UpperArm={MeasuredUpperArmLength:F4}, LowerArm={MeasuredLowerArmLength:F4}, UpperLeg={MeasuredUpperLegLength:F4}, LowerLeg={MeasuredLowerLegLength:F4}");
                    }
                }
            }
            else
            {
                // T-pose lost - reset sampling state
                if (_IsSampling)
                {
                    Debug.Log($"[MultiSourceManager] T-pose lost during sampling. Collected {_HeightSamples.Count}/{MEASUREMENT_SAMPLE_COUNT} samples. Restarting...");
                }
                _TPoseHoldTimer = 0f;
                _IsSampling = false;
                ClearMeasurementSamples();
            }
        }
    }
    
    /// <summary>
    /// Converts a Kinect joint to Unity Vector3 (in meters).
    /// </summary>
    private Vector3 GetVector3FromJoint(Windows.Kinect.Joint joint)
    {
        return new Vector3(joint.Position.X, joint.Position.Y, joint.Position.Z);
    }

    /// <summary>
    /// Measures the user's height from Head to the average of both feet (in meters).
    /// Called once during T-pose when the user is standing upright.
    /// </summary>
    private float MeasureUserHeight()
    {
        var joints = _TrackedBody.Joints;
        Vector3 head = BodySourceView.GetVector3FromJoint(joints[JointType.Head]);
        Vector3 footLeft = BodySourceView.GetVector3FromJoint(joints[JointType.FootLeft]);
        Vector3 footRight = BodySourceView.GetVector3FromJoint(joints[JointType.FootRight]);

        float height = head.y - ((footLeft.y + footRight.y) * 0.5f);
        Debug.Log($"[MultiSourceManager] Measured user height: {height:F3} units");
        return height;
    }

    /// <summary>
    /// Measures all bone lengths during T-pose using raw Kinect coordinates (meters).
    /// Averages left and right sides for bilateral bones.
    /// Uses raw coordinates (no ×10 scaling) to match AvatarController's local-space bone measurements.
    /// </summary>
    private void MeasureBoneLengths()
    {
        var joints = _TrackedBody.Joints;

        // Upper arm: Shoulder → Elbow (avg left/right)
        float leftUpperArm = RawJointDistance(joints[JointType.ShoulderLeft], joints[JointType.ElbowLeft]);
        float rightUpperArm = RawJointDistance(joints[JointType.ShoulderRight], joints[JointType.ElbowRight]);
        MeasuredUpperArmLength = (leftUpperArm + rightUpperArm) * 0.5f;

        // Lower arm: Elbow → Wrist (avg left/right)
        float leftLowerArm = RawJointDistance(joints[JointType.ElbowLeft], joints[JointType.WristLeft]);
        float rightLowerArm = RawJointDistance(joints[JointType.ElbowRight], joints[JointType.WristRight]);
        MeasuredLowerArmLength = (leftLowerArm + rightLowerArm) * 0.5f;

        // Upper leg: Hip → Knee (avg left/right)
        float leftUpperLeg = RawJointDistance(joints[JointType.HipLeft], joints[JointType.KneeLeft]);
        float rightUpperLeg = RawJointDistance(joints[JointType.HipRight], joints[JointType.KneeRight]);
        MeasuredUpperLegLength = (leftUpperLeg + rightUpperLeg) * 0.5f;

        // Lower leg: Knee → Ankle (avg left/right)
        float leftLowerLeg = RawJointDistance(joints[JointType.KneeLeft], joints[JointType.AnkleLeft]);
        float rightLowerLeg = RawJointDistance(joints[JointType.KneeRight], joints[JointType.AnkleRight]);
        MeasuredLowerLegLength = (leftLowerLeg + rightLowerLeg) * 0.5f;

        Debug.Log($"[MultiSourceManager] Bone lengths (m): UpperArm={MeasuredUpperArmLength:F4}, LowerArm={MeasuredLowerArmLength:F4}, UpperLeg={MeasuredUpperLegLength:F4}, LowerLeg={MeasuredLowerLegLength:F4}");
    }

    /// <summary>
    /// Calculates the distance between two joints using raw Kinect coordinates (meters, no ×10).
    /// </summary>
    private float RawJointDistance(Windows.Kinect.Joint a, Windows.Kinect.Joint b)
    {
        Vector3 posA = GetVector3FromJoint(a);
        Vector3 posB = GetVector3FromJoint(b);
        return Vector3.Distance(posA, posB);
    }
    
    // =====================================================
    // T-Pose Detection and Measurement
    // =====================================================

    /// <summary>
    /// Re-triggers the T-pose measurement cycle.
    /// Call this from UI to re-measure.
    /// </summary>
    public void StartMeasurement()
    {
        IsMeasured = false;
        MeasuredSpineMidWidth = -1f;
        MeasuredHeight = -1f;
        MeasuredUpperArmLength = -1f;
        MeasuredLowerArmLength = -1f;
        MeasuredUpperLegLength = -1f;
        MeasuredLowerLegLength = -1f;
        MeasuredShoulderDist = -1f;
        _TPoseHoldTimer = 0f;
        _IsSampling = false;
        ClearMeasurementSamples();
    }

    /// <summary>
    /// Clears all measurement sample lists.
    /// </summary>
    private void ClearMeasurementSamples()
    {
        _HeightSamples.Clear();
        _WidthSamples.Clear();
        _UpperArmSamples.Clear();
        _LowerArmSamples.Clear();
        _UpperLegSamples.Clear();
        _LowerLegSamples.Clear();
        _ShoulderDistSamples.Clear();
    }

    /// <summary>
    /// Collects one measurement sample for all measurements.
    /// Called once per frame while sampling.
    /// </summary>
    private void CollectMeasurementSample()
    {
        // Height
        float height = MeasureUserHeight();
        if (height > 0) _HeightSamples.Add(height);

        // Width
        float width = MeasureSpineMidWidth();
        if (width > 0) _WidthSamples.Add(width);

        // Bone lengths
        var joints = _TrackedBody.Joints;

        // Upper arm
        float leftUpperArm = RawJointDistance(joints[JointType.ShoulderLeft], joints[JointType.ElbowLeft]);
        float rightUpperArm = RawJointDistance(joints[JointType.ShoulderRight], joints[JointType.ElbowRight]);
        float upperArm = (leftUpperArm + rightUpperArm) * 0.5f;
        if (upperArm > 0) _UpperArmSamples.Add(upperArm);

        // Lower arm
        float leftLowerArm = RawJointDistance(joints[JointType.ElbowLeft], joints[JointType.WristLeft]);
        float rightLowerArm = RawJointDistance(joints[JointType.ElbowRight], joints[JointType.WristRight]);
        float lowerArm = (leftLowerArm + rightLowerArm) * 0.5f;
        if (lowerArm > 0) _LowerArmSamples.Add(lowerArm);

        // Upper leg
        float leftUpperLeg = RawJointDistance(joints[JointType.HipLeft], joints[JointType.KneeLeft]);
        float rightUpperLeg = RawJointDistance(joints[JointType.HipRight], joints[JointType.KneeRight]);
        float upperLeg = (leftUpperLeg + rightUpperLeg) * 0.5f;
        if (upperLeg > 0) _UpperLegSamples.Add(upperLeg);

        // Lower leg
        float leftLowerLeg = RawJointDistance(joints[JointType.KneeLeft], joints[JointType.FootLeft]);
        float rightLowerLeg = RawJointDistance(joints[JointType.KneeRight], joints[JointType.FootRight]);
        float lowerLeg = (leftLowerLeg + rightLowerLeg) * 0.5f;
        if (lowerLeg > 0) _LowerLegSamples.Add(lowerLeg);

        // Shoulder half-distance: avg(SpineShoulder → ShoulderRight, SpineShoulder → ShoulderLeft)
        float shoulderlDist = RawJointDistance(joints[JointType.SpineShoulder], joints[JointType.ShoulderRight]);
        float shoulderrDist = RawJointDistance(joints[JointType.SpineShoulder], joints[JointType.ShoulderLeft]);
        float shoulderDist = (shoulderlDist + shoulderrDist) * 0.5f;
        if (shoulderDist > 0) _ShoulderDistSamples.Add(shoulderDist);
    }

    /// <summary>
    /// Detects whether the tracked body is in a T-pose.
    /// Checks that both arms are extended horizontally.
    /// </summary>
    private bool DetectTPose()
    {
        if (_TrackedBody == null || !_TrackedBody.IsTracked)
            return false;

        var joints = _TrackedBody.Joints;

        Vector3 shoulderL = GetVector3FromJoint(joints[JointType.ShoulderLeft]);
        Vector3 shoulderR = GetVector3FromJoint(joints[JointType.ShoulderRight]);
        Vector3 elbowL = GetVector3FromJoint(joints[JointType.ElbowLeft]);
        Vector3 elbowR = GetVector3FromJoint(joints[JointType.ElbowRight]);
        Vector3 wristL = GetVector3FromJoint(joints[JointType.WristLeft]);
        Vector3 wristR = GetVector3FromJoint(joints[JointType.WristRight]);

        // Check left arm
        float leftArmDeltaX = Mathf.Abs(wristL.x - shoulderL.x);
        float leftArmDeltaY = Mathf.Abs(wristL.y - shoulderL.y);
        float leftElbowDeltaX = Mathf.Abs(elbowL.x - shoulderL.x);
        float leftElbowDeltaY = Mathf.Abs(elbowL.y - shoulderL.y);

        // Check right arm
        float rightArmDeltaX = Mathf.Abs(wristR.x - shoulderR.x);
        float rightArmDeltaY = Mathf.Abs(wristR.y - shoulderR.y);
        float rightElbowDeltaX = Mathf.Abs(elbowR.x - shoulderR.x);
        float rightElbowDeltaY = Mathf.Abs(elbowR.y - shoulderR.y);

        // Arms must be spread out horizontally
        if (leftArmDeltaX < TPOSE_ARM_EXTENSION_MIN || rightArmDeltaX < TPOSE_ARM_EXTENSION_MIN)
            return false;

        // Wrists must be roughly horizontal relative to shoulders
        if (leftArmDeltaY / leftArmDeltaX > TPOSE_ANGLE_RATIO_MAX)
            return false;
        if (rightArmDeltaY / rightArmDeltaX > TPOSE_ANGLE_RATIO_MAX)
            return false;

        // Elbows must also be roughly horizontal
        if (leftElbowDeltaX > 0.01f && leftElbowDeltaY / leftElbowDeltaX > TPOSE_ANGLE_RATIO_MAX)
            return false;
        if (rightElbowDeltaX > 0.01f && rightElbowDeltaY / rightElbowDeltaX > TPOSE_ANGLE_RATIO_MAX)
            return false;

        return true;
    }

    // =====================================================
    // Body Width Measurement Methods
    // =====================================================

    /// <summary>
    /// Gets the current tracked body (if any).
    /// </summary>
    public Body GetTrackedBody()
    {
        return _TrackedBody;
    }

    /// <summary>
    /// Selects the tracked body closest to the Kinect camera based on average torso Z depth.
    /// Matches the same logic used by AvatarController.GetClosestTrackedBody().
    /// </summary>
    private Body GetClosestTrackedBody(Body[] bodies)
    {
        Body closestBody = null;
        float closestZ = float.MaxValue;

        foreach (var body in bodies)
        {
            if (body == null || !body.IsTracked) continue;

            var joints = body.Joints;
            float avgZ = 0f;
            int count = 0;

            var torsoJoints = new[] { JointType.SpineBase, JointType.SpineMid, JointType.SpineShoulder };
            foreach (var jt in torsoJoints)
            {
                if (joints[jt].TrackingState != TrackingState.NotTracked)
                {
                    avgZ += joints[jt].Position.Z;
                    count++;
                }
            }

            if (count > 0)
            {
                avgZ /= count;
                if (avgZ < closestZ)
                {
                    closestZ = avgZ;
                    closestBody = body;
                }
            }
        }

        return closestBody;
    }
    
    /// <summary>
    /// Measures body width at the midpoint between SpineMid and SpineBase.
    /// Returns width in units, or 0 if measurement failed.
    /// </summary>
    public float MeasureSpineMidWidth()
    {
        if (_TrackedBody == null || !_TrackedBody.IsTracked)
        {
            return 0f;
        }

        var spineMid = _TrackedBody.Joints[JointType.SpineMid];
        var spineBase = _TrackedBody.Joints[JointType.SpineBase];

        if (spineMid.TrackingState == TrackingState.NotTracked ||
            spineBase.TrackingState == TrackingState.NotTracked)
        {
            return 0f;
        }

        // Calculate midpoint between SpineMid and SpineBase in camera space
        CameraSpacePoint midpoint = new CameraSpacePoint
        {
            X = (spineMid.Position.X + spineBase.Position.X) / 2f,
            Y = (spineMid.Position.Y + spineBase.Position.Y) / 2f,
            Z = (spineMid.Position.Z + spineBase.Position.Z) / 2f
        };

        return MeasureWidthAtPosition(midpoint);
    }
    
    /// <summary>
    /// Measures the body width at a specific joint by scanning the silhouette horizontally.
    /// </summary>
    public float MeasureWidthAtJoint(Windows.Kinect.Joint joint)
    {
        if (joint.TrackingState == TrackingState.NotTracked)
        {
            Debug.LogWarning("MeasureWidthAtJoint: Joint not tracked");
            return 0f;
        }

        return MeasureWidthAtPosition(joint.Position);
    }

    /// <summary>
    /// Measures the body width at a specific camera-space position by scanning the silhouette horizontally.
    /// </summary>
    public float MeasureWidthAtPosition(CameraSpacePoint position)
    {
        if (_BodyIndexData == null || _DepthData == null || _Mapper == null)
        {
            Debug.LogWarning("MeasureWidthAtPosition: Required data not available");
            return 0f;
        }

        // Convert camera space position to depth space (2D pixels)
        DepthSpacePoint depthPoint = _Mapper.MapCameraPointToDepthSpace(position);

        int centerX = (int)(depthPoint.X + 0.5f);
        int centerY = (int)(depthPoint.Y + 0.5f);

        // Check bounds
        if (centerX < 0 || centerX >= DEPTH_WIDTH || centerY < 0 || centerY >= DEPTH_HEIGHT)
        {
            Debug.LogWarning("MeasureWidthAtPosition: Position out of depth frame bounds");
            return 0f;
        }

        // Scan LEFT from center to find left edge
        int leftEdge = centerX;
        for (int x = centerX; x >= 0; x--)
        {
            int index = centerY * DEPTH_WIDTH + x;
            if (_BodyIndexData[index] == 255) // Not a body pixel
            {
                leftEdge = x + 1;
                break;
            }
            if (x == 0) leftEdge = 0;
        }

        // Scan RIGHT from center to find right edge
        int rightEdge = centerX;
        for (int x = centerX; x < DEPTH_WIDTH; x++)
        {
            int index = centerY * DEPTH_WIDTH + x;
            if (_BodyIndexData[index] == 255) // Not a body pixel
            {
                rightEdge = x - 1;
                break;
            }
            if (x == DEPTH_WIDTH - 1) rightEdge = DEPTH_WIDTH - 1;
        }

        int widthInPixels = rightEdge - leftEdge + 1;

        // Get depth at the center point (in millimeters)
        int centerIndex = centerY * DEPTH_WIDTH + centerX;
        float depthMm = _DepthData[centerIndex];

        if (depthMm <= 0)
        {
            Debug.LogWarning("MeasureWidthAtPosition: Invalid depth value");
            return 0f;
        }

        // Convert pixels to meters
        float widthInMeters = PixelsToMeters(widthInPixels, depthMm);
        float widthInUnits = widthInMeters * 10f;

        Debug.Log($"MeasureWidthAtPosition: center=({centerX},{centerY}), pixels={widthInPixels}, depth={depthMm}mm, width={widthInMeters:F3}m, width={widthInUnits:F3} units");

        return widthInUnits;
    }
    
    /// <summary>
    /// Converts a horizontal pixel distance to real-world meters at a given depth.
    /// </summary>
    private float PixelsToMeters(int pixels, float depthMm)
    {
        float depthM = depthMm / 1000f;
        float halfFovRad = (DEPTH_HORIZONTAL_FOV / 2f) * Mathf.Deg2Rad;
        float frameWidthAtDepth = 2f * depthM * Mathf.Tan(halfFovRad);
        float metersPerPixel = frameWidthAtDepth / DEPTH_WIDTH;
        return pixels * metersPerPixel;
    }
    
    void OnApplicationQuit()
    {
        if (_Reader != null)
        {
            _Reader.Dispose();
            _Reader = null;
        }
        
        if (_Sensor != null)
        {
            if (_Sensor.IsOpen)
            {
                _Sensor.Close();
            }
            
            _Sensor = null;
        }
    }
}

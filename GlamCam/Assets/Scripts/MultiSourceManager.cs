using UnityEngine;
using System.Collections;
using System.Linq;
using Windows.Kinect;

public class MultiSourceManager : MonoBehaviour {
    public int ColorWidth { get; private set; }
    public int ColorHeight { get; private set; }

    // Measurement status (polled by AvatarController and UI)
    public bool IsMeasured { get; private set; } = false;
    public float MeasuredSpineMidWidth { get; private set; } = -1f;
    public float MeasuredHeight { get; private set; } = -1f;

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
    private const float TPOSE_ANGLE_RATIO_MAX = 0.3f;   // max |deltaY/deltaX| (~17° from horizontal)
    
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
                                
                                // Get first tracked body
                                _TrackedBody = _BodyData.FirstOrDefault(b => b != null && b.IsTracked);
                                
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
        
        // T-pose detection and one-shot measurement
        if (!IsMeasured && _TrackedBody != null && _TrackedBody.IsTracked)
        {
            if (DetectTPose())
            {
                _TPoseHoldTimer += Time.deltaTime;

                if (_TPoseHoldTimer >= TPOSE_HOLD_DURATION)
                {
                    MeasuredSpineMidWidth = MeasureSpineMidWidth();
                    MeasuredHeight = MeasureUserHeight();
                    IsMeasured = true;
                    Debug.Log($"[MultiSourceManager] Measurement complete. Height={MeasuredHeight:F3}m Width={MeasuredSpineMidWidth:F3}m");
                }
            }
            else
            {
                _TPoseHoldTimer = 0f;
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
        Vector3 head = GetVector3FromJoint(joints[JointType.Head]);
        Vector3 footLeft = GetVector3FromJoint(joints[JointType.FootLeft]);
        Vector3 footRight = GetVector3FromJoint(joints[JointType.FootRight]);

        float height = head.y - ((footLeft.y + footRight.y) * 0.5f);
        Debug.Log($"[MultiSourceManager] Measured user height: {height:F3}m");
        return height;
    }

    // GUI buttons for visualization
    void OnGUI()
    {
        if (GUI.Button(new Rect(10, 10, 200, 50), "Capture Body Mask"))
        {
            Debug.Log("Button clicked - capturing body mask...");
            CaptureBodyMask();
        }
        
        if (GUI.Button(new Rect(10, 70, 200, 50), "Capture Silhouette"))
        {
            Debug.Log("Button clicked - capturing silhouette...");
            CaptureSilhouette();
        }
    }
    
    /// <summary>
    /// Captures raw BodyIndexFrame as a simple silhouette image.
    /// White = body, Black = background. Resolution: 512×424.
    /// </summary>
    private void CaptureSilhouette()
    {
        if (_BodyIndexData == null)
        {
            Debug.LogError("CaptureSilhouette: _BodyIndexData is null");
            return;
        }
        
        // Create texture at depth resolution
        Texture2D texture = new Texture2D(DEPTH_WIDTH, DEPTH_HEIGHT, TextureFormat.RGBA32, false);
        Color32[] pixels = new Color32[DEPTH_WIDTH * DEPTH_HEIGHT];
        
        int bodyPixelCount = 0;
        
        for (int i = 0; i < _BodyIndexData.Length; i++)
        {
            if (_BodyIndexData[i] != 255)
            {
                // Body pixel - WHITE
                pixels[i] = new Color32(255, 255, 255, 255);
                bodyPixelCount++;
            }
            else
            {
                // Background - BLACK
                pixels[i] = new Color32(0, 0, 0, 255);
            }
        }
        
        texture.SetPixels32(pixels);
        texture.Apply();
        
        Debug.Log($"CaptureSilhouette: Found {bodyPixelCount} body pixels");
        
        // Save to Desktop
        try
        {
            byte[] pngData = texture.EncodeToPNG();
            string path = System.Environment.GetFolderPath(System.Environment.SpecialFolder.Desktop) + "/Silhouette.png";
            System.IO.File.WriteAllBytes(path, pngData);
            Debug.Log($"CaptureSilhouette: Saved to {path}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"CaptureSilhouette: Failed - {e.Message}");
        }
        
        Destroy(texture);
    }
    
    /// <summary>
    /// Captures Body Mask - color image with background removed (Lab 05 approach).
    /// Maps color pixels to depth space and checks BodyIndex to keep only body pixels.
    /// </summary>
    private void CaptureBodyMask()
    {
        Debug.Log("CaptureBodyMask: Starting capture...");
        
        // Checkpoint 1: Check frame data
        if (_ColorData == null)
        {
            Debug.LogError("CaptureBodyMask: _ColorData is null - no color frame received");
            return;
        }
        if (_DepthData == null)
        {
            Debug.LogError("CaptureBodyMask: _DepthData is null - no depth frame received");
            return;
        }
        if (_BodyIndexData == null)
        {
            Debug.LogError("CaptureBodyMask: _BodyIndexData is null - no body index frame received");
            return;
        }
        Debug.Log("CaptureBodyMask: All frame data available");
        
        // Checkpoint 2: Check mapper
        if (_Mapper == null)
        {
            Debug.LogError("CaptureBodyMask: CoordinateMapper not available");
            return;
        }
        Debug.Log("CaptureBodyMask: CoordinateMapper ready");
        
        // Map color frame to depth space
        _Mapper.MapColorFrameToDepthSpace(_DepthData, _ColorMappedToDepthPoints);
        
        // Create output texture at color resolution
        Texture2D texture = new Texture2D(ColorWidth, ColorHeight, TextureFormat.RGBA32, false);
        Color32[] colors = new Color32[ColorWidth * ColorHeight];
        
        int bodyPixelCount = 0;
        
        // For each color pixel, check if it corresponds to a body
        for (int colorIndex = 0; colorIndex < _ColorMappedToDepthPoints.Length; colorIndex++)
        {
            float colorMappedToDepthX = _ColorMappedToDepthPoints[colorIndex].X;
            float colorMappedToDepthY = _ColorMappedToDepthPoints[colorIndex].Y;
            
            bool isBody = false;
            
            // Check if this color pixel maps to a valid depth point
            if (!float.IsNegativeInfinity(colorMappedToDepthX) && 
                !float.IsNegativeInfinity(colorMappedToDepthY))
            {
                int depthX = (int)(colorMappedToDepthX + 0.5f);
                int depthY = (int)(colorMappedToDepthY + 0.5f);
                
                // Check bounds
                if (depthX >= 0 && depthX < DEPTH_WIDTH && 
                    depthY >= 0 && depthY < DEPTH_HEIGHT)
                {
                    int depthIndex = (depthY * DEPTH_WIDTH) + depthX;
                    
                    // Check if this pixel is a body (not 0xff/255)
                    if (_BodyIndexData[depthIndex] != 255)
                    {
                        isBody = true;
                        bodyPixelCount++;
                    }
                }
            }
            
            if (isBody)
            {
                // Keep the original color (RGBA format: R, G, B, A)
                int byteIndex = colorIndex * 4;
                colors[colorIndex] = new Color32(
                    _ColorData[byteIndex],     // R
                    _ColorData[byteIndex + 1], // G
                    _ColorData[byteIndex + 2], // B
                    255                         // A (fully opaque)
                );
            }
            else
            {
                // Background - make black/transparent
                colors[colorIndex] = new Color32(0, 0, 0, 255);
            }
        }
        
        texture.SetPixels32(colors);
        texture.Apply();
        
        Debug.Log($"CaptureBodyMask: Processed image - {bodyPixelCount} body pixels found");
        
        // Debug: Check sample color values
        int sampleCount = 0;
        for (int i = 0; i < colors.Length && sampleCount < 5; i++)
        {
            if (colors[i].r > 0 || colors[i].g > 0 || colors[i].b > 0)
            {
                Debug.Log($"Sample body pixel {sampleCount}: R={colors[i].r} G={colors[i].g} B={colors[i].b}");
                sampleCount++;
            }
        }
        if (sampleCount == 0)
        {
            // Check raw color data
            Debug.Log($"No colored pixels found! Checking raw _ColorData...");
            Debug.Log($"_ColorData length: {_ColorData.Length}");
            Debug.Log($"Sample raw bytes [0-11]: {_ColorData[0]}, {_ColorData[1]}, {_ColorData[2]}, {_ColorData[3]}, {_ColorData[4]}, {_ColorData[5]}, {_ColorData[6]}, {_ColorData[7]}, {_ColorData[8]}, {_ColorData[9]}, {_ColorData[10]}, {_ColorData[11]}");
        }
        
        // Checkpoint 3: Save to Desktop
        try
        {
            byte[] pngData = texture.EncodeToPNG();
            string path = System.Environment.GetFolderPath(System.Environment.SpecialFolder.Desktop) + "/BodyMask_Capture.png";
            Debug.Log($"CaptureBodyMask: Saving to {path}");
            System.IO.File.WriteAllBytes(path, pngData);
            Debug.Log($"CaptureBodyMask: SUCCESS! File saved to {path}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"CaptureBodyMask: Failed to save file - {e.Message}");
        }
        
        Destroy(texture);
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
        _TPoseHoldTimer = 0f;
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
    /// Measures body width at SpineMid joint.
    /// Returns width in meters, or 0 if measurement failed.
    /// </summary>
    public float MeasureSpineMidWidth()
    {
        if (_TrackedBody == null || !_TrackedBody.IsTracked)
        {
            return 0f;
        }
        
        var joint = _TrackedBody.Joints[JointType.SpineMid];
        return MeasureWidthAtJoint(joint);
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
        
        if (_BodyIndexData == null || _DepthData == null || _Mapper == null)
        {
            Debug.LogWarning("MeasureWidthAtJoint: Required data not available");
            return 0f;
        }
        
        // Convert joint position (3D camera space) to depth space (2D pixels)
        DepthSpacePoint depthPoint = _Mapper.MapCameraPointToDepthSpace(joint.Position);
        
        int centerX = (int)(depthPoint.X + 0.5f);
        int centerY = (int)(depthPoint.Y + 0.5f);
        
        // Check bounds
        if (centerX < 0 || centerX >= DEPTH_WIDTH || centerY < 0 || centerY >= DEPTH_HEIGHT)
        {
            Debug.LogWarning("MeasureWidthAtJoint: Joint position out of depth frame bounds");
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
            Debug.LogWarning("MeasureWidthAtJoint: Invalid depth value");
            return 0f;
        }
        
        // Convert pixels to meters
        float widthInMeters = PixelsToMeters(widthInPixels, depthMm);
        
        Debug.Log($"MeasureWidthAtJoint: center=({centerX},{centerY}), pixels={widthInPixels}, depth={depthMm}mm, width={widthInMeters:F3}m");
        
        return widthInMeters;
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

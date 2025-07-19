using UnityEngine;
using System.Collections;
using Kinect = Windows.Kinect;
using Vector3 = UnityEngine.Vector3;

// Data structures for scale computation
public struct DepthMappedCoordinates
{
    public Windows.Kinect.DepthSpacePoint ShoulderLeft;
    public Windows.Kinect.DepthSpacePoint ShoulderRight;
    public Windows.Kinect.DepthSpacePoint SpineMid;
    public Windows.Kinect.DepthSpacePoint HipLeft;
    public Windows.Kinect.DepthSpacePoint HipRight;
}

public struct BodyMeasurements
{
    public float ShoulderWidth;
    public float ChestWidth;
    public float HipWidth;
    public float TorsoHeight;
}

public class ScaleController : MonoBehaviour 
{
    public SyncedSourceManager _SyncedSourceManager;
    
    [Header("Debug")]
    public bool showDebugInfo = true;
    
    // Scale computation
    private float _ComputedScaleFactor = 1.0f;
    private bool _HasValidScale = false;
    private bool _IsCapturing = false;

    private GameObject Armature;
    private float BaseAvatarHeight;
    
    // Two-step scaling
    private float _HeightScaleFactor = 1.0f;
    private float _WidthScaleFactor = 1.0f;
    
    // Global captured frame data (single user)
    private ushort[] _CapturedDepthData = null;
    private Kinect.Body _CapturedTrackedBody = null;
    
    public float ScaleFactor => _ComputedScaleFactor;
    public bool HasValidScale => _HasValidScale;
    
    // Two-step scaling properties
    public float HeightScaleFactor => _HeightScaleFactor;
    public float WidthScaleFactor => _WidthScaleFactor;

    void Start()
    {
        _SyncedSourceManager = GetComponent<SyncedSourceManager>();

        if (_SyncedSourceManager == null)
        {
            Debug.LogError("SyncedSourceManager component not found.");
        }
    }
    // Update is called only once in the beginning
    void Update() 
    {
        // Manual capture trigger
        if (Input.GetKeyDown(KeyCode.Space))
        {
            CaptureScaleData();
        }
    }
    
    public void CaptureScaleData()
    {
        if (_SyncedSourceManager == null) return;
        
        // Get synchronized data from SyncedSourceManager
        var bodyData = _SyncedSourceManager.GetBodyData();
        var depthData = _SyncedSourceManager.GetDepthData();
        
        // Check if we have valid data
        if (bodyData == null || depthData == null) 
        {
            Debug.LogWarning("No data available");
            return;
        }
        
        // Check if we have a tracked body and store it globally
        Kinect.Body trackedBody = null;
        foreach (var body in bodyData)
        {
            if (body != null && body.IsTracked)
            {
                trackedBody = body;
                break;
            }
        }
        
        if (trackedBody != null && depthData.Length > 0)
        {
            // Store captured data globally for use in other functions (single user)
            _CapturedDepthData = depthData;
            _CapturedTrackedBody = trackedBody;
            
            ComputeScaleFactor();
            Debug.Log("Successfully captured frame!");
        }
        else
        {
            Debug.LogWarning("No tracked body found");
        }
    }

    private void ComputeScaleFactor()
    {        
        var depthData = GetCapturedDepthData();
        var trackedBody = GetCapturedTrackedBody();
        
        // Step 1: Map coordinate of body skeleton and depth data
        var mappedCoordinates = MapBodyToDepthCoordinates(trackedBody);
        
        // Step 2: User will be on T-Pose - Get the distance between those points and the edge of the human
        var bodyMeasurements = GetBodyMeasurementsFromDepthEdges(mappedCoordinates, depthData);
        
        // Step 3: Calculate scale factor based on measurements
        CalculateScaleFromMeasurements(bodyMeasurements);
    }
    
    // Simple scale calculation using basic height measurement (same as BodySourceView)
    public void ComputeBasicScaleFactor()
    {
        var trackedBody = GetCapturedTrackedBody();
        if (trackedBody == null) return;
        
        // Get key joints
        var joints = trackedBody.Joints;
        
        // Convert to Unity coordinates (same as BodySourceView)
        Vector3 head = GetVector3FromJoint(joints[Kinect.JointType.Head]);
        Vector3 footLeft = GetVector3FromJoint(joints[Kinect.JointType.FootLeft]);
        Vector3 footRight = GetVector3FromJoint(joints[Kinect.JointType.FootRight]);
        
        // Calculate user height (same logic as BodySourceView)
        float avgFootY = (footLeft.y + footRight.y) / 2f; // average foot height
        float userHeight = head.y - avgFootY;
        
        // Use a default base avatar height
        float baseAvatarHeight = 1.8f; // Default Unity units height
        
        // Calculate scale factor (same as BodySourceView)
        _ComputedScaleFactor = userHeight / baseAvatarHeight;
        // _HasValidScale = true;
        
        if (showDebugInfo)
        {
            Debug.Log($"ScaleController Basic: User height: {userHeight:F3}, Base height: {baseAvatarHeight:F3}, Scale factor: {_ComputedScaleFactor:F3}");
        }
    }
    
    // Helper: Convert joint to Unity coordinates (same as BodySourceView)
    private Vector3 GetVector3FromJoint(Kinect.Joint joint)
    {
        return new Vector3(joint.Position.X * 10, joint.Position.Y * 10, joint.Position.Z * 10);
    }
    
    // Step 1: Map body skeleton coordinates to depth image coordinates
    private DepthMappedCoordinates MapBodyToDepthCoordinates(Kinect.Body trackedBody)
    {
        var sensor = _SyncedSourceManager.GetSensor();
        var mapper = sensor.CoordinateMapper;
        
        var coordinates = new DepthMappedCoordinates();
        
        // Map key joints to depth space
        coordinates.ShoulderLeft = mapper.MapCameraPointToDepthSpace(trackedBody.Joints[Kinect.JointType.ShoulderLeft].Position);
        coordinates.ShoulderRight = mapper.MapCameraPointToDepthSpace(trackedBody.Joints[Kinect.JointType.ShoulderRight].Position);
        coordinates.SpineMid = mapper.MapCameraPointToDepthSpace(trackedBody.Joints[Kinect.JointType.SpineMid].Position);
        coordinates.HipLeft = mapper.MapCameraPointToDepthSpace(trackedBody.Joints[Kinect.JointType.HipLeft].Position);
        coordinates.HipRight = mapper.MapCameraPointToDepthSpace(trackedBody.Joints[Kinect.JointType.HipRight].Position);
        
        return coordinates;
    }
    

    // Step 2: Get body measurements by finding distance to depth edges
    private BodyMeasurements GetBodyMeasurementsFromDepthEdges(DepthMappedCoordinates joints, ushort[] depthData)
    {
        var measurements = new BodyMeasurements();
        var depthWidth = _SyncedSourceManager.DepthWidth;
        var depthHeight = _SyncedSourceManager.DepthHeight;
        
        // Get reference depth at spine center
        int spinePixelIndex = (int)(joints.SpineMid.Y * depthWidth + joints.SpineMid.X);
        ushort referenceDepth = depthData[spinePixelIndex];
        
        // T-Pose measurements: Find edges from center outward
        
        // 1. Shoulder width: for T-pose, measure arm thickness from shoulder center to edge
        float leftShoulderEdge = FindVerticalEdgePosition(joints.ShoulderLeft, depthData, depthWidth, depthHeight, referenceDepth, findTopEdge: true);
        float leftArmThickness = System.Math.Abs(leftShoulderEdge - joints.ShoulderLeft.Y); // Distance from shoulder center to arm edge

        // Double the arm thickness to get total arm width (assuming symmetric)
        measurements.ShoulderWidth = leftArmThickness * 2f;
        
        // 2. Chest width: scan left and right from spine chest
        float leftChestEdge = FindEdgePosition(joints.SpineMid, depthData, depthWidth, depthHeight, referenceDepth, findLeftEdge: true);
        float rightChestEdge = FindEdgePosition(joints.SpineMid, depthData, depthWidth, depthHeight, referenceDepth, findLeftEdge: false);
        float chestThickness = System.Math.Abs(leftChestEdge - rightChestEdge);
        measurements.ChestWidth = chestThickness;
        
        // 3. Hip width: measure distance between left and right hips
        float leftHipEdge = FindEdgePosition(joints.HipLeft, depthData, depthWidth, depthHeight, referenceDepth, findLeftEdge: true);
        float leftHipThickness = System.Math.Abs(leftHipEdge - joints.HipLeft.Y);

        measurements.HipWidth = hipThickness * 2f;

        
        return measurements;
    }
    
    // Helper: Find edge position (left or right) from center point
    private int FindEdgePosition(Windows.Kinect.DepthSpacePoint center, ushort[] depthData, int width, int height, ushort referenceDepth, bool findLeftEdge)
    {
        int centerX = (int)center.X;
        int centerY = (int)center.Y;
        int threshold = 200; // mm depth 
        
        if (findLeftEdge)
        {
            // Find left edge
            for (int x = centerX; x >= 0; x--)
            {
                int pixelIndex = centerY * width + x;
                if (pixelIndex < 0 || pixelIndex >= depthData.Length) break;
                
                ushort currentDepth = depthData[pixelIndex];
                if (currentDepth == 0) continue; // Skip invalid depth values
                if (System.Math.Abs(currentDepth - referenceDepth) > threshold)
                {
                    return x;
                }
            }
            if (showDebugInfo)
            {
                Debug.LogWarning($"ScaleController: Left edge not found at center ({centerX}, {centerY})");
            }
            return 0; // Edge not found, return image boundary
        }
        else
        {
            // Find right edge
            for (int x = centerX; x < width; x++)
            {
                int pixelIndex = centerY * width + x;
                if (pixelIndex >= depthData.Length) break;
                
                ushort currentDepth = depthData[pixelIndex];
                if (currentDepth == 0) continue; // Skip invalid depth values
                if (System.Math.Abs(currentDepth - referenceDepth) > threshold)
                {
                    return x;
                }
            }
            if (showDebugInfo)
            {
                Debug.LogWarning($"ScaleController: Right edge not found at center ({centerX}, {centerY})");
            }
            return width - 1; // Edge not found, return image boundary
        }
    }
    
    // Helper: Find vertical edge position (top or bottom) from center point
    private int FindVerticalEdgePosition(Windows.Kinect.DepthSpacePoint center, ushort[] depthData, int width, int height, ushort referenceDepth, bool findTopEdge)
    {
        int centerX = (int)center.X;
        int centerY = (int)center.Y;
        int threshold = 200; // mm depth 
        
        if (findTopEdge)
        {
            // Find top edge
            for (int y = centerY; y >= 0; y--)
            {
                int pixelIndex = y * width + centerX;
                if (pixelIndex < 0 || pixelIndex >= depthData.Length) break;
                
                ushort currentDepth = depthData[pixelIndex];
                if (currentDepth == 0) continue; // Skip invalid depth values
                if (System.Math.Abs(currentDepth - referenceDepth) > threshold)
                {
                    return y;
                }
            }
            if (showDebugInfo)
            {
                Debug.LogWarning($"ScaleController: Top edge not found at center ({centerX}, {centerY})");
            }
            return 0; // Edge not found, return image boundary
        }
        else
        {
            // Find bottom edge
            for (int y = centerY; y < height; y++)
            {
                int pixelIndex = y * width + centerX;
                if (pixelIndex >= depthData.Length) break;
                
                ushort currentDepth = depthData[pixelIndex];
                if (currentDepth == 0) continue; // Skip invalid depth values
                if (System.Math.Abs(currentDepth - referenceDepth) > threshold)
                {
                    return y;
                }
            }
            if (showDebugInfo)
            {
                Debug.LogWarning($"ScaleController: Bottom edge not found at center ({centerX}, {centerY})");
            }
            return height - 1; // Edge not found, return image boundary
        }
    }
    
    // Step 3: Calculate final scale factor from measurements
    private void CalculateScaleFromMeasurements(BodyMeasurements measurements)
    {
       
    }
    
    // Compute width scale factor using measured arm and chest width
    public float ComputeWidthScaleFactor(float heightScaleFactor)
    {
        if (!HasCapturedData())
        {
            Debug.LogWarning("ScaleController: No captured data for width scale calculation");
            return 1.0f;
        }
        
        // Get measurements from captured data
        var depthData = GetCapturedDepthData();
        var trackedBody = GetCapturedTrackedBody();
        var mappedCoordinates = MapBodyToDepthCoordinates(trackedBody);
        var measurements = GetBodyMeasurementsFromDepthEdges(mappedCoordinates, depthData);
        
        // Base avatar measurements (in Unity units)
        float baseAvatarArmWidth = 0.3f;    // Default avatar arm thickness
        float baseAvatarChestWidth = 1.2f;   // Default avatar chest width
        
        // Convert pixel measurements to Unity units (rough conversion)
        float userArmWidth = measurements.ShoulderWidth * 0.002f;   // Pixels to Unity units
        float userChestWidth = measurements.ChestWidth * 0.002f;    // Pixels to Unity units
        
        // Calculate what the avatar widths should be after height scaling
        float scaledAvatarArmWidth = baseAvatarArmWidth * heightScaleFactor;
        float scaledAvatarChestWidth = baseAvatarChestWidth * heightScaleFactor;
        
        // Calculate width scale factors
        float armWidthScale = userArmWidth / scaledAvatarArmWidth;
        float chestWidthScale = userChestWidth / scaledAvatarChestWidth;
        
        // Average the two width scales for final result
        float finalWidthScale = (armWidthScale + chestWidthScale) / 2f;
        
        if (showDebugInfo)
        {
            Debug.Log($"ScaleController Width: User arm: {userArmWidth:F3}, chest: {userChestWidth:F3}");
            Debug.Log($"ScaleController Width: Scaled avatar arm: {scaledAvatarArmWidth:F3}, chest: {scaledAvatarChestWidth:F3}");
            Debug.Log($"ScaleController Width: Arm scale: {armWidthScale:F3}, chest scale: {chestWidthScale:F3}");
            Debug.Log($"ScaleController Width: Final width scale: {finalWidthScale:F3}");
        }
        
        return finalWidthScale;
    }
    
    private void GetBaseAvatarHeight()
    {
        Transform headTop = Armature.transform.FindDeepChild("mixamorig:HeadTop_End");
        Transform footLeft = Armature.transform.FindDeepChild("mixamorig:LeftToeBase");
        Transform footRight = Armature.transform.FindDeepChild("mixamorig:RightToeBase");

        if (headTop == null || footLeft == null || footRight == null)
        {
            Debug.LogError("Could not find one or more required bones for avatar height calculation.");
            return 0f; // fallback to no scaling
        }

        float footAvgY = (footLeft.position.y + footRight.position.y) / 2f;
        return headTop.position.y - footAvgY;
    }
    // private float GetBaseAvatarWidth()
    // {
    //     // You need to pass the avatar/armature reference here
    //     // For now, assuming you'll add it as a parameter or class field
        
    //     // Option 1: Shoulder span (most common for width)
    //     Transform leftShoulder = Armature.transform.FindDeepChild("mixamorig:LeftShoulder");
    //     Transform rightShoulder = Armature.transform.FindDeepChild("mixamorig:RightShoulder");
        
    //     if (leftShoulder != null && rightShoulder != null)
    //     {
    //         float shoulderWidth = Vector3.Distance(leftShoulder.position, rightShoulder.position);
    //         Debug.Log($"Avatar shoulder width: {shoulderWidth:F3}");
    //         return shoulderWidth;
    //     }
        
    //     // Option 2: Chest/torso width (alternative measurement)
    //     Transform spine = Armature.transform.FindDeepChild("mixamorig:Spine");
    //     Transform leftArm = Armature.transform.FindDeepChild("mixamorig:LeftArm");
    //     Transform rightArm = Armature.transform.FindDeepChild("mixamorig:RightArm");
        
    //     if (leftArm != null && rightArm != null)
    //     {
    //         float armSpan = Vector3.Distance(leftArm.position, rightArm.position);
    //         Debug.Log($"Avatar arm span: {armSpan:F3}");
    //         return armSpan;
    //     }
        
    //     Debug.LogError("Could not find avatar shoulder or arm bones for width calculation.");
    //     return 1.5f; // fallback value
    // }
    
    // // Get avatar arm thickness (for comparison with user's arm thickness)
    // private float GetBaseAvatarArmThickness()
    // {
    //     Transform leftShoulder = Armature.transform.FindDeepChild("mixamorig:LeftShoulder");
    //     Transform leftArm = Armature.transform.FindDeepChild("mixamorig:LeftArm");
    //     Transform leftForeArm = Armature.transform.FindDeepChild("mixamorig:LeftForeArm");
        
    //     if (leftShoulder != null && leftArm != null && leftForeArm != null)
    //     {
    //         // Estimate arm thickness based on bone structure
    //         // This is an approximation since bones don't directly give thickness
    //         float upperArmLength = Vector3.Distance(leftShoulder.position, leftArm.position);
    //         float foreArmLength = Vector3.Distance(leftArm.position, leftForeArm.position);
            
    //         // Rough estimate: arm thickness is typically 1/10th of arm length
    //         float avgArmLength = (upperArmLength + foreArmLength) / 2f;
    //         float estimatedThickness = avgArmLength * 0.1f;
            
    //         Debug.Log($"Avatar estimated arm thickness: {estimatedThickness:F3}");
    //         return estimatedThickness;
    //     }
        
    //     Debug.LogError("Could not find avatar arm bones for thickness calculation.");
    //     return 0.3f; // fallback value
    // }

    // Helper methods to access global captured data (single user)
    public ushort[] GetCapturedDepthData() => _CapturedDepthData;
    public Kinect.Body GetCapturedTrackedBody() => _CapturedTrackedBody;
    
    // Check if we have valid captured data
    public bool HasCapturedData() => _CapturedDepthData != null && _CapturedTrackedBody != null;


}

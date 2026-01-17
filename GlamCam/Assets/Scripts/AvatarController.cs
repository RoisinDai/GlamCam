using System.Linq;
using UnityEngine;
using Kinect = Windows.Kinect;
using Vector3 = UnityEngine.Vector3;
using System;
using System.Collections.Generic;

class HumanoidMeasurements
{
    public float height;
    public float upperArmLength; // Shoulder to elbow
    public float lowerArmLength; // Elbow to wrist
    public float upperLegLength; // Hip to knee
    public float lowerLegLength; // Knee to foot
    
    // Silhouette-based width measurement (from BodyIndex)
    public float spineMidWidth;  // Body width at SpineMid (belly)
}

// Used for non-uniform scaling of legs/arms
class ExtensionFactors
{
    public float upperArmExtensionFactor = 0;
    public float lowerArmExtensionFactor = 0;
    public float upperLegExtensionFactor = 0;
    public float lowerLegExtensionFactor = 0;
}

// Responsible for controlling the clothed base avatar, making it track the user's body
public class AvatarController : MonoBehaviour
{
  // Unity Objects
    public Animator animator;
    public GameObject BodySourceManager;
    private static BodySourceManager _BodyManager;
    public GameObject ClothedBaseAvatar;
    private GameObject BaseAvatar; // The unclothed base avatar
    public static Kinect.Body trackedBody; // The body being tracked by the avatar
    private bool _HideAvatar = false; // Flag to toggle BaseAvatar visibility (show only clothes)

    // Inverse Kinematics Variables
    public bool enableInverseKinematics = true;

    // User Measurement Variables
    private HumanoidMeasurements _KinectUserMeasurements = new();

    // Clothed Base Avatar Measurement Variables
    private HumanoidMeasurements _AvatarMeasurements = new();

    // Scaling factor variables
    private float UniformScaleFactor = -1f;
    private ExtensionFactors _ExtensionFactors = new();
    private bool hasValidBody = false;

  void Start()
  {
    print("AvatarController Start called for " + ClothedBaseAvatar.name);
    animator = GetComponent<Animator>();
    if (animator == null)
    {
      Debug.LogError(ClothedBaseAvatar.name + " Animator component not found on AvatarController.");
    }

    _BodyManager = BodySourceManager.GetComponent<BodySourceManager>();
    if (_BodyManager == null)
    {
      Debug.LogError(ClothedBaseAvatar.name + " BodySourceManager component not found.");
    }

    // Get the measurements of the ClothedBaseAvatar
        GetAvatarHeight();
    // Debug.Log("_AvatarMeasurements - Height: " + _AvatarMeasurements.height);
    // Debug.Log("_AvatarMeasurements - Upper Arm Length: " + _AvatarMeasurements.upperArmLength);
    // Debug.Log("_AvatarMeasurements - Lower Arm Length: " + _AvatarMeasurements.lowerArmLength);
    // Debug.Log("_AvatarMeasurements - Upper Leg Length: " + _AvatarMeasurements.upperLegLength);
    // Debug.Log("_AvatarMeasurements - Lower Leg Length: " + _AvatarMeasurements.lowerLegLength);

    // Hide the BaseAvatar if desired
    if (_HideAvatar)
    {
      BaseAvatar = ClothedBaseAvatar.transform.GetChild(2).gameObject;
      if (BaseAvatar != null)
      {
        // Debug.Log(ClothedBaseAvatar.name + " Hiding the base avatar (leaving clothes).");
        BaseAvatar.SetActive(false); // Hide the BaseAvatar
      }
      else
      {
        Debug.LogError(ClothedBaseAvatar.name + " BaseAvatar not found in ClothedBaseAvatar hierarchy.");
      }
    }
  }

  void Update()
  {
        hasValidBody = false;

    if (_BodyManager == null) return;

    Kinect.Body[] data = _BodyManager.GetData();
    if (data == null) return;

    trackedBody = data.FirstOrDefault(b => b != null && b.IsTracked);
    if (trackedBody == null) return;

        if (animator != null)
        {
            animator.enabled = false; // Disables the Animator temporarily
            Debug.Log("Animator has been disabled for manual bone control.");
        }

        // Cache joints we need later
        var joints = trackedBody.Joints;

        // Height measurement (data only)
        Vector3 head = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.Head]);
        Vector3 footL = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.FootLeft]);
        Vector3 footR = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.FootRight]);

        _KinectUserMeasurements.height =
            head.y - ((footL.y + footR.y) * 0.5f);

        hasValidBody = true;
        CaptureBodyWidthMeasurements();

    }

    void LateUpdate()
    {
        if (!hasValidBody || trackedBody == null) return;

        // 1. Apply forward kinematics (bone rotations)
        ApplyForwardKinematics(trackedBody);

        // 2. Rotate avatar root from shoulders
        RotateAvatarBasedOnShoulders(
            trackedBody.Joints[Kinect.JointType.ShoulderLeft],
            trackedBody.Joints[Kinect.JointType.ShoulderRight]
        );
        var joints = trackedBody.Joints;

        // 3. Move avatar root to spine base
        Vector3 spineBasePos = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.SpineBase]);
        ClothedBaseAvatar.transform.position = new Vector3(spineBasePos.x, spineBasePos.y, spineBasePos.z);

        // 4. Apply uniform scaling ONCE
        if (UniformScaleFactor < 0f && _AvatarMeasurements.height > 0f)
        {
            UniformScaleFactor =
                _KinectUserMeasurements.height / _AvatarMeasurements.height;

            ClothedBaseAvatar.transform.localScale =
                Vector3.one * UniformScaleFactor;

            Debug.Log($"UniformScaleFactor set to {UniformScaleFactor:F3}");
        }

        // 5. Optional: shoulder-based translation correction
        ApplyUniformShoulderTranslation();
    }

    private void ApplyUniformShoulderTranslation()
    {
        var joints = trackedBody.Joints;
        Transform modelShoulderLeft =
            animator.GetBoneTransform(HumanBodyBones.LeftUpperArm);
        Transform modelShoulderRight =
            animator.GetBoneTransform(HumanBodyBones.RightUpperArm);

        if (modelShoulderLeft == null || modelShoulderRight == null)
            return;

        Vector3 kinectLeft =
            BodySourceView.GetVector3FromJoint(trackedBody.Joints[Kinect.JointType.ShoulderLeft]);
        Vector3 kinectRight =
            BodySourceView.GetVector3FromJoint(trackedBody.Joints[Kinect.JointType.ShoulderRight]);

        Vector3 modelLeft = modelShoulderLeft.position;
        Vector3 modelRight = modelShoulderRight.position;

        Vector3 delta =
            (kinectLeft - modelRight + (kinectRight - modelLeft)) * 0.5f;
        
        Vector3 spineBasePos = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.SpineBase]);
        Vector3 newPosition = spineBasePos + delta;
        ClothedBaseAvatar.transform.position = new Vector3(newPosition.x, newPosition.y, newPosition.z);
    }

    private void RotateAvatarBasedOnShoulders(Kinect.Joint leftShoulder, Kinect.Joint rightShoulder)
    {
        // Get positions of the shoulders in Unity coordinates
        Vector3 shoulderLeft = BodySourceView.GetVector3FromKinectCoord(
          leftShoulder.Position.X,
          leftShoulder.Position.Y,
          leftShoulder.Position.Z
        );

        Vector3 shoulderRight = BodySourceView.GetVector3FromKinectCoord(
          rightShoulder.Position.X,
          rightShoulder.Position.Y,
          rightShoulder.Position.Z
        );

        // Vector pointing from left shoulder to right shoulder
        Vector3 shoulderDirection = shoulderRight - shoulderLeft;

        // The forward direction is perpendicular to the shoulder vector in the horizontal plane
        Vector3 forward = Vector3.Cross(shoulderDirection, Vector3.up);

        // Optional: smooth the rotation
        Quaternion targetRotation = Quaternion.LookRotation(-1 * forward /*Avatar faces Z-*/, Vector3.up);
        ClothedBaseAvatar.transform.rotation = Quaternion.Slerp(
            ClothedBaseAvatar.transform.rotation,
            targetRotation,
            Time.deltaTime * 5f // smoothing speed
        );
    }

    // Get avatar height from head to feet
    void GetAvatarHeight()
    {
        Transform headTop = animator.GetBoneTransform(HumanBodyBones.Head);
        Transform footLeft = animator.GetBoneTransform(HumanBodyBones.LeftFoot);
        Transform footRight = animator.GetBoneTransform(HumanBodyBones.RightFoot);

        if (headTop == null || footLeft == null || footRight == null)
        {
            Debug.LogError("AvatarMeasurement: Could not find bones for height calculation.");
        }

        float footAvgY = (footLeft.position.y + footRight.position.y) / 2f;
        float height = headTop.position.y - footAvgY;

        // Debug.Log($"AvatarMeasurement: Height = {height:F3} Unity units");
        _AvatarMeasurements.height = height;
    }

    private void ApplyForwardKinematics(Kinect.Body body)
    {
        if (body == null) return;

        // Define Kinect bone mapping
        var boneMapping = new List<(Kinect.JointType Start, Kinect.JointType End, HumanBodyBones UnityBone)>
        {
            (Kinect.JointType.ElbowLeft, Kinect.JointType.WristLeft, HumanBodyBones.RightLowerArm),
            (Kinect.JointType.ShoulderLeft, Kinect.JointType.ElbowLeft, HumanBodyBones.RightUpperArm),
            (Kinect.JointType.KneeLeft, Kinect.JointType.AnkleLeft, HumanBodyBones.RightLowerLeg),
            (Kinect.JointType.HipLeft, Kinect.JointType.KneeLeft, HumanBodyBones.RightUpperLeg),

            (Kinect.JointType.ElbowRight, Kinect.JointType.WristRight, HumanBodyBones.LeftLowerArm),
            (Kinect.JointType.ShoulderRight, Kinect.JointType.ElbowRight, HumanBodyBones.LeftUpperArm),
            (Kinect.JointType.KneeRight, Kinect.JointType.AnkleRight, HumanBodyBones.LeftLowerLeg),
            (Kinect.JointType.HipRight, Kinect.JointType.KneeRight, HumanBodyBones.LeftUpperLeg),
        };

        foreach (var (Start, End, UnityBone) in boneMapping)
        {
            // Joint data from Kinect
            Kinect.Joint startJoint = body.Joints[Start];
            Kinect.Joint endJoint = body.Joints[End];

            // Get Unity bone
            Transform unityBoneTransform = animator.GetBoneTransform(UnityBone);
            if (unityBoneTransform == null) continue;

            // Convert Kinect positions to Unity space
            Vector3 startPos = BodySourceView.GetVector3FromKinectCoord(
                startJoint.Position.X, startJoint.Position.Y, startJoint.Position.Z
            );
            Vector3 endPos = BodySourceView.GetVector3FromKinectCoord(
                endJoint.Position.X, endJoint.Position.Y, endJoint.Position.Z
            );
            Vector3 boneDirection = (startPos - endPos).normalized;

            // Adjust for coordinate system misalignment (optional fine-tuning required for your rig specifics)
            boneDirection = new Vector3(boneDirection.x, boneDirection.y, boneDirection.z);

            // Define T-pose offset if needed
            Quaternion tPoseOffset = Quaternion.Euler(-90, 0, 0);

            // Compute world rotation with pose offset
            Quaternion worldRotation = Quaternion.LookRotation(boneDirection, Vector3.up) * tPoseOffset;

            // Transform world rotation to local rotation for the bone
            if (unityBoneTransform.parent != null)
            {
                unityBoneTransform.localRotation = Quaternion.Inverse(unityBoneTransform.parent.rotation) * worldRotation;
            }
            else
            {
                unityBoneTransform.localRotation = worldRotation;
            }
        }
    }

    // =====================================================
    // Width Measurement Methods (using silhouette)
    // =====================================================
    
    private const int DEPTH_WIDTH = 512;
    private const int DEPTH_HEIGHT = 424;
    private const float DEPTH_HORIZONTAL_FOV = 70.6f; // Kinect v2 depth camera horizontal FOV in degrees
    
    /// <summary>
    /// Captures body width measurement at SpineMid.
    /// Call this when T-pose is detected.
    /// </summary>
    public void CaptureBodyWidthMeasurements()
    {
        if (MultiSourceManager == null)
        {
            Debug.LogError("CaptureBodyWidthMeasurements: MultiSourceManager not assigned!");
            return;
        }
        
        if (trackedBody == null || !trackedBody.IsTracked)
        {
            Debug.LogError("CaptureBodyWidthMeasurements: No tracked body available!");
            return;
        }
        
        var mapper = MultiSourceManager.GetCoordinateMapper();
        var bodyIndexData = MultiSourceManager.GetBodyIndexData();
        var depthData = MultiSourceManager.GetDepthData();
        
        if (mapper == null || bodyIndexData == null || depthData == null)
        {
            Debug.LogError("CaptureBodyWidthMeasurements: Required data not available from MultiSourceManager!");
            return;
        }
        
        var joints = trackedBody.Joints;
        
        // Measure width at SpineMid (belly)
        _KinectUserMeasurements.spineMidWidth = MeasureWidthAtJoint(
            joints[Kinect.JointType.SpineMid], mapper, bodyIndexData, depthData);
        
        Debug.Log($"=== Body Measurements (all in meters) ===");
        Debug.Log($"  Height: {_KinectUserMeasurements.height:F3} m");
        Debug.Log($"  SpineMid width: {_KinectUserMeasurements.spineMidWidth:F3} m");
    }
    
    /// <summary>
    /// Measures the body width at a specific joint by scanning the silhouette horizontally.
    /// </summary>
    private float MeasureWidthAtJoint(Kinect.Joint joint, Kinect.CoordinateMapper mapper, 
                                       byte[] bodyIndexData, ushort[] depthData)
    {
        if (joint.TrackingState == Kinect.TrackingState.NotTracked)
        {
            Debug.LogWarning($"MeasureWidthAtJoint: Joint not tracked");
            return 0f;
        }
        
        // Convert joint position (3D camera space) to depth space (2D pixels)
        Kinect.DepthSpacePoint depthPoint = mapper.MapCameraPointToDepthSpace(joint.Position);
        
        int centerX = (int)(depthPoint.X + 0.5f);
        int centerY = (int)(depthPoint.Y + 0.5f);
        
        // Check bounds
        if (centerX < 0 || centerX >= DEPTH_WIDTH || centerY < 0 || centerY >= DEPTH_HEIGHT)
        {
            Debug.LogWarning($"MeasureWidthAtJoint: Joint position out of depth frame bounds");
            return 0f;
        }
        
        // Scan LEFT from center to find left edge
        int leftEdge = centerX;
        for (int x = centerX; x >= 0; x--)
        {
            int index = centerY * DEPTH_WIDTH + x;
            if (bodyIndexData[index] == 255) // Not a body pixel
            {
                leftEdge = x + 1; // Last body pixel was x+1
                break;
            }
            if (x == 0) leftEdge = 0; // Reached edge of frame
        }
        
        // Scan RIGHT from center to find right edge
        int rightEdge = centerX;
        for (int x = centerX; x < DEPTH_WIDTH; x++)
        {
            int index = centerY * DEPTH_WIDTH + x;
            if (bodyIndexData[index] == 255) // Not a body pixel
            {
                rightEdge = x - 1; // Last body pixel was x-1
                break;
            }
            if (x == DEPTH_WIDTH - 1) rightEdge = DEPTH_WIDTH - 1; // Reached edge of frame
        }
        
        int widthInPixels = rightEdge - leftEdge + 1;
        
        // Get depth at the center point (in millimeters)
        int centerIndex = centerY * DEPTH_WIDTH + centerX;
        float depthMm = depthData[centerIndex];
        
        if (depthMm <= 0)
        {
            Debug.LogWarning($"MeasureWidthAtJoint: Invalid depth value at joint");
            return 0f;
        }
        
        // Convert pixels to meters
        float widthInMeters = PixelsToMeters(widthInPixels, depthMm);
        
        Debug.Log($"MeasureWidthAtJoint: center=({centerX},{centerY}), left={leftEdge}, right={rightEdge}, " +
                  $"pixels={widthInPixels}, depth={depthMm}mm, width={widthInMeters:F3}m");
        
        return widthInMeters;
    }
    
    /// <summary>
    /// Converts a horizontal pixel distance to real-world meters at a given depth.
    /// Uses the Kinect depth camera's horizontal FOV.
    /// </summary>
    private float PixelsToMeters(int pixels, float depthMm)
    {
        // Calculate the real-world width of the entire depth frame at this depth
        // Using: width = 2 * depth * tan(FOV/2)
        float depthM = depthMm / 1000f;
        float halfFovRad = (DEPTH_HORIZONTAL_FOV / 2f) * Mathf.Deg2Rad;
        float frameWidthAtDepth = 2f * depthM * Mathf.Tan(halfFovRad);
        
        // Each pixel represents frameWidth / DEPTH_WIDTH meters
        float metersPerPixel = frameWidthAtDepth / DEPTH_WIDTH;
        
        return pixels * metersPerPixel;
    }
}
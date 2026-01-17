using System.Linq;
using UnityEngine;
using Kinect = Windows.Kinect;
using Vector3 = UnityEngine.Vector3;
using System;

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
    public MultiSourceManager MultiSourceManager; // For BodyIndex data
    public GameObject ClothedBaseAvatar;
    private GameObject Armature;
    private GameObject BaseAvatar; // The unclothed base avatar
    public static Kinect.Body trackedBody; // The body being tracked by the avatar
    private bool _HideAvatar = false; // Flag to toggle BaseAvatar visibility (show only clothes)

    // Inverse Kinematics Variables
    public bool enableInverseKinematics = true;
    private readonly float IK_HANDS_WEIGHT = 1f;
    private readonly float IK_FEET_WEIGHT = 1f;
    private readonly float IK_HEAD_DIRECTION_WEIGHT = 1f;
    private readonly float IK_DEFAULT_WEIGHT = 1f;

    // User Measurement Variables
    private HumanoidMeasurements _UserMeasurements = new();

    // Clothed Base Avatar Measurement Variables
    private HumanoidMeasurements _AvatarMeasurements = new();

    // Scaling factor variables
    private float UniformScaleFactor = -1f;
    private ExtensionFactors _ExtensionFactors = new();

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
    Armature = ClothedBaseAvatar.transform.GetChild(0).gameObject;
    GetAvatarHeight(Armature);
    GetAvatarArmLengths(Armature);
    GetAvatarLegLength(Armature);
    // Debug.Log("_AvatarMeasurements - Height: " + _AvatarMeasurements.height);
    // Debug.Log("_AvatarMeasurements - Upper Arm Length: " + _AvatarMeasurements.upperArmLength);
    // Debug.Log("_AvatarMeasurements - Lower Arm Length: " + _AvatarMeasurements.lowerArmLength);
    // Debug.Log("_AvatarMeasurements - Upper Leg Length: " + _AvatarMeasurements.upperLegLength);
    // Debug.Log("_AvatarMeasurements - Lower Leg Length: " + _AvatarMeasurements.lowerLegLength);

    // Scale up the shirt a bit (hardcoded for now)
    GameObject shirt = ClothedBaseAvatar.transform.GetChild(1).gameObject;
    var ShirtUniformScaleFactor = 1.2f; // Scale factor for the shirt
    shirt.transform.localScale = new Vector3(ShirtUniformScaleFactor, ShirtUniformScaleFactor, ShirtUniformScaleFactor);

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

  // Updates the body object currently being tracked
  void Update()
  {
    // Debug.Log(ClothedBaseAvatar.name + " Update called.");
    if (_BodyManager == null) return;

    // Get the bodies
    Kinect.Body[] data = _BodyManager.GetData();
    if (data == null) return;

    // Use the first tracked body
    trackedBody = data.FirstOrDefault(b => b != null && b.IsTracked);
    if (trackedBody == null) return;

    // Get the joints map
    var joints = trackedBody.Joints;

    // Move the avatar to the location of the skeleton
    Vector3 spineBasePos = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.SpineBase]);
    ClothedBaseAvatar.transform.position = new Vector3(spineBasePos.x, spineBasePos.y + 1.7f /*Align shoulders, prevent sagging knees*/, spineBasePos.z);

    // Get joints of interest
    Vector3 head = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.Head]);
    Vector3 footLeft = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.FootLeft]);
    Vector3 footRight = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.FootRight]);

    Vector3 shoulderLeft = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.ShoulderLeft]);
    Vector3 elbowLeft = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.ElbowLeft]);
    Vector3 wristLeft = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.WristLeft]);

    Vector3 hipLeft = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.HipLeft]);
    Vector3 kneeLeft = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.KneeLeft]);
    Vector3 ankleLeft = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.AnkleLeft]);

    // First uniformly scale based on height
    _UserMeasurements.height = head.y - ((footLeft.y + footRight.y) / 2f);
    if (UniformScaleFactor == -1f)
    {
      // Set uniform scaling factor once
      UniformScaleFactor = (_UserMeasurements.height / _AvatarMeasurements.height) + 0.5f; // AvatarHeight * scaleFactor = UserHeight
      // UniformScaleFactor = 2.6f; // For testing purposes
      ClothedBaseAvatar.transform.localScale = new Vector3(UniformScaleFactor, UniformScaleFactor, UniformScaleFactor);
      // Update avatar's measurements after uniform scaling
      GetAvatarHeight(Armature);
      GetAvatarArmLengths(Armature);
      GetAvatarLegLength(Armature);
    }

    // Scale arms
    _UserMeasurements.upperArmLength = Vector3.Distance(shoulderLeft, elbowLeft);
    _UserMeasurements.lowerArmLength = Vector3.Distance(elbowLeft, wristLeft);
    // Debug.Log("_TESTING: Uniform Scale Factor: " + UniformScaleFactor);
    // Debug.Log("_TESTING Height: Avatar:" + _AvatarMeasurements.height + "  User:" + _UserMeasurements.height);
    // Debug.Log("_TESTING Upper Arm Length: Avatar: " + _AvatarMeasurements.upperArmLength + "  User: " + _UserMeasurements.upperArmLength);
    // Debug.Log("_TESTING Lower Arm Length: Avatar: " + _AvatarMeasurements.lowerArmLength + "  User: " + _UserMeasurements.lowerArmLength);
    _ExtensionFactors.upperArmExtensionFactor = (_UserMeasurements.upperArmLength - _AvatarMeasurements.upperArmLength);
    _ExtensionFactors.lowerArmExtensionFactor = (_UserMeasurements.lowerArmLength - _AvatarMeasurements.lowerArmLength);
    // Debug.Log("_TESTING Upper Arm Ext Factor: " + _ExtensionFactors.upperArmExtensionFactor);
    // Debug.Log("_TESTING Lower Arm Ext Factor: " + _ExtensionFactors.lowerArmExtensionFactor);

    // Scale legs
    _UserMeasurements.upperLegLength = Vector3.Distance(hipLeft, kneeLeft);
    _UserMeasurements.lowerLegLength = Vector3.Distance(kneeLeft, ankleLeft);
    // Debug.Log("_TESTING Upper Leg Length: Avatar: " + _AvatarMeasurements.upperLegLength + "  User: " + _UserMeasurements.upperLegLength);
    // Debug.Log("_TESTING Lower Leg Length: Avatar: " + _AvatarMeasurements.lowerLegLength + "  User: " + _UserMeasurements.lowerLegLength);
    _ExtensionFactors.upperLegExtensionFactor = (_UserMeasurements.upperLegLength - _AvatarMeasurements.upperLegLength);
    _ExtensionFactors.lowerLegExtensionFactor = (_UserMeasurements.lowerLegLength - _AvatarMeasurements.lowerLegLength);
    // Debug.Log("_TESTING Upper Leg Ext Factor: " + _ExtensionFactors.upperLegExtensionFactor);
    // Debug.Log(ClothedBaseAvatar.name + "_TESTING Lower Leg Ext Factor: " + _ExtensionFactors.lowerLegExtensionFactor);
    
    // Press T to capture body width measurements (for testing)
    if (Input.GetKeyDown(KeyCode.T))
    {
        Debug.Log("T key pressed - capturing body measurements...");
        CaptureBodyWidthMeasurements();
    }
  }

    // A callback function to calculate inverse kinematics
    private void OnAnimatorIK(int layerIndex)
    {
        // Debug.Log(ClothedBaseAvatar.name + "OnAnimatorIK called with layer: " + layerIndex);
        if (animator == null || trackedBody == null || !enableInverseKinematics)
        {
            // Debug.Log(ClothedBaseAvatar.name + " But no trackedBody or inverseKinematics disabled!");
            return;
        }

        // NOTE: Kinect uses camera-facing perspective, meaning its left is the avatar's right
        //       Meanwhile, Unity’s AvatarIKGoal.LeftHand refers to the avatar’s anatomical left

        // Move hands to their goals
        ApplyIKGoal(Kinect.JointType.HandTipRight, AvatarIKGoal.LeftHand);
        ApplyIKGoal(Kinect.JointType.HandTipLeft, AvatarIKGoal.RightHand);

        // Move feet to their goals
        ApplyIKGoal(Kinect.JointType.FootRight, AvatarIKGoal.LeftFoot);
        ApplyIKGoal(Kinect.JointType.FootLeft, AvatarIKGoal.RightFoot);

        // Move elbows to their hints
        ApplyIKHint(Kinect.JointType.ElbowRight, AvatarIKHint.LeftElbow);
        ApplyIKHint(Kinect.JointType.ElbowLeft, AvatarIKHint.RightElbow);

        // Move knees to their hints
        ApplyIKHint(Kinect.JointType.KneeRight, AvatarIKHint.LeftKnee);
        ApplyIKHint(Kinect.JointType.KneeLeft, AvatarIKHint.RightKnee);

        // Rotate head to look at a specific position
        var kinectHead = trackedBody.Joints[Kinect.JointType.Head];
        if (kinectHead.TrackingState == Kinect.TrackingState.Tracked)
        {
            Vector3 headTarget = BodySourceView.GetVector3FromKinectCoord(0, 0, 0); // Default position

            animator.SetLookAtWeight(IK_HEAD_DIRECTION_WEIGHT);
            animator.SetLookAtPosition(headTarget);
        }

        // Rotate avatar based on the orientation of the shoulders
        RotateAvatarBasedOnShoulders(trackedBody.Joints[Kinect.JointType.ShoulderLeft],
                                     trackedBody.Joints[Kinect.JointType.ShoulderRight]);
    }


    // Apply inverse kinematics to the avatar's joints to move them to the goal
    private void ApplyIKGoal(Kinect.JointType joint, AvatarIKGoal goal)
    {
        var kinectJoint = trackedBody.Joints[joint];
        if (kinectJoint.TrackingState == Kinect.TrackingState.NotTracked) return; // Avoid phantom movements

        var kinectJointPos = trackedBody.Joints[joint].Position;
        Vector3 unityPos = BodySourceView.GetVector3FromKinectCoord(kinectJointPos.X, kinectJointPos.Y, kinectJointPos.Z);

        if (kinectJoint.JointType == Kinect.JointType.HandTipRight || kinectJoint.JointType == Kinect.JointType.HandTipLeft)
        {
            // For hands, we use a specific weight
            animator.SetIKPositionWeight(goal, IK_HANDS_WEIGHT);
        }
        else if (kinectJoint.JointType == Kinect.JointType.FootRight || kinectJoint.JointType == Kinect.JointType.FootLeft)
        {
            // For feet, we use a different weight
            animator.SetIKPositionWeight(goal, IK_FEET_WEIGHT);
        }
        else
        {
            // For other joints, use the default weight
            animator.SetIKPositionWeight(goal, IK_DEFAULT_WEIGHT);
        }

        animator.SetIKPosition(goal, unityPos);
    }
    // Inverse Kinematics Hints for elbows and knees
    private void ApplyIKHint(Kinect.JointType joint, AvatarIKHint hint)
    {
        var kinectJoint = trackedBody.Joints[joint];
        if (kinectJoint.TrackingState == Kinect.TrackingState.NotTracked) return; // Avoid phantom movements

        var kinectJointPos = trackedBody.Joints[joint].Position;
        Vector3 unityPos = BodySourceView.GetVector3FromKinectCoord(kinectJointPos.X, kinectJointPos.Y, kinectJointPos.Z);
        
        if (kinectJoint.JointType == Kinect.JointType.ElbowLeft || kinectJoint.JointType == Kinect.JointType.ElbowRight)
        {
          animator.SetIKHintPositionWeight(hint, IK_DEFAULT_WEIGHT);
        }
        else if (kinectJoint.JointType == Kinect.JointType.KneeLeft || kinectJoint.JointType == Kinect.JointType.KneeRight)
        {
          animator.SetIKHintPositionWeight(hint, IK_DEFAULT_WEIGHT);
        }
        else
        {
          animator.SetIKHintPositionWeight(hint, IK_DEFAULT_WEIGHT);
        }

        animator.SetIKHintPosition(hint, unityPos);
    }

    private void LateUpdate()
    {
        ApplyArmExtension();
        ApplyLegExtension();
    }

    // Stretches arms by an extension value
    // extension: controls how much farther out to push the lower arm and hand
    // in their respective bone-local directions.
    // NOTE: effectiveExtension = extension * scaleFactor, which scaleFactor is performed uniformly based on height
    void ApplyArmExtension()
    {
        if (_ExtensionFactors.lowerArmExtensionFactor == 0 || _ExtensionFactors.upperArmExtensionFactor == 0) return;

        // LEFT HAND
        var l_UpperArmT = animator.GetBoneTransform(HumanBodyBones.LeftUpperArm);
        var l_lowerArmT = animator.GetBoneTransform(HumanBodyBones.LeftLowerArm);
        var l_handT = animator.GetBoneTransform(HumanBodyBones.LeftHand);

        // Extend the left upper arm from the shoulder along the upper arm's direction.
        if (l_UpperArmT != null && l_lowerArmT != null)
        {
            float before = Vector3.Distance(l_UpperArmT.position, l_lowerArmT.position);

            Vector3 direction = (l_lowerArmT.position - l_UpperArmT.position).normalized;

            // Extension should be scaled to world space (because UniformScaleFactor was applied to the whole avatar)
            Vector3 worldOffset = direction * _ExtensionFactors.upperArmExtensionFactor;

            l_lowerArmT.position += worldOffset;

            float after = Vector3.Distance(l_UpperArmT.position, l_lowerArmT.position);
        }


        // Extend the hand farther from the elbow along the forearm's direction.
        if (l_lowerArmT != null && l_handT != null)
        {
            float before = Vector3.Distance(l_lowerArmT.position, l_handT.position);

            Vector3 forearmDir = (l_handT.position - l_lowerArmT.position).normalized;
            Vector3 worldOffset = forearmDir * _ExtensionFactors.lowerArmExtensionFactor;

            l_handT.position += worldOffset;

            float after = Vector3.Distance(l_lowerArmT.position, l_handT.position);
        }

        // RIGHT HAND
        var r_UpperArmT = animator.GetBoneTransform(HumanBodyBones.RightUpperArm);
        var r_lowerArmT = animator.GetBoneTransform(HumanBodyBones.RightLowerArm);
        var r_handT = animator.GetBoneTransform(HumanBodyBones.RightHand);

        // Extend the left upper arm from the shoulder along the upper arm's direction.
        if (r_UpperArmT != null && r_lowerArmT != null)
        {
            float before = Vector3.Distance(r_UpperArmT.position, r_lowerArmT.position);

            Vector3 direction = (r_lowerArmT.position - r_UpperArmT.position).normalized;

            // Extension should be scaled to world space (because UniformScaleFactor was applied to the whole avatar)
            Vector3 worldOffset = direction * _ExtensionFactors.upperArmExtensionFactor;

            r_lowerArmT.position += worldOffset;

            float after = Vector3.Distance(r_UpperArmT.position, r_lowerArmT.position);
        }


        // Extend the hand farther from the elbow along the forearm's direction.
        if (r_lowerArmT != null && r_handT != null)
        {
            float before = Vector3.Distance(r_lowerArmT.position, r_handT.position);

            Vector3 forearmDir = (r_handT.position - r_lowerArmT.position).normalized;
            Vector3 worldOffset = forearmDir * _ExtensionFactors.lowerArmExtensionFactor;

            r_handT.position += worldOffset;

            float after = Vector3.Distance(r_lowerArmT.position, r_handT.position);
        }
    }

    private void ApplyLegExtension()
    {
        if (_ExtensionFactors.lowerLegExtensionFactor == 0 || _ExtensionFactors.upperLegExtensionFactor == 0) return;

        // LEFT LEG
        var l_UpperLegT = animator.GetBoneTransform(HumanBodyBones.LeftUpperLeg);
        var l_lowerLegT = animator.GetBoneTransform(HumanBodyBones.LeftLowerLeg);
        var l_footT = animator.GetBoneTransform(HumanBodyBones.LeftFoot);

        // Extend the left upper leg from the hip along the upper leg's direction.
        if (l_UpperLegT != null && l_lowerLegT != null)
        {
            float before = Vector3.Distance(l_UpperLegT.position, l_lowerLegT.position);

            Vector3 direction = (l_lowerLegT.position - l_UpperLegT.position).normalized;

            Vector3 worldOffset = direction * _ExtensionFactors.upperLegExtensionFactor;

            l_lowerLegT.position += worldOffset;

            float after = Vector3.Distance(l_UpperLegT.position, l_lowerLegT.position);
        }

        // Extend the foot farther from the knee along the lower leg's direction.

        if (l_lowerLegT != null && l_footT != null)
        {
            float before = Vector3.Distance(l_lowerLegT.position, l_footT.position);

            Vector3 lowerLegDir = (l_footT.position - l_lowerLegT.position).normalized;

            Vector3 worldOffset = lowerLegDir * _ExtensionFactors.lowerLegExtensionFactor;

            l_footT.position += worldOffset;

            float after = Vector3.Distance(l_lowerLegT.position, l_footT.position);
        }

        // RIGHT LEG
        var r_UpperLegT = animator.GetBoneTransform(HumanBodyBones.RightUpperLeg);
        var r_lowerLegT = animator.GetBoneTransform(HumanBodyBones.RightLowerLeg);
        var r_footT = animator.GetBoneTransform(HumanBodyBones.RightFoot);

        // Extend the right upper leg from the hip along the upper leg's direction.
        if (r_UpperLegT != null && r_lowerLegT != null)
        {
            float before = Vector3.Distance(r_UpperLegT.position, r_lowerLegT.position);

            Vector3 direction = (r_lowerLegT.position - r_UpperLegT.position).normalized;

            Vector3 worldOffset = direction * _ExtensionFactors.upperLegExtensionFactor;

            r_lowerLegT.position += worldOffset;

            float after = Vector3.Distance(r_UpperLegT.position, r_lowerLegT.position);
        }

        // Extend the foot farther from the knee along the lower leg's direction.

        if (r_lowerLegT != null && r_footT != null)
        {
            float before = Vector3.Distance(r_lowerLegT.position, r_footT.position);

            Vector3 lowerLegDir = (r_footT.position - r_lowerLegT.position).normalized;

            Vector3 worldOffset = lowerLegDir * _ExtensionFactors.lowerLegExtensionFactor;

            r_footT.position += worldOffset;

            float after = Vector3.Distance(r_lowerLegT.position, r_footT.position);
        }
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
    void GetAvatarHeight(GameObject armature)
    {
        Transform headTop = armature.transform.FindDeepChild("mixamorig:HeadTop_End");
        Transform footLeft = armature.transform.FindDeepChild("mixamorig:LeftToeBase");
        Transform footRight = armature.transform.FindDeepChild("mixamorig:RightToeBase");

        if (headTop == null || footLeft == null || footRight == null)
        {
            Debug.LogError("AvatarMeasurement: Could not find bones for height calculation.");
        }

        float footAvgY = (footLeft.position.y + footRight.position.y) / 2f;
        float height = headTop.position.y - footAvgY;

        // Debug.Log($"AvatarMeasurement: Height = {height:F3} Unity units");
        _AvatarMeasurements.height = height;
    }

    // Populate arm lengths
    void GetAvatarArmLengths(GameObject armature)
    {
        Transform leftShoulder = armature.transform.FindDeepChild("mixamorig:LeftArm");
        Transform leftElbow = armature.transform.FindDeepChild("mixamorig:LeftForeArm");
        Transform leftWrist = armature.transform.FindDeepChild("mixamorig:LeftHand");

        if (leftShoulder == null || leftElbow == null || leftWrist == null)
        {
            Debug.LogError("AvatarMeasurement: Could not find bones for arm length calculation.");
        }

        float upperArmLength = Vector3.Distance(leftShoulder.position, leftElbow.position);
        float foreArmLength = Vector3.Distance(leftElbow.position, leftWrist.position);

        // Debug.Log($"AvatarMeasurement: Upper arm = {upperArmLength:F3}, Forearm = {foreArmLength:F3}");
        _AvatarMeasurements.upperArmLength = upperArmLength;
        _AvatarMeasurements.lowerArmLength = foreArmLength;
    }

    // Get total leg length (hip to foot)
    public void GetAvatarLegLength(GameObject armature)
    {
        Transform hipLeft = armature.transform.FindDeepChild("mixamorig:LeftUpLeg");
        Transform kneeLeft = armature.transform.FindDeepChild("mixamorig:LeftLeg");
        Transform ankleLeft = armature.transform.FindDeepChild("mixamorig:LeftFoot");

        if (hipLeft == null || kneeLeft == null || ankleLeft == null)
        {
            Debug.LogError("AvatarMeasurement: Could not find bones for leg length calculation.");
        }

        float upperLegLength = Vector3.Distance(hipLeft.position, kneeLeft.position);
        float lowerLegLength = Vector3.Distance(kneeLeft.position, ankleLeft.position);

        // Debug.Log($"AvatarMeasurement: Upper leg = {upperLegLength:F3}, Lower leg = {lowerLegLength:F3}");
        _AvatarMeasurements.upperLegLength = upperLegLength;
        _AvatarMeasurements.lowerLegLength = lowerLegLength;
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
        _UserMeasurements.spineMidWidth = MeasureWidthAtJoint(
            joints[Kinect.JointType.SpineMid], mapper, bodyIndexData, depthData);
        
        Debug.Log($"=== Body Measurements (all in meters) ===");
        Debug.Log($"  Height: {_UserMeasurements.height:F3} m");
        Debug.Log($"  SpineMid width: {_UserMeasurements.spineMidWidth:F3} m");
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

using System.Linq;
using UnityEngine;
using Kinect = Windows.Kinect;
using Vector3 = UnityEngine.Vector3;

class HumanoidMeasurements
{
    public float height;
    public float armLength;
    public float legLength;
}

// Responsible for controlling the clothed base avatar, making it track the user's body
public class AvatarController : MonoBehaviour
{
    // Unity Objects
    public Animator animator;
    public GameObject BodySourceManager;
    private BodySourceManager _BodyManager;
    public GameObject ClothedBaseAvatar;
    private GameObject Armature;
    private const string ARMATURE = "Armature";
    private Kinect.Body trackedBody; // The body being tracked by the avatar
    
    // Inverse Kinematics Variables
    public bool enableInverseKinematics = true;
    private bool armExtended = false; // Flag to check if the arm has been extended already
    private Vector3 initialLowerArmLocalPos;
    private Vector3 initialHandLocalPos;
    private readonly float IK_HANDS_WEIGHT = 1f;
    private readonly float IK_FEET_WEIGHT = 1f;
    private readonly float IK_HEAD_DIRECTION_WEIGHT = 1f;
    private readonly float IK_DEFAULT_WEIGHT = 1f;

    // User Measurement Variables
    private HumanoidMeasurements _UserMeasurements = new();

    // Clothed Base Avatar Measurement Variables
    private HumanoidMeasurements _AvatarMeasurements = new();

    void Start()
    {
        animator = GetComponent<Animator>();
        if (animator == null)
        {
            Debug.LogError("Animator component not found on AvatarController.");
        }

        _BodyManager = BodySourceManager.GetComponent<BodySourceManager>();
        if (_BodyManager == null)
        {
            Debug.LogError("BodySourceManager component not found.");
        }

        // Get the measurements of the ClothedBaseAvatar
        Armature = ClothedBaseAvatar.transform.Find(ARMATURE)?.gameObject;
        _AvatarMeasurements.height = GetHeight(Armature);
        _AvatarMeasurements.armLength = GetArmLength(Armature);
        _AvatarMeasurements.legLength = GetLegLength(Armature);
        Debug.Log("_AvatarMeasurements - Height: " + _AvatarMeasurements.height);
        Debug.Log("_AvatarMeasurements - Arm Length: " + _AvatarMeasurements.armLength);
        Debug.Log("_AvatarMeasurements - Leg Length: " + _AvatarMeasurements.legLength);

        // Cache original local positions
        initialLowerArmLocalPos = animator.GetBoneTransform(HumanBodyBones.LeftLowerArm)?.localPosition ?? Vector3.zero;
        initialHandLocalPos = animator.GetBoneTransform(HumanBodyBones.LeftHand)?.localPosition ?? Vector3.zero;
    }

    // Updates the body object currently being tracked
    void Update()
    {
        if (_BodyManager == null) return;

        // Get the bodies
        Kinect.Body[] data = _BodyManager.GetData();
        if (data == null) return;

        // Use the first tracked body
        trackedBody = data.FirstOrDefault(b => b != null && b.IsTracked);

        // Get the joints map
        var joints = trackedBody.Joints;

        // Move the avatar to the location of the skeleton
        Vector3 spineBasePos = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.SpineBase]);
        ClothedBaseAvatar.transform.position = new Vector3(spineBasePos.x, spineBasePos.y, spineBasePos.z);

        // Determine scaling factors
        Vector3 head = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.Head]);
        Vector3 footLeft = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.FootLeft]);
        Vector3 footRight = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.FootRight]);

        // First uniformly scale based on height
        _UserMeasurements.height = head.y - ((footLeft.y + footRight.y) / 2f); // average foot height
        Debug.Log("Height (Kinect 10x): " + _UserMeasurements.height.ToString("F3"));
        float scaleFactor = _UserMeasurements.height / _AvatarMeasurements.height; // AvatarHeight*scaleFactor = UserHeight
        ClothedBaseAvatar.transform.localScale = new Vector3(scaleFactor, scaleFactor, scaleFactor);

        // Arm length (Shoulder -> Elbow -> Wrist)
        // _UserMeasurements.armLength = Vector3.Distance(shoulderLeft, elbowLeft) + Vector3.Distance(elbowLeft, wristLeft);
        // Debug.Log("Arm length (Kinect 10x): " + _UserMeasurements.armLength.ToString("F3"));
    }

    // A callback function to calculate inverse kinematics
    private void OnAnimatorIK(int layerIndex)
    {
        Debug.Log("OnAnimatorIK called with layer: " + layerIndex);
        if (animator == null || trackedBody == null || !enableInverseKinematics)
        {
            Debug.Log("  But no trackedBody!");
            return;
        }

        // NOTE: Kinect uses camera-facing perspective, meaning its left is the avatar's right
        //       Meanwhile, Unity’s AvatarIKGoal.LeftHand refers to the avatar’s anatomical left

        // Move hands to their goals
        ApplyIK(Kinect.JointType.HandTipRight, AvatarIKGoal.LeftHand);
        ApplyIK(Kinect.JointType.HandTipLeft, AvatarIKGoal.RightHand);

        // Move feet to their goals
        ApplyIK(Kinect.JointType.FootRight, AvatarIKGoal.LeftFoot);
        ApplyIK(Kinect.JointType.FootLeft, AvatarIKGoal.RightFoot);

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
    private void ApplyIK(Kinect.JointType joint, AvatarIKGoal goal)
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

  private void LateUpdate()
  {
    float armLengthExtension = 0.005f; // 0.5cm extension
    ApplyArmExtension(armLengthExtension);
  }

  // Stretches arms by an extension value
  // extension: controls how much farther out to push the lower arm and hand
  // in their respective bone-local directions.
  // NOTE: effectiveExtension = extension * scaleFactor, which scaleFactor is performed uniformly based on height
  void ApplyArmExtension(float extension)
  {
    var upperArmT = animator.GetBoneTransform(HumanBodyBones.LeftUpperArm);
    var lowerArmT = animator.GetBoneTransform(HumanBodyBones.LeftLowerArm);
    var handT = animator.GetBoneTransform(HumanBodyBones.LeftHand);

    // Extend the upper arm from the shoulder along the upper arm's direction.
    if (upperArmT != null && lowerArmT != null)
    {
      float before = Vector3.Distance(upperArmT.position, lowerArmT.position);

      Vector3 direction = (lowerArmT.position - upperArmT.position).normalized; // Direction from upper arm to lower arm
      Vector3 localDir = upperArmT.InverseTransformDirection(direction);        // Get direction of extension relative to the rig
      lowerArmT.localPosition = initialLowerArmLocalPos + localDir * extension; // Extends the lower arm in the local direction

      float after = Vector3.Distance(upperArmT.position, lowerArmT.position);
      Debug.Log($"Elbow extended: {after - before} world units");
    }

    // Extend the hand farther from the elbow along the forearm's direction.
    if (lowerArmT != null && handT != null)
    {
      float before = Vector3.Distance(lowerArmT.position, handT.position);

      Vector3 forearmDir = (handT.position - lowerArmT.position).normalized;
      Vector3 localForearmDir = lowerArmT.InverseTransformDirection(forearmDir);
      handT.localPosition = initialHandLocalPos + localForearmDir * extension;

      float after = Vector3.Distance(lowerArmT.position, handT.position);
      Debug.Log($"Hand extended: {after - before} world units");
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
    public static float GetHeight(GameObject armature)
    {
        Transform headTop = armature.transform.FindDeepChild("mixamorig:HeadTop_End");
        Transform footLeft = armature.transform.FindDeepChild("mixamorig:LeftToeBase");
        Transform footRight = armature.transform.FindDeepChild("mixamorig:RightToeBase");

        if (headTop == null || footLeft == null || footRight == null)
        {
            Debug.LogError("AvatarMeasurement: Could not find bones for height calculation.");
            return 0f;
        }

        float footAvgY = (footLeft.position.y + footRight.position.y) / 2f;
        float height = headTop.position.y - footAvgY;

        Debug.Log($"AvatarMeasurement: Height = {height:F3} Unity units");
        return height;
    }

    // Get total arm length (shoulder to wrist)
    public static float GetArmLength(GameObject armature)
    {
        Transform leftShoulder = armature.transform.FindDeepChild("mixamorig:LeftArm");
        Transform leftElbow = armature.transform.FindDeepChild("mixamorig:LeftForeArm");
        Transform leftWrist = armature.transform.FindDeepChild("mixamorig:LeftHand");

        if (leftShoulder == null || leftElbow == null || leftWrist == null)
        {
            Debug.LogError("AvatarMeasurement: Could not find bones for arm length calculation.");
            return 0f;
        }

        float upperArmLength = Vector3.Distance(leftShoulder.position, leftElbow.position);
        float foreArmLength = Vector3.Distance(leftElbow.position, leftWrist.position);
        float totalArmLength = upperArmLength + foreArmLength;

        Debug.Log($"AvatarMeasurement: Upper arm = {upperArmLength:F3}, Forearm = {foreArmLength:F3}, Total = {totalArmLength:F3}");
        return totalArmLength;
    }


    // Get total leg length (hip to foot)
    public static float GetLegLength(GameObject armature)
    {
        Transform hipLeft = armature.transform.FindDeepChild("mixamorig:LeftUpLeg");
        Transform kneeLeft = armature.transform.FindDeepChild("mixamorig:LeftLeg");
        Transform ankleLeft = armature.transform.FindDeepChild("mixamorig:LeftFoot");

        if (hipLeft == null || kneeLeft == null || ankleLeft == null)
        {
            Debug.LogError("AvatarMeasurement: Could not find bones for leg length calculation.");
            return 0f;
        }

        float upperLegLength = Vector3.Distance(hipLeft.position, kneeLeft.position);
        float lowerLegLength = Vector3.Distance(kneeLeft.position, ankleLeft.position);
        float totalLegLength = upperLegLength + lowerLegLength;

        Debug.Log($"AvatarMeasurement: Upper leg = {upperLegLength:F3}, Lower leg = {lowerLegLength:F3}, Total = {totalLegLength:F3}");
        return totalLegLength;
    }
}

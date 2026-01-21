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

    public float napeToWaist;    // SpineShoulder to SpineMid
    public float shoulderDist;   // Avg(SpineShoulder to ShoulderLeft/Right)
    public float waistToHip;     // SpineBase to SpineMid
    public float neckHeight;     // Head to Neck
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

        _KinectUserMeasurements.height = head.y - ((footL.y + footR.y) * 0.5f);

        // MeasureKinectUserBodyParts(trackedBody);
        // Print all measurements (in meters)
        // Debug.Log(
        //     $"Height: {_KinectUserMeasurements.height:F3} m, " +
        //     $"NapeToWaist: {_KinectUserMeasurements.napeToWaist:F3} m, " +
        //     $"ShoulderDist: {_KinectUserMeasurements.shoulderDist:F3} m, " +
        //     $"WaistToHip: {_KinectUserMeasurements.waistToHip:F3} m, " +
        //     $"NeckHeight: {_KinectUserMeasurements.neckHeight:F3} m, " +
        //     $"UpperArm: {_KinectUserMeasurements.upperArmLength:F3} m, " +
        //     $"LowerArm: {_KinectUserMeasurements.lowerArmLength:F3} m, " +
        //     $"UpperLeg: {_KinectUserMeasurements.upperLegLength:F3} m, " +
        //     $"LowerLeg: {_KinectUserMeasurements.lowerLegLength:F3} m"
        // );

        hasValidBody = true;
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

    private void MeasureKinectUserBodyParts(Kinect.Body body)
    {
        var joints = body.Joints;

        // Torso measurements
        _KinectUserMeasurements.napeToWaist = MeasureDistance(joints, Kinect.JointType.SpineShoulder, Kinect.JointType.SpineMid);
        _KinectUserMeasurements.waistToHip = MeasureDistance(joints, Kinect.JointType.SpineBase, Kinect.JointType.SpineMid);
        _KinectUserMeasurements.neckHeight = MeasureDistance(joints, Kinect.JointType.Head, Kinect.JointType.Neck);
        _KinectUserMeasurements.shoulderDist = MeasureAverageShoulderDistance(joints);

        // Limb measurements
        _KinectUserMeasurements.upperArmLength = MeasureAverageBilateralDistance(joints, 
            Kinect.JointType.ShoulderLeft, Kinect.JointType.ElbowLeft,
            Kinect.JointType.ShoulderRight, Kinect.JointType.ElbowRight);
        
        _KinectUserMeasurements.lowerArmLength = MeasureAverageBilateralDistance(joints,
            Kinect.JointType.ElbowLeft, Kinect.JointType.WristLeft,
            Kinect.JointType.ElbowRight, Kinect.JointType.WristRight);
        
        _KinectUserMeasurements.upperLegLength = MeasureAverageBilateralDistance(joints,
            Kinect.JointType.HipLeft, Kinect.JointType.KneeLeft,
            Kinect.JointType.HipRight, Kinect.JointType.KneeRight);
        
        _KinectUserMeasurements.lowerLegLength = MeasureAverageBilateralDistance(joints,
            Kinect.JointType.KneeLeft, Kinect.JointType.AnkleLeft,
            Kinect.JointType.KneeRight, Kinect.JointType.AnkleRight);
    }

    /// <summary>
    /// Measures the distance between two joints.
    /// </summary>
    private float MeasureDistance(IReadOnlyDictionary<Kinect.JointType, Kinect.Joint> joints, Kinect.JointType jointA, Kinect.JointType jointB)
    {
        Vector3 posA = BodySourceView.GetVector3FromJoint(joints[jointA]);
        Vector3 posB = BodySourceView.GetVector3FromJoint(joints[jointB]);
        return Vector3.Distance(posA, posB);
    }

    /// <summary>
    /// Measures the average distance between bilateral (left/right) joint pairs.
    /// </summary>
    private float MeasureAverageBilateralDistance(IReadOnlyDictionary<Kinect.JointType, Kinect.Joint> joints,
        Kinect.JointType leftStart, Kinect.JointType leftEnd,
        Kinect.JointType rightStart, Kinect.JointType rightEnd)
    {
        float leftDistance = MeasureDistance(joints, leftStart, leftEnd);
        float rightDistance = MeasureDistance(joints, rightStart, rightEnd);
        return (leftDistance + rightDistance) * 0.5f;
    }

    /// <summary>
    /// Measures the average distance from SpineShoulder to both shoulders.
    /// </summary>
    private float MeasureAverageShoulderDistance(IReadOnlyDictionary<Kinect.JointType, Kinect.Joint> joints)
    {
        Vector3 spineShoulder = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.SpineShoulder]);
        Vector3 shoulderLeft = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.ShoulderLeft]);
        Vector3 shoulderRight = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.ShoulderRight]);
        
        float distLeft = Vector3.Distance(spineShoulder, shoulderLeft);
        float distRight = Vector3.Distance(spineShoulder, shoulderRight);
        return (distLeft + distRight) * 0.5f;
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

        _AvatarMeasurements.height = height;
    }

    /// <summary>
    /// Applies forward kinematics to map Kinect body tracking to Unity avatar bones.
    /// </summary>
    private void ApplyForwardKinematics(Kinect.Body body)
    {
        if (body == null) return;

        ApplyLimbBoneRotations(body);
        ApplyHeadBoneRotation(body);
    }

    /// <summary>
    /// Applies rotations to limb bones (arms and legs) using forward kinematics.
    /// </summary>
    private void ApplyLimbBoneRotations(Kinect.Body body)
    {
        var limbBoneMapping = new List<(Kinect.JointType Start, Kinect.JointType End, HumanBodyBones UnityBone)>
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

        foreach (var (Start, End, UnityBone) in limbBoneMapping)
        {
            Transform boneTransform = animator.GetBoneTransform(UnityBone);
            if (boneTransform == null) continue;

            // Get joint positions in Unity space
            Vector3 startPos = GetJointPosition(body.Joints[Start]);
            Vector3 endPos = GetJointPosition(body.Joints[End]);
            Vector3 boneDirection = (startPos - endPos).normalized;

            // Apply T-pose offset for limbs
            Quaternion tPoseOffset = Quaternion.Euler(-90, 0, 0);
            Quaternion worldRotation = Quaternion.LookRotation(boneDirection, Vector3.up) * tPoseOffset;

            // Convert to local rotation
            ApplyLocalRotation(boneTransform, worldRotation);
        }
    }

    /// <summary>
    /// Applies rotation to the head bone using forward kinematics with anatomical constraints.
    /// </summary>
    private void ApplyHeadBoneRotation(Kinect.Body body)
    {
        Transform headTransform = animator.GetBoneTransform(HumanBodyBones.Head);
        if (headTransform == null) return;

        // Get neck and head positions
        Vector3 neckPos = GetJointPosition(body.Joints[Kinect.JointType.Neck]);
        Vector3 headPos = GetJointPosition(body.Joints[Kinect.JointType.Head]);
        Vector3 headUp = (headPos - neckPos).normalized;

        // Calculate head forward direction using shoulder orientation
        Vector3 shoulderLeft = GetJointPosition(body.Joints[Kinect.JointType.ShoulderLeft]);
        Vector3 shoulderRight = GetJointPosition(body.Joints[Kinect.JointType.ShoulderRight]);
        Vector3 shoulderDir = (shoulderRight - shoulderLeft).normalized;
        
        // Head forward is perpendicular to both shoulder line and head up vector
        Vector3 headForward = -Vector3.Cross(shoulderDir, headUp).normalized;
        Quaternion worldRotation = Quaternion.LookRotation(headForward, headUp);

        // Convert to local rotation
        ApplyLocalRotation(headTransform, worldRotation);
    }

    /// <summary>
    /// Converts a Kinect joint to Unity world space position.
    /// </summary>
    private Vector3 GetJointPosition(Windows.Kinect.Joint joint)
    {
        return BodySourceView.GetVector3FromKinectCoord(
            joint.Position.X, 
            joint.Position.Y, 
            joint.Position.Z
        );
    }

    /// <summary>
    /// Applies world rotation as local rotation to a bone transform.
    /// </summary>
    private void ApplyLocalRotation(Transform boneTransform, Quaternion worldRotation)
    {
        if (boneTransform.parent != null)
        {
            boneTransform.localRotation = Quaternion.Inverse(boneTransform.parent.rotation) * worldRotation;
        }
        else
        {
            boneTransform.localRotation = worldRotation;
        }
    }
}

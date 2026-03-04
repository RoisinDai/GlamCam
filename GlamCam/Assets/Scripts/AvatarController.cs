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

// Stores the original local scale of each bone for reference
class BoneScaleData
{
    public Vector3 originalLocalScale;
    public Vector3 originalLocalPosition;
    public Vector3 currentScale = Vector3.one;
    public Transform boneTransform;

    public BoneScaleData(Transform bone)
    {
        boneTransform = bone;
        originalLocalScale = bone.localScale;
        originalLocalPosition = bone.localPosition;
    }
}

// Phase 1.4: Configuration for mapping Kinect joint pairs to Unity humanoid bones
class BoneMappingConfig
{
    public Kinect.JointType startJoint;      // Start Kinect joint (e.g., ShoulderLeft)
    public Kinect.JointType endJoint;        // End Kinect joint (e.g., ElbowLeft)
    public HumanBodyBones unityBone;         // Corresponding Unity bone (note left-right mirror)
    public Vector3 stretchAxis;              // Which local axis the bone stretches along (typically Y)

    public BoneMappingConfig(Kinect.JointType start, Kinect.JointType end, HumanBodyBones bone, Vector3 axis)
    {
        startJoint = start;
        endJoint = end;
        unityBone = bone;
        stretchAxis = axis;
    }
}

// FakeUMA - Non-uniform bone scaling system
// Allows independent scaling of individual bones using inverse scale compensation
public class FakeUMA
{
    private Dictionary<Transform, BoneScaleData> _BoneScaleDatabase = new Dictionary<Transform, BoneScaleData>();
    private bool _Initialized = false;
    private GameObject _Avatar;
    private Animator _Animator;

    // ========================================================================================
    // ROTATION-AWARE INVERSE SCALE TRACKING
    // ========================================================================================
    // Tracks parent-child scale relationships to enable dynamic inverse scale updates
    // when child bones rotate relative to their parents.
    
    // Stores the scale factor applied to each parent bone (used to compute child inverse)
    private Dictionary<Transform, Vector3> _AppliedParentScale = new Dictionary<Transform, Vector3>();
    
    // Stores parent-child relationships for inverse scale updates
    // Key: child transform, Value: (parent transform, inverse scale currently applied, rotation when inverse was applied)
    private Dictionary<Transform, (Transform parent, Vector3 appliedInverse, Quaternion rotationAtApply)> _InverseScaleTracking 
        = new Dictionary<Transform, (Transform, Vector3, Quaternion)>();

    public FakeUMA(GameObject avatar, Animator animator)
    {
        _Avatar = avatar;
        _Animator = animator;
    }

    /// <summary>
    /// Initializes the bone scaling system by collecting all bones in the hierarchy.
    /// Call this once before using the scaling APIs.
    /// </summary>
    public void Initialize()
    {
        if (_Initialized) return;

        _BoneScaleDatabase.Clear();
        _AppliedParentScale.Clear();
        _InverseScaleTracking.Clear();

        // Get all transforms in the avatar hierarchy
        Transform[] allTransforms = _Avatar.GetComponentsInChildren<Transform>();

        foreach (Transform bone in allTransforms)
        {
            _BoneScaleDatabase[bone] = new BoneScaleData(bone);
        }

        _Initialized = true;
        Debug.Log($"FakeUMA bone scaling system initialized with {_BoneScaleDatabase.Count} bones.");
    }

    /// <summary>
    /// Scales a specific bone independently by applying inverse scale to its direct children.
    /// This is the core API for non-uniform bone scaling.
    /// </summary>
    /// <param name="bone">The bone transform to scale</param>
    /// <param name="scaleFactor">The scale factor to apply (e.g., Vector3(1.5f, 1.5f, 1.5f))</param>
    public void ScaleBoneIndependently(Transform bone, Vector3 scaleFactor)
    {
        if (!_Initialized)
        {
            Debug.LogWarning("FakeUMA not initialized. Call Initialize() first.");
            return;
        }

        if (bone == null)
        {
            Debug.LogError("Cannot scale null bone.");
            return;
        }

        if (!_BoneScaleDatabase.ContainsKey(bone))
        {
            Debug.LogWarning($"Bone {bone.name} not found in scaling database.");
            return;
        }

        // Step 1: Apply the scale to the bone itself
        bone.localScale = Vector3.Scale(bone.localScale, scaleFactor);

        // Update the database with the new scale
        _BoneScaleDatabase[bone].currentScale = Vector3.Scale(_BoneScaleDatabase[bone].currentScale, scaleFactor);

        // Track the cumulative scale applied to this bone (for child inverse calculations)
        if (!_AppliedParentScale.ContainsKey(bone))
            _AppliedParentScale[bone] = scaleFactor;
        else
            _AppliedParentScale[bone] = Vector3.Scale(_AppliedParentScale[bone], scaleFactor);

        // Step 2: Apply ROTATION-AWARE inverse scale to all direct children
        // The inverse must account for how the child's local axes align with the parent's axes
        foreach (Transform child in bone)
        {
            if (_BoneScaleDatabase.ContainsKey(child))
            {
                // Compute rotation-aware inverse based on child's current local rotation
                Vector3 rotationAwareInverse = ComputeRotationAwareInverseScale(scaleFactor, child.localRotation);
                
                child.localScale = Vector3.Scale(child.localScale, rotationAwareInverse);
                _BoneScaleDatabase[child].currentScale = Vector3.Scale(_BoneScaleDatabase[child].currentScale, rotationAwareInverse);
                
                // Track this relationship for future rotation updates
                _InverseScaleTracking[child] = (bone, rotationAwareInverse, child.localRotation);
            }
        }

        Debug.Log($"Scaled bone '{bone.name}' by {scaleFactor}, applied rotation-aware inverse to {bone.childCount} children.");
    }

    /// <summary>
    /// Scales a bone by name. Convenience method that finds the bone first.
    /// </summary>
    /// <param name="boneName">The name of the bone to scale</param>
    /// <param name="scaleFactor">The scale factor to apply</param>
    public void ScaleBoneByName(string boneName, Vector3 scaleFactor)
    {
        Transform bone = FindBoneByName(boneName);
        if (bone != null)
        {
            ScaleBoneIndependently(bone, scaleFactor);
        }
        else
        {
            Debug.LogError($"Bone with name '{boneName}' not found in hierarchy.");
        }
    }

    /// <summary>
    /// Scales a Unity humanoid bone independently.
    /// </summary>
    /// <param name="humanBone">The HumanBodyBones enum value</param>
    /// <param name="scaleFactor">The scale factor to apply</param>
    public void ScaleHumanoidBone(HumanBodyBones humanBone, Vector3 scaleFactor)
    {
        if (_Animator == null)
        {
            Debug.LogError("Animator is null. Cannot scale humanoid bone.");
            return;
        }

        Transform bone = _Animator.GetBoneTransform(humanBone);
        if (bone != null)
        {
            ScaleBoneIndependently(bone, scaleFactor);
        }
        else
        {
            Debug.LogError($"Humanoid bone '{humanBone}' not found in animator.");
        }
    }

    /// <summary>
    /// Resets a bone's scale to its original value.
    /// </summary>
    /// <param name="bone">The bone to reset</param>
    public void ResetBoneScale(Transform bone)
    {
        if (!_Initialized || bone == null || !_BoneScaleDatabase.ContainsKey(bone))
        {
            Debug.LogWarning("Cannot reset bone scale. System not initialized or bone not found.");
            return;
        }

        BoneScaleData data = _BoneScaleDatabase[bone];

        // Calculate the inverse of the current scale to return to original
        Vector3 resetScale = new Vector3(
            data.originalLocalScale.x / bone.localScale.x,
            data.originalLocalScale.y / bone.localScale.y,
            data.originalLocalScale.z / bone.localScale.z
        );

        // Apply the reset scale (which will also inverse scale children)
        ScaleBoneIndependently(bone, resetScale);
    }

    /// <summary>
    /// Resets all bones to their original scales.
    /// </summary>
    public void ResetAllBoneScales()
    {
        if (!_Initialized) return;

        foreach (var kvp in _BoneScaleDatabase)
        {
            Transform bone = kvp.Key;
            BoneScaleData data = kvp.Value;

            if (bone != null)
            {
                bone.localScale = data.originalLocalScale;
                bone.localPosition = data.originalLocalPosition;
                data.currentScale = Vector3.one;
            }
        }

        // Clear rotation-aware tracking data
        _AppliedParentScale.Clear();
        _InverseScaleTracking.Clear();

        // Debug.Log("Reset all bone scales to original values.");
    }

    /// <summary>
    /// Gets the current scale factor applied to a bone.
    /// </summary>
    /// <param name="bone">The bone to query</param>
    /// <returns>The cumulative scale factor applied to this bone</returns>
    public Vector3 GetBoneScaleFactor(Transform bone)
    {
        if (!_Initialized || bone == null || !_BoneScaleDatabase.ContainsKey(bone))
        {
            return Vector3.one;
        }

        return _BoneScaleDatabase[bone].currentScale;
    }

    // ========================================================================================
    // ROTATION-AWARE INVERSE SCALE COMPUTATION
    // ========================================================================================

    /// <summary>
    /// Computes the inverse scale to apply to a child bone, accounting for the child's rotation.
    /// When parent has non-uniform scale and child is rotated, the inverse must be applied
    /// to the child's axes that align with the parent's scaled axes.
    /// </summary>
    /// <param name="parentScaleFactor">The scale factor applied to the parent bone</param>
    /// <param name="childLocalRotation">The child's local rotation relative to parent</param>
    /// <returns>Rotation-aware inverse scale vector for the child</returns>
    private Vector3 ComputeRotationAwareInverseScale(Vector3 parentScaleFactor, Quaternion childLocalRotation)
    {
        // Get child's local axes expressed in parent's local coordinate system
        // childLocalRotation rotates from child space to parent space
        Vector3 childXInParent = childLocalRotation * Vector3.right;
        Vector3 childYInParent = childLocalRotation * Vector3.up;
        Vector3 childZInParent = childLocalRotation * Vector3.forward;

        // Compute how much parent's scale stretches each of the child's axes
        // For axis d, the stretch factor is |S * d| where S is the diagonal scale matrix
        float stretchAlongChildX = CalculateDirectionalStretch(parentScaleFactor, childXInParent);
        float stretchAlongChildY = CalculateDirectionalStretch(parentScaleFactor, childYInParent);
        float stretchAlongChildZ = CalculateDirectionalStretch(parentScaleFactor, childZInParent);

        // Return inverse of stretch factors (to compensate)
        return new Vector3(
            stretchAlongChildX > 0.001f ? 1f / stretchAlongChildX : 1f,
            stretchAlongChildY > 0.001f ? 1f / stretchAlongChildY : 1f,
            stretchAlongChildZ > 0.001f ? 1f / stretchAlongChildZ : 1f
        );
    }

    /// <summary>
    /// Calculates how much a scale factor stretches a particular direction.
    /// For a unit vector d and scale S, the stretch is |S * d|.
    /// </summary>
    private float CalculateDirectionalStretch(Vector3 scale, Vector3 direction)
    {
        // Apply scale to direction: S * d = (Sx*dx, Sy*dy, Sz*dz)
        Vector3 scaled = new Vector3(
            scale.x * direction.x,
            scale.y * direction.y,
            scale.z * direction.z
        );
        // Return magnitude (how much the direction is stretched)
        return scaled.magnitude;
    }

    /// <summary>
    /// Updates inverse scales for all tracked child bones based on their current rotations.
    /// Call this EVERY FRAME after applying forward kinematics (rotations) to correct
    /// for rotation-scale interaction.
    /// </summary>
    public void UpdateInverseScalesForCurrentRotations()
    {
        if (!_Initialized) return;

        // Collect updates to avoid modifying dictionary during enumeration
        var updates = new List<(Transform child, Transform parent, Vector3 newInverse, Quaternion newRotation)>();

        foreach (var kvp in _InverseScaleTracking)
        {
            Transform child = kvp.Key;
            Transform parent = kvp.Value.parent;
            Vector3 previousInverse = kvp.Value.appliedInverse;
            Quaternion previousRotation = kvp.Value.rotationAtApply;

            if (child == null || parent == null) continue;
            if (!_AppliedParentScale.ContainsKey(parent)) continue;

            // Check if rotation has changed significantly
            float rotationDelta = Quaternion.Angle(previousRotation, child.localRotation);
            if (rotationDelta < 0.1f) continue; // Skip if rotation hasn't changed much

            // Compute new rotation-aware inverse based on current rotation
            Vector3 parentScale = _AppliedParentScale[parent];
            Vector3 newInverse = ComputeRotationAwareInverseScale(parentScale, child.localRotation);

            // Compute correction factor: undo old inverse, apply new inverse
            // correction = newInverse / previousInverse
            Vector3 correction = new Vector3(
                previousInverse.x > 0.001f ? newInverse.x / previousInverse.x : 1f,
                previousInverse.y > 0.001f ? newInverse.y / previousInverse.y : 1f,
                previousInverse.z > 0.001f ? newInverse.z / previousInverse.z : 1f
            );

            // Apply correction to child's localScale
            child.localScale = Vector3.Scale(child.localScale, correction);

            // Update tracking data
            if (_BoneScaleDatabase.ContainsKey(child))
            {
                _BoneScaleDatabase[child].currentScale = Vector3.Scale(_BoneScaleDatabase[child].currentScale, correction);
            }

            // Queue tracking update (don't modify dictionary during iteration)
            updates.Add((child, parent, newInverse, child.localRotation));
        }

        // Apply all tracking updates after enumeration is complete
        foreach (var update in updates)
        {
            _InverseScaleTracking[update.child] = (update.parent, update.newInverse, update.newRotation);
        }
    }

    /// <summary>
    /// Finds a bone by name in the avatar hierarchy.
    /// </summary>
    private Transform FindBoneByName(string boneName)
    {
        foreach (var kvp in _BoneScaleDatabase)
        {
            if (kvp.Key.name == boneName)
            {
                return kvp.Key;
            }
        }
        return null;
    }

    /// <summary>
    /// Lists all bones in the scaling system for debugging.
    /// </summary>
    public void ListAllBones()
    {
        if (!_Initialized)
        {
            Debug.LogWarning("FakeUMA not initialized.");
            return;
        }

        Debug.Log($"=== FakeUMA Bone Scaling System: {_BoneScaleDatabase.Count} bones ===");
        foreach (var kvp in _BoneScaleDatabase)
        {
            Transform bone = kvp.Key;
            BoneScaleData data = kvp.Value;
            Debug.Log($"Bone: {bone.name}, Original Scale: {data.originalLocalScale}, Current Scale: {data.currentScale}, Children: {bone.childCount}");
        }
    }

    // ========================================================================================
    // HYBRID SCALING MODEL
    // ========================================================================================
    // Length is handled via TRANSLATION (moving child bone position) — zero shear.
    // Thickness is handled via SCALE with rotation-aware inverse — minimal shear.
    // This eliminates the dominant source of visual distortion (Y-axis stretch at joints).

    /// <summary>
    /// Handles length scaling by translating the child bone's localPosition along the bone axis.
    /// This completely bypasses Y-axis shear at joints by using translation instead of scale.
    /// The mesh stretches naturally through blended vertex weights between parent and child joints.
    /// </summary>
    /// <param name="humanBone">The HumanBodyBones enum value of the parent bone</param>
    /// <param name="lengthScale">Length scale factor (e.g., 1.3f = 30% longer)</param>
    public void ScaleBoneLengthViaTranslation(HumanBodyBones humanBone, float lengthScale)
    {
        if (_Animator == null) return;
        Transform parentBone = _Animator.GetBoneTransform(humanBone);
        if (parentBone == null || parentBone.childCount == 0) return;

        Transform childBone = parentBone.GetChild(0);

        if (_BoneScaleDatabase.ContainsKey(childBone))
        {
            // Get the original local position of the child bone
            Vector3 origPos = _BoneScaleDatabase[childBone].originalLocalPosition;

            // Push the child further along the parent's axis (scales the bone "length" via position)
            childBone.localPosition = origPos * lengthScale;

            Debug.Log($"Length via translation: '{parentBone.name}' child '{childBone.name}' moved from {origPos} to {childBone.localPosition} (scale {lengthScale:F3})");
        }
    }

    // ========================================================================================
    // CYLINDER-BASED SCALING API (User-Friendly Wrappers)
    // ========================================================================================
    // Each bone is approximated as a cylinder with a length axis and a thickness (radius).
    // These methods make it easier to scale bones by separating length from thickness.

    /// <summary>
    /// Scales the length of a humanoid bone along its primary axis.
    /// Think of this as making the bone longer or shorter (changing cylinder height).
    /// </summary>
    /// <param name="humanBone">The HumanBodyBones enum value</param>
    /// <param name="lengthScale">Length scale factor (e.g., 1.5f = 50% longer)</param>
    public void ScaleBoneLength(HumanBodyBones humanBone, float lengthScale)
    {
        if (_Animator == null) return;
        Transform bone = _Animator.GetBoneTransform(humanBone);
        if (bone == null) return;

        Vector3 scaleVector = GetBoneLengthScaleVector(humanBone, lengthScale);
        ScaleBoneIndependently(bone, scaleVector);
    }

    /// <summary>
    /// Scales the thickness of a humanoid bone (cross-section radius).
    /// Think of this as making the bone thicker or thinner (changing cylinder radius).
    /// </summary>
    /// <param name="humanBone">The HumanBodyBones enum value</param>
    /// <param name="thicknessScale">Thickness scale factor (e.g., 1.5f = 50% thicker)</param>
    public void ScaleBoneThickness(HumanBodyBones humanBone, float thicknessScale)
    {
        if (_Animator == null) return;
        Transform bone = _Animator.GetBoneTransform(humanBone);
        if (bone == null) return;

        Vector3 scaleVector = GetBoneThicknessScaleVector(humanBone, thicknessScale);
        ScaleBoneIndependently(bone, scaleVector);
    }

    /// <summary>
    /// Scales both length and thickness of a humanoid bone independently.
    /// </summary>
    /// <param name="humanBone">The HumanBodyBones enum value</param>
    /// <param name="lengthScale">Length scale factor</param>
    /// <param name="thicknessScale">Thickness scale factor</param>
    public void ScaleBoneLengthAndThickness(HumanBodyBones humanBone, float lengthScale, float thicknessScale)
    {
        if (_Animator == null) return;
        Transform bone = _Animator.GetBoneTransform(humanBone);
        if (bone == null) return;

        Vector3 lengthVector = GetBoneLengthScaleVector(humanBone, lengthScale);
        Vector3 thicknessVector = GetBoneThicknessScaleVector(humanBone, thicknessScale);

        // Combine length and thickness scales
        Vector3 combinedScale = new Vector3(
            lengthVector.x * thicknessVector.x,
            lengthVector.y * thicknessVector.y,
            lengthVector.z * thicknessVector.z
        );

        ScaleBoneIndependently(bone, combinedScale);
    }

    /// <summary>
    /// Determines the length axis scale vector for a specific bone.
    /// Returns a Vector3 where the length axis has the scale factor and other axes are 1.0.
    /// For standard Unity humanoid rigs, the Y-axis is the length axis for all bones.
    /// </summary>
    private Vector3 GetBoneLengthScaleVector(HumanBodyBones humanBone, float lengthScale)
    {
        // For standard Unity humanoid rigs, Y-axis is the length axis (bones grow along Y)
        // This applies to: spine, torso, head, arms, legs, fingers
        return new Vector3(1f, lengthScale, 1f);
    }

    /// <summary>
    /// Determines the thickness (cross-section) scale vector for a specific bone.
    /// Returns a Vector3 where the thickness axes have the scale factor and the length axis is 1.0.
    /// For standard Unity humanoid rigs, X and Z axes control thickness (perpendicular to Y length axis).
    /// </summary>
    private Vector3 GetBoneThicknessScaleVector(HumanBodyBones humanBone, float thicknessScale)
    {
        // For standard Unity humanoid rigs, X and Z axes control thickness
        // Since Y is the length axis, X and Z are the radial thickness axes
        // This creates uniform radial expansion (circular cross-section remains circular)
        return new Vector3(thicknessScale, 1f, thicknessScale);
    }
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
    private const float AVATAR_WIDTH_HEIGHT_RATIO = 0.1791f; // avatar SpineMid width / height (30.15cm / 168.33cm)

    // Population-based waist-to-limb width ratios (N=2505)
    private const float POPULATION_WAIST_ARM_RATIO     = 2.9527f; // waist / bicep (upper arm)
    private const float POPULATION_WAIST_FOREARM_RATIO = 3.3806f; // waist / forearm (lower arm)
    private const float POPULATION_WAIST_THIGH_RATIO   = 1.6570f; // waist / thigh (upper leg)
    private const float POPULATION_WAIST_CALF_RATIO    = 2.3948f; // waist / calf (lower leg)
    private float UniformScaleFactor = -1f;
    private ExtensionFactors _ExtensionFactors = new();
    private bool hasValidBody = false;

    // Non-uniform bone scaling system
    public FakeUMA fakeUMA;

    // MultiSourceManager reference (for T-pose width measurement)
    private MultiSourceManager _MultiSourceManager;

    // ========================================================================================
    // SMOOTHED CONTINUOUS BONE SCALING SYSTEM
    // ========================================================================================

    // Phase 1.1: Current avatar bone lengths (measured every frame to reflect scaling changes)
    private Dictionary<HumanBodyBones, float> _CurrentAvatarBoneLengths = new Dictionary<HumanBodyBones, float>();

    // Phase 1.2: Smoothed Kinect bone length measurements (updated every frame)
    private Dictionary<HumanBodyBones, float> _SmoothedKinectBoneLengths = new Dictionary<HumanBodyBones, float>();

    // Phase 1.3: Smoothing factor for exponential smoothing (0-1 range, lower = smoother)
    [Range(0.01f, 1.0f)]
    [Tooltip("Smoothing factor for bone scaling. Lower values (0.05) = more smoothing, Higher values (0.2) = more responsive")]
    public float boneSmoothingFactor = 0.1f;

    // Phase 1.4: Bone mapping configuration (Kinect joints -> Unity bones)
    private List<BoneMappingConfig> _BoneMappingConfigs;

    // ========================================================================================
    // STATISTICAL THICKNESS SCALING SYSTEM
    // ========================================================================================

    // Enable/disable thickness scaling
    [Tooltip("Enable statistical body build estimation for thickness scaling")]
    public bool enableThicknessScaling = true;

    // Spine bones that receive thickness-only scaling (no length scaling)
    // These bones make the torso wider/thinner based on the build factor
    private static readonly HumanBodyBones[] _SpineThicknessBones = new HumanBodyBones[]
    {
        HumanBodyBones.Hips,
        HumanBodyBones.Spine,
        HumanBodyBones.Chest,
        HumanBodyBones.UpperChest
    };

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

        // Find MultiSourceManager for T-pose width measurement
        _MultiSourceManager = FindObjectOfType<MultiSourceManager>();

        // Get the measurements of the ClothedBaseAvatar
        GetAvatarHeight();

        // Initialize the non-uniform bone scaling system (FakeUMA)
        fakeUMA = new FakeUMA(ClothedBaseAvatar, animator);
        fakeUMA.Initialize();

        // Phase 2: Initialize the smoothed continuous bone scaling system
        InitializeBoneMappingConfigs();

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
        // Gate: pause avatar tracking while waiting for T-pose measurement
        if (_MultiSourceManager != null && !_MultiSourceManager.IsMeasured) return;

        hasValidBody = false;

        if (_BodyManager == null) return;

        Kinect.Body[] data = _BodyManager.GetData();
        if (data == null) return;

        // trackedBody = data.FirstOrDefault(b => b != null && b.IsTracked);
        trackedBody = GetClosestTrackedBody(data);
        if (trackedBody == null) return;

        if (animator != null)
        {
            animator.enabled = false; // Disables the Animator temporarily
            // Debug.Log("Animator has been disabled for manual bone control.");
        }

        // Cache joints for use in this frame
        var joints = trackedBody.Joints;

        // Use T-pose height from MultiSourceManager (needed by EstimateBodyBuildFactor)
        if (_MultiSourceManager != null && _MultiSourceManager.MeasuredHeight > 0f)
        {
            _KinectUserMeasurements.height = _MultiSourceManager.MeasuredHeight;
        }

        hasValidBody = true;
    }

    /// <summary>
    /// Selects the tracked body closest to the Kinect camera based on Z coordinate (depth).
    /// Uses multiple torso joints (SpineBase, SpineMid, SpineShoulder) and averages their Z values
    /// for more robust and accurate depth measurement.
    /// In Kinect's coordinate system, smaller Z values indicate bodies closer to the camera.
    /// </summary>
    private Kinect.Body GetClosestTrackedBody(Kinect.Body[] bodies)
    {
        Kinect.Body closestBody = null;
        float closestZ = float.MaxValue;

        foreach (var body in bodies)
        {
            if (body != null && body.IsTracked)
            {
                // Use multiple torso joints and average their Z values for more robust measurement
                var joints = body.Joints;
                float avgZ = 0f;
                int validJointCount = 0;

                // Average Z coordinates from torso joints
                var torsoJoints = new[]
                {
                    Kinect.JointType.SpineBase,
                    Kinect.JointType.SpineMid,
                    Kinect.JointType.SpineShoulder
                };

                foreach (var jointType in torsoJoints)
                {
                    var joint = joints[jointType];
                    // Prefer tracked joints, but include inferred joints if needed
                    if (joint.TrackingState != Kinect.TrackingState.NotTracked)
                    {
                        avgZ += joint.Position.Z;
                        validJointCount++;
                    }
                }

                // Only use this body if we have at least one valid joint
                if (validJointCount > 0)
                {
                    avgZ /= validJointCount;

                    if (avgZ < closestZ)
                    {
                        closestZ = avgZ;
                        closestBody = body;
                    }
                }
            }
        }
        return closestBody;
    }

    void LateUpdate()
    {
        // Gate: pause avatar tracking while waiting for T-pose measurement
        if (_MultiSourceManager != null && !_MultiSourceManager.IsMeasured) return;

        if (!hasValidBody || trackedBody == null) return;

        // Phase 5.3: Execution order

        // 2. Then, apply forward kinematics (bone rotations)
        ApplyForwardKinematics(trackedBody);

        // 2.5. Update inverse scales for rotation changes (CRITICAL for non-uniform scaling)
        // After FK changes bone rotations, the inverse scale compensation must be updated
        if (fakeUMA != null && UniformScaleFactor > 0f)
        {
            fakeUMA.UpdateInverseScalesForCurrentRotations();
        }

        // 3. Rotate avatar root from shoulders
        RotateAvatarBasedOnShoulders(
            trackedBody.Joints[Kinect.JointType.ShoulderLeft],
            trackedBody.Joints[Kinect.JointType.ShoulderRight]
        );
        var joints = trackedBody.Joints;

        // 4. Move avatar root to spine base
        Vector3 spineBasePos = BodySourceView.GetVector3FromJoint(joints[Kinect.JointType.SpineBase]);
        ClothedBaseAvatar.transform.position = new Vector3(spineBasePos.x, spineBasePos.y, spineBasePos.z);

        // 5. Apply uniform scaling ONCE using T-pose height from MultiSourceManager
        if (UniformScaleFactor < 0f && _MultiSourceManager != null && _MultiSourceManager.MeasuredHeight > 0f && _AvatarMeasurements.height > 0f && _MultiSourceManager.MeasuredSpineMidWidth > 0f)
        {
            UniformScaleFactor =
                _KinectUserMeasurements.height / _AvatarMeasurements.height;

            ClothedBaseAvatar.transform.localScale =
                Vector3.one * UniformScaleFactor;

            // 1. Update bone scaling based on Kinect measurements
            UpdateContinuousBoneScaling();

            Debug.Log($"UniformScaleFactor set to {UniformScaleFactor:F3} (T-pose height: {_MultiSourceManager.MeasuredHeight:F3}m, avatar height: {_AvatarMeasurements.height:F3})");
        }

        // 6. Optional: shoulder-based translation correction
        ApplyUniformTranslationBasedOnShoulders();
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

    private void ApplyUniformTranslationBasedOnShoulders()
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
        Vector3 kinectShoulderAvg = (kinectLeft + kinectRight) * 0.5f;

        Vector3 modelLeft = modelShoulderLeft.position;
        Vector3 modelRight = modelShoulderRight.position;
        Vector3 modelShoulderAvg = (modelLeft + modelRight) * 0.5f;

        Vector3 delta = (kinectShoulderAvg - modelShoulderAvg);

        Vector3 avatarPosition = ClothedBaseAvatar.transform.position;
        ClothedBaseAvatar.transform.position = new Vector3(avatarPosition.x, avatarPosition.y + delta.y, avatarPosition.z); // Small offset from experimental results
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

        ApplySpineBoneRotations(body);
        ApplyLimbBoneRotations(body);
        ApplyHeadBoneRotation(body);
        ApplyFeetRotations(body);
    }

    /// <summary>
    /// Applies rotations to spine bones (Hips, Spine, Chest) to enable bending.
    /// </summary>
    private void ApplySpineBoneRotations(Kinect.Body body)
    {
        // Calculate shoulder orientation for forward direction
        Vector3 shoulderLeft = GetJointPosition(body.Joints[Kinect.JointType.ShoulderLeft]);
        Vector3 shoulderRight = GetJointPosition(body.Joints[Kinect.JointType.ShoulderRight]);
        Vector3 shoulderDir = (shoulderRight - shoulderLeft).normalized;

        var spineBoneMapping = new List<(Kinect.JointType Start, Kinect.JointType End, HumanBodyBones UnityBone)>
        {
            (Kinect.JointType.SpineBase, Kinect.JointType.SpineMid, HumanBodyBones.Hips),
            (Kinect.JointType.SpineMid, Kinect.JointType.SpineShoulder, HumanBodyBones.Spine),
            (Kinect.JointType.SpineShoulder, Kinect.JointType.Neck, HumanBodyBones.Chest),
        };

        foreach (var (Start, End, UnityBone) in spineBoneMapping)
        {
            Transform boneTransform = animator.GetBoneTransform(UnityBone);
            if (boneTransform == null) continue;

            // Get joint positions in Unity space
            Vector3 startPos = GetJointPosition(body.Joints[Start]);
            Vector3 endPos = GetJointPosition(body.Joints[End]);
            Vector3 boneDirection = (endPos - startPos).normalized;

            // Forward direction is perpendicular to shoulder line
            Vector3 forward = -Vector3.Cross(shoulderDir, Vector3.up).normalized;

            // Create rotation: bone points up, forward direction based on shoulders
            Quaternion worldRotation = Quaternion.LookRotation(forward, boneDirection);

            // Convert to local rotation
            ApplyLocalRotation(boneTransform, worldRotation);
        }
    }

    /// <summary>
    /// Applies rotations to limb bones (arms and legs) using forward kinematics.
    /// </summary>
    private void ApplyLimbBoneRotations(Kinect.Body body)
    {
        ApplyArmBoneRotations(body);
        ApplyLegBoneRotations(body);
    }

    /// <summary>
    /// Applies rotations to arm bones using forward kinematics.
    /// </summary>
    private void ApplyArmBoneRotations(Kinect.Body body)
    {
        var armBoneMapping = new List<(Kinect.JointType Start, Kinect.JointType End, HumanBodyBones UnityBone)>
        {
            (Kinect.JointType.ElbowLeft, Kinect.JointType.WristLeft, HumanBodyBones.RightLowerArm),
            (Kinect.JointType.ShoulderLeft, Kinect.JointType.ElbowLeft, HumanBodyBones.RightUpperArm),
            (Kinect.JointType.ElbowRight, Kinect.JointType.WristRight, HumanBodyBones.LeftLowerArm),
            (Kinect.JointType.ShoulderRight, Kinect.JointType.ElbowRight, HumanBodyBones.LeftUpperArm),
        };

        foreach (var (Start, End, UnityBone) in armBoneMapping)
        {
            Transform boneTransform = animator.GetBoneTransform(UnityBone);
            if (boneTransform == null) continue;

            // Get joint positions in Unity space
            Vector3 startPos = GetJointPosition(body.Joints[Start]);
            Vector3 endPos = GetJointPosition(body.Joints[End]);
            Vector3 boneDirection = (startPos - endPos).normalized;

            // Apply T-pose offset for arms
            Quaternion tPoseOffset = Quaternion.Euler(-90, 0, 0);
            Quaternion worldRotation = Quaternion.LookRotation(boneDirection, Vector3.up) * tPoseOffset;

            // Convert to local rotation
            ApplyLocalRotation(boneTransform, worldRotation);
        }
    }

    /// <summary>
    /// Applies rotations to leg bones using forward kinematics.
    /// </summary>
    private void ApplyLegBoneRotations(Kinect.Body body)
    {
        // 1. REORDERED MAPPING: Upper Legs MUST be processed before Lower Legs.
        var legBoneMapping = new List<(Kinect.JointType Start, Kinect.JointType End, HumanBodyBones UnityBone)>
        {
            // Upper Legs (Parents)
            (Kinect.JointType.HipLeft, Kinect.JointType.KneeLeft, HumanBodyBones.RightUpperLeg),
            (Kinect.JointType.HipRight, Kinect.JointType.KneeRight, HumanBodyBones.LeftUpperLeg),
            
            // Lower Legs (Children)
            (Kinect.JointType.KneeLeft, Kinect.JointType.AnkleLeft, HumanBodyBones.RightLowerLeg),
            (Kinect.JointType.KneeRight, Kinect.JointType.AnkleRight, HumanBodyBones.LeftLowerLeg),
        };

        // Get hip line to determine the "Right" axis of the character
        Vector3 hipLeft = GetJointPosition(body.Joints[Kinect.JointType.HipLeft]);
        Vector3 hipRight = GetJointPosition(body.Joints[Kinect.JointType.HipRight]);
        Vector3 hipLine = (hipRight - hipLeft).normalized;

        foreach (var (Start, End, UnityBone) in legBoneMapping)
        {
            Transform boneTransform = animator.GetBoneTransform(UnityBone);
            if (boneTransform == null) continue;

            // Get joint positions in Unity space
            Vector3 startPos = GetJointPosition(body.Joints[Start]);
            Vector3 endPos = GetJointPosition(body.Joints[End]);
            Vector3 boneDirection = (endPos - startPos).normalized;
            if (boneDirection == Vector3.zero) continue;

            // 2. CALCULATE FACING DIRECTION (Z-Axis)
            // We want the front of the leg (Z) to point Forward relative to the hips.
            // Cross Product: (Bone Direction [Down]) x (Hip Line [Right]) = Forward
            // Note: If leg is vertical, Down x Right = Forward.
            Vector3 faceDirection = Vector3.Cross(boneDirection, hipLine).normalized;

            // 3. APPLY ROTATION
            // Quaternion.LookRotation(View, UpHint):
            // - We set View (Z) to 'faceDirection' (Forward).
            // - We set UpHint (Y) to 'boneDirection' (Down). 
            //   (Note: Passing the bone direction as the "Up" hint aligns the Y-axis along the bone).
            Quaternion worldRotation = Quaternion.LookRotation(faceDirection, boneDirection);

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

    /// <summary>
    /// Points the feet forward using the shoulder normal as the forward direction, with a 90-degree X offset.
    /// </summary>
    private void ApplyFeetRotations(Kinect.Body body)
    {
        // Calculate shoulder normal (forward direction)
        Vector3 shoulderLeft = GetJointPosition(body.Joints[Kinect.JointType.ShoulderLeft]);
        Vector3 shoulderRight = GetJointPosition(body.Joints[Kinect.JointType.ShoulderRight]);
        Vector3 shoulderDir = (shoulderRight - shoulderLeft).normalized;
        Vector3 forward = -Vector3.Cross(shoulderDir, Vector3.up).normalized;
        Vector3 up = Vector3.up;
        Quaternion footOffset = Quaternion.Euler(90, 0, 0);

        // Left foot
        Transform leftFoot = animator.GetBoneTransform(HumanBodyBones.LeftFoot);
        if (leftFoot != null)
        {
            Quaternion worldRotation = Quaternion.LookRotation(forward, up) * footOffset;
            ApplyLocalRotation(leftFoot, worldRotation);
        }

        // Right foot
        Transform rightFoot = animator.GetBoneTransform(HumanBodyBones.RightFoot);
        if (rightFoot != null)
        {
            Quaternion worldRotation = Quaternion.LookRotation(forward, up) * footOffset;
            ApplyLocalRotation(rightFoot, worldRotation);
        }
    }

    // ========================================================================================
    // PHASE 2: INITIALIZATION SYSTEM
    // ========================================================================================

    /// <summary>
    /// Phase 2.1 & 1.4: Initialize bone mapping configuration that maps Kinect joint pairs to Unity bones.
    /// This defines which bones will be scaled and how to measure them from Kinect data.
    /// </summary>
    private void InitializeBoneMappingConfigs()
    {
        // Y-axis is the standard stretch axis for Unity humanoid bones
        Vector3 yAxis = new Vector3(1f, 0f, 1f); // Scale only Y, keep X and Z at 1

        _BoneMappingConfigs = new List<BoneMappingConfig>
        {
            // Upper arms (bilateral) - Note: Kinect left = Unity right due to mirroring
            new BoneMappingConfig(Kinect.JointType.ShoulderLeft, Kinect.JointType.ElbowLeft, HumanBodyBones.RightUpperArm, yAxis),
            new BoneMappingConfig(Kinect.JointType.ShoulderRight, Kinect.JointType.ElbowRight, HumanBodyBones.LeftUpperArm, yAxis),
            
            // Lower arms (bilateral)
            new BoneMappingConfig(Kinect.JointType.ElbowLeft, Kinect.JointType.WristLeft, HumanBodyBones.RightLowerArm, yAxis),
            new BoneMappingConfig(Kinect.JointType.ElbowRight, Kinect.JointType.WristRight, HumanBodyBones.LeftLowerArm, yAxis),
            
            // Upper legs (bilateral)
            new BoneMappingConfig(Kinect.JointType.HipLeft, Kinect.JointType.KneeLeft, HumanBodyBones.RightUpperLeg, yAxis),
            new BoneMappingConfig(Kinect.JointType.HipRight, Kinect.JointType.KneeRight, HumanBodyBones.LeftUpperLeg, yAxis),
            
            // Lower legs (bilateral)
            new BoneMappingConfig(Kinect.JointType.KneeLeft, Kinect.JointType.FootLeft, HumanBodyBones.RightLowerLeg, yAxis),
            new BoneMappingConfig(Kinect.JointType.KneeRight, Kinect.JointType.FootRight, HumanBodyBones.LeftLowerLeg, yAxis),
        };

        Debug.Log($"Initialized {_BoneMappingConfigs.Count} bone mapping configurations.");
    }

    /// <summary>
    /// Phase 2.1: Measure current avatar bone lengths.
    /// CRITICAL: This must be called EVERY FRAME before calculating scale factors,
    /// so that we measure the bone lengths after the previous frame's scaling.
    /// This allows the system to correct overshooting or undershooting.
    /// </summary>
    private void MeasureCurrentAvatarBoneLengths()
    {
        if (animator == null)
        {
            Debug.LogError("Cannot measure avatar bone lengths: Animator is null.");
            return;
        }

        _CurrentAvatarBoneLengths.Clear();

        foreach (var config in _BoneMappingConfigs)
        {
            Transform boneTransform = animator.GetBoneTransform(config.unityBone);

            if (boneTransform == null)
            {
                continue;
            }

            // Measure bone length from the bone to its first child
            float boneLength = MeasureAvatarBoneLength(boneTransform);

            if (boneLength <= 0f)
            {
                continue;
            }

            _CurrentAvatarBoneLengths[config.unityBone] = boneLength;
        }
    }

    /// <summary>
    /// Helper method to measure the length of an avatar bone.
    /// Measures the distance from the bone to its first child using LOCAL space.
    /// CRITICAL: Must use local space because FakeUMA manipulates localScale and applies inverse scale to children.
    /// World space measurements would be corrupted by the inverse scale compensation.
    /// </summary>
    private float MeasureAvatarBoneLength(Transform bone)
    {
        if (bone == null || bone.childCount == 0)
        {
            return 0f;
        }

        Transform firstChild = bone.GetChild(0);
        if (firstChild == null)
        {
            return 0f;
        }

        // CRITICAL: Use local position of child, which represents the bone's actual length
        // The child's localPosition.y is the bone length along the Y-axis (the stretch axis)
        // We take the magnitude of the local position vector to get the actual bone length
        Vector3 childLocalPos = firstChild.localPosition;
        float distance = childLocalPos.magnitude;

        return distance;
    }

    // ========================================================================================
    // PHASE 3: MEASUREMENT AND SMOOTHING SYSTEM
    // ========================================================================================

    /// <summary>
    /// Phase 3.1: Measure the distance between two Kinect joints in REAL METERS (no scaling).
    /// CRITICAL: This uses RAW Kinect coordinates WITHOUT the ×10 scaling factor applied by GetVector3FromJoint().
    /// This ensures measurements are in real meters, matching the avatar's local space measurements (also in meters).
    /// Previously, using GetVector3FromJoint() applied ×10 scaling, causing scale factors to be ~10 instead of ~1.
    /// </summary>
    private float MeasureRawKinectBoneLength(Kinect.Body body, Kinect.JointType startJoint, Kinect.JointType endJoint)
    {
        if (body == null)
        {
            return 0f;
        }

        // Get RAW Kinect positions in meters (NO ×10 scaling!)
        var startJointData = body.Joints[startJoint];
        var endJointData = body.Joints[endJoint];

        Vector3 startPos = new Vector3(
            startJointData.Position.X,
            startJointData.Position.Y,
            startJointData.Position.Z
        );

        Vector3 endPos = new Vector3(
            endJointData.Position.X,
            endJointData.Position.Y,
            endJointData.Position.Z
        );

        // Calculate distance in real meters (matches avatar's local space)
        float distance = Vector3.Distance(startPos, endPos);

        return distance;
    }

    /// <summary>
    /// Phase 3.2: Apply exponential smoothing to Kinect bone length measurements.
    /// This reduces jitter while maintaining responsiveness to actual body movements.
    /// </summary>
    private float MeasureSmoothedKinectBoneLength(Kinect.Body body, Kinect.JointType startJoint, Kinect.JointType endJoint, HumanBodyBones boneId)
    {
        // Phase 3.3: Measurement quality checks
        if (body == null)
        {
            return GetPreviousSmoothedValue(boneId);
        }

        // Check tracking state of both joints
        var startTracking = body.Joints[startJoint].TrackingState;
        var endTracking = body.Joints[endJoint].TrackingState;

        if (startTracking != Kinect.TrackingState.Tracked || endTracking != Kinect.TrackingState.Tracked)
        {
            // Joints not fully tracked, use previous smoothed value
            return GetPreviousSmoothedValue(boneId);
        }

        // Get current raw measurement
        float rawMeasurement = MeasureRawKinectBoneLength(body, startJoint, endJoint);

        // Phase 3.3: Validate measurement is within reasonable bounds
        // Unity transformed space: typical bone lengths range from ~2-10 units for limbs
        // (Same coordinate system as height, which is ~170-200 units for adult)
        // if (rawMeasurement < 1.0f || rawMeasurement > 100.0f)
        // {
        //     // Measurement out of reasonable range, use previous value
        //     Debug.LogWarning($"[KINECT MEASUREMENT] {boneId}: OUT OF BOUNDS! Raw = {rawMeasurement:F4} (valid range: 1.0-100.0)");
        //     return GetPreviousSmoothedValue(boneId);
        // }

        // Phase 3.2: Apply exponential smoothing
        if (!_SmoothedKinectBoneLengths.ContainsKey(boneId))
        {
            // First measurement: store and return raw value
            _SmoothedKinectBoneLengths[boneId] = rawMeasurement;
            return rawMeasurement;
        }

        // Apply exponential moving average: smoothed = lerp(previous, current, alpha)
        float previousSmoothed = _SmoothedKinectBoneLengths[boneId];
        float newSmoothed = Mathf.Lerp(previousSmoothed, rawMeasurement, boneSmoothingFactor);

        _SmoothedKinectBoneLengths[boneId] = newSmoothed;
        return newSmoothed;
    }

    /// <summary>
    /// Helper method to retrieve previous smoothed value or zero if none exists.
    /// </summary>
    private float GetPreviousSmoothedValue(HumanBodyBones boneId)
    {
        if (_SmoothedKinectBoneLengths.ContainsKey(boneId))
        {
            return _SmoothedKinectBoneLengths[boneId];
        }
        return 0f;
    }

    /// <summary>
    /// Returns the fixed T-pose bone length (in raw meters) for a given Unity bone.
    /// Maps bilateral bones (left/right) to the averaged T-pose measurement from MultiSourceManager.
    /// </summary>
    private float GetTPoseBoneLength(HumanBodyBones bone)
    {
        if (_MultiSourceManager == null) return -1f;

        switch (bone)
        {
            case HumanBodyBones.LeftUpperArm:
            case HumanBodyBones.RightUpperArm:
                return _MultiSourceManager.MeasuredUpperArmLength;

            case HumanBodyBones.LeftLowerArm:
            case HumanBodyBones.RightLowerArm:
                return _MultiSourceManager.MeasuredLowerArmLength;

            case HumanBodyBones.LeftUpperLeg:
            case HumanBodyBones.RightUpperLeg:
                return _MultiSourceManager.MeasuredUpperLegLength;

            case HumanBodyBones.LeftLowerLeg:
            case HumanBodyBones.RightLowerLeg:
                return _MultiSourceManager.MeasuredLowerLegLength;

            default:
                return -1f;
        }
    }

    // ========================================================================================
    // PHASE 4: SCALE FACTOR CALCULATION
    // ========================================================================================

    /// <summary>
    /// Phase 4.1: Calculate the scale factor needed to match Kinect bone length to avatar bone length.
    /// Includes safety clamping to prevent extreme scaling.
    /// </summary>
    private float CalculateBoneScaleFactor(float kinectBoneLength, float avatarBoneLength)
    {
        // Validate avatar bone length
        if (avatarBoneLength <= 0f)
        {
            Debug.LogWarning($"Invalid avatar bone length: {avatarBoneLength}. Cannot calculate scale factor.");
            return 1.0f;
        }

        if (kinectBoneLength <= 0f)
        {
            // No valid Kinect measurement yet
            Debug.LogWarning($"Invalid Kinect bone length: {kinectBoneLength}. Cannot calculate scale factor.");
            return 1.0f;
        }

        // Calculate ratio
        float scaleFactor = kinectBoneLength / avatarBoneLength;

        // Phase 4.1: Safety clamping to prevent extreme scaling
        // Using same range as uniform scale factor (typically 110-120x for this avatar)
        // Allow 50x-200x range to accommodate variations
        // scaleFactor = Mathf.Clamp(scaleFactor, 50.0f, 200.0f);
        Debug.Log($"[BONE SCALING] Calculated scale factor: Kinect Length = {kinectBoneLength:F4}, Avatar Length = {avatarBoneLength:F4}, Scale Factor = {scaleFactor:F4}");

        return scaleFactor;
    }

    // ========================================================================================
    // PHASE 5: INTEGRATION INTO UPDATE LOOP
    // ========================================================================================

    /// <summary>
    /// Phase 5.2: Main scaling update method called every frame to apply smoothed bone scaling.
    /// CRITICAL: Processes bones in HIERARCHICAL ORDER (parent before children) to ensure
    /// cumulative scale calculations are correct in a single pass.
    /// Now includes both length and thickness scaling based on statistical estimation.
    /// </summary>
    private void UpdateContinuousBoneScaling()
    {
        // Verify we have a valid tracked body
        if (!hasValidBody || trackedBody == null)
        {
            return;
        }

        // CRITICAL: Measure current avatar bone lengths BEFORE calculating scale factors
        // This ensures we compare against the CURRENT scaled lengths, not the original lengths
        // This allows the system to correct errors from previous frames
        MeasureCurrentAvatarBoneLengths();

        float thicknessFactor = 1.0f;
        if (enableThicknessScaling)
        {
            float rawBuildFactor = EstimateBodyBuildFactor();
            thicknessFactor = rawBuildFactor;
        }

        // CRITICAL FIX: Sort bones in hierarchical order (parent before children)
        // This ensures parent scales are applied before children calculate cumulative parent scales
        var sortedConfigs = SortBonesInHierarchicalOrder(_BoneMappingConfigs);

        // Process each bone in HIERARCHICAL ORDER using HYBRID SCALING MODEL:
        // - LENGTH: via child bone translation (zero shear at joints)
        // - THICKNESS: via localScale with rotation-aware inverse (minimal shear)
        foreach (var config in sortedConfigs)
        {
            // Check if we have a current measurement for this bone
            if (!_CurrentAvatarBoneLengths.ContainsKey(config.unityBone))
            {
                Debug.LogWarning($"[BONE SCALING] SKIPPED {config.unityBone}: No current measurement in dictionary");
                continue;
            }

            // Use fixed T-pose bone length from MultiSourceManager (no per-frame Kinect jitter)
            float kinectLength = GetTPoseBoneLength(config.unityBone);

            if (kinectLength <= 0f)
            {
                // No valid measurement yet
                Debug.LogWarning($"[BONE SCALING] SKIPPED {config.unityBone}: Invalid Kinect length ({kinectLength})");
                continue;
            }

            // Phase 2.1: Retrieve CURRENT avatar bone length (measured this frame)
            float avatarLength = _CurrentAvatarBoneLengths[config.unityBone];

            // Phase 4.1: Calculate DESIRED length scale factor
            float desiredLengthScale = CalculateBoneScaleFactor(kinectLength, avatarLength);

            // Get the bone transform
            Transform boneTransform = animator.GetBoneTransform(config.unityBone);
            if (boneTransform == null)
            {
                Debug.LogError($"[BONE SCALING] SKIPPED {config.unityBone}: Bone transform is NULL from animator!");
                continue;
            }

            // ============================================================
            // HYBRID STEP 1: Length via TRANSLATION (shear-free)
            // ============================================================
            // Move the child bone's localPosition to stretch the bone length.
            // This creates zero shear because translation doesn't interact with
            // the scale matrix — vertices stretch naturally through blended weights.
            fakeUMA.ScaleBoneLengthViaTranslation(config.unityBone, desiredLengthScale);

            // ============================================================
            // HYBRID STEP 2: Thickness via SCALE (rotation-aware inverse)
            // ============================================================
            // Only X and Z axes are scaled (thickness). Y stays at 1.0 (no length in scale).
            // This means the inverse scale compensator only has to fight minor X/Z differences,
            // making visual artifacts almost imperceptible.
            Vector3 parentCumulativeScale = GetParentCumulativeScale(boneTransform);

            float boneThicknessFactor = enableThicknessScaling
                ? GetLimbThicknessFactor(config.unityBone)
                : 1.0f;

            // Desired world scale: thickness on X/Z, uniform (no stretch) on Y
            Vector3 desiredWorldScale = new Vector3(
                boneThicknessFactor * UniformScaleFactor,  // Thickness X
                UniformScaleFactor,                         // Y = uniform only (length handled by translation)
                boneThicknessFactor * UniformScaleFactor   // Thickness Z
            );

            // Required local scale = desiredWorldScale / parentCumulativeScale
            Vector3 requiredLocalScale = new Vector3(
                desiredWorldScale.x / parentCumulativeScale.x,
                desiredWorldScale.y / parentCumulativeScale.y,
                desiredWorldScale.z / parentCumulativeScale.z
            );

            // Get current local scale from FakeUMA database
            Vector3 currentRelativeScale = fakeUMA.GetBoneScaleFactor(boneTransform);

            // Calculate the scale factor to apply (relative to current)
            Vector3 scaleFactorToApply = new Vector3(
                currentRelativeScale.x != 0 ? requiredLocalScale.x / currentRelativeScale.x : 1f,
                currentRelativeScale.y != 0 ? requiredLocalScale.y / currentRelativeScale.y : 1f,
                currentRelativeScale.z != 0 ? requiredLocalScale.z / currentRelativeScale.z : 1f
            );

            // Apply thickness-only scale (with rotation-aware inverse to children)
            fakeUMA.ScaleBoneIndependently(boneTransform, scaleFactorToApply);
        }

        // Apply thickness-only scaling to spine/torso bones
        if (enableThicknessScaling)
        {
            ApplySpineThicknessScaling(thicknessFactor); // *1.2 Based on experimental results for torso scaling
        }
    }

    /// <summary>
    /// Applies thickness-only scaling to spine bones (Hips, Spine, Chest, UpperChest).
    /// These bones don't have clear Kinect length measurements, so we only scale their width/depth
    /// based on the body build factor from depth measurement.
    /// </summary>
    /// <param name="thicknessFactor">The body build factor from depth measurement</param>
    private void ApplySpineThicknessScaling(float thicknessFactor)
    {
        foreach (HumanBodyBones spineBone in _SpineThicknessBones)
        {
            Transform boneTransform = animator.GetBoneTransform(spineBone);
            if (boneTransform == null)
            {
                continue;
            }

            // Calculate parent's cumulative scale
            Vector3 parentCumulativeScale = GetParentCumulativeScale(boneTransform);

            // For spine bones: apply thickness to X and Z, keep Y at uniform scale (no length change)
            // This makes the torso wider/thinner without stretching it vertically
            Vector3 desiredWorldScale = new Vector3(
                thicknessFactor * UniformScaleFactor,  // Width (X)
                UniformScaleFactor,                     // Height (Y) - no change
                thicknessFactor * UniformScaleFactor * 0.75f   // Depth (Z) -> doesn't change as much as width, but we can scale it slightly for better proportions
            );

            // Required local scale = desiredWorldScale / parentCumulativeScale
            Vector3 requiredLocalScale = new Vector3(
                desiredWorldScale.x / parentCumulativeScale.x,
                desiredWorldScale.y / parentCumulativeScale.y,
                desiredWorldScale.z / parentCumulativeScale.z
            );

            // Get current local scale from FakeUMA database
            Vector3 currentRelativeScale = fakeUMA.GetBoneScaleFactor(boneTransform);

            // Calculate the scale factor to apply (relative to current)
            Vector3 scaleFactorToApply = new Vector3(
                currentRelativeScale.x != 0 ? requiredLocalScale.x / currentRelativeScale.x : 1f,
                currentRelativeScale.y != 0 ? requiredLocalScale.y / currentRelativeScale.y : 1f,
                currentRelativeScale.z != 0 ? requiredLocalScale.z / currentRelativeScale.z : 1f
            );

            // Scale down slightly for chest and upper chest to prevent extreme widening, based on experimental results
            if (spineBone == HumanBodyBones.Chest || spineBone == HumanBodyBones.UpperChest)
            {
                scaleFactorToApply.x = 1.0f; // Don't scale in x, else we push arms outwards
            }

            // Apply the calculated scale factor
            fakeUMA.ScaleBoneIndependently(boneTransform, scaleFactorToApply);
        }
    }

    /// <summary>
    /// Calculates the cumulative scale of all parent transforms in the hierarchy,
    /// projected onto the bone's own local axes to account for rotations.
    /// 
    /// The naive approach (component-wise multiplication of parent localScales) assumes
    /// all bones share the same axis orientation. This breaks for arm bones, which are
    /// rotated ~90° relative to the spine — causing the spine's X/Z asymmetry to incorrectly
    /// map onto the arm's cross-section (one thickness axis gets a different scale than the other).
    /// 
    /// This rotation-aware version uses the parent's lossyScale (world-space scale) and
    /// projects it onto the bone's local axes via directional stretch, so each axis of
    /// the bone sees the correct inherited scale regardless of the rotation chain.
    /// </summary>
    /// <param name="bone">The bone whose parent cumulative scale to calculate</param>
    /// <returns>The cumulative scale vector of all parents, in the bone's local frame</returns>
    private Vector3 GetParentCumulativeScale(Transform bone)
    {
        if (bone == null || bone.parent == null)
        {
            return Vector3.one;
        }

        // Parent's world-space scale (accounts for all rotations in the chain)
        Vector3 parentLossyScale = bone.parent.lossyScale;

        // Get the bone's local axes expressed in world space
        // bone.rotation = parentChainRotation * localRotation
        Vector3 boneXWorld = bone.rotation * Vector3.right;
        Vector3 boneYWorld = bone.rotation * Vector3.up;
        Vector3 boneZWorld = bone.rotation * Vector3.forward;

        // For each bone-local axis, compute how much the parent chain stretches it.
        // Treat lossyScale as a diagonal scale matrix S in world space.
        // Stretch along direction d = |S * d| = sqrt((Sx*dx)² + (Sy*dy)² + (Sz*dz)²)
        float scaleX = ComputeWorldDirectionalStretch(parentLossyScale, boneXWorld);
        float scaleY = ComputeWorldDirectionalStretch(parentLossyScale, boneYWorld);
        float scaleZ = ComputeWorldDirectionalStretch(parentLossyScale, boneZWorld);

        return new Vector3(scaleX, scaleY, scaleZ);
    }

    /// <summary>
    /// Computes how much a world-space scale stretches a particular direction.
    /// For a unit direction d and diagonal scale S, the stretch is |S * d|.
    /// </summary>
    private float ComputeWorldDirectionalStretch(Vector3 scale, Vector3 direction)
    {
        Vector3 scaled = new Vector3(
            scale.x * direction.x,
            scale.y * direction.y,
            scale.z * direction.z
        );
        return scaled.magnitude;
    }

    /// <summary>
    /// Sorts bone mapping configurations in hierarchical order (parent before children).
    /// Uses depth-first pre-order traversal to ensure parents are processed before their children.
    /// This is CRITICAL for correct cumulative scale calculations in a single pass.
    /// </summary>
    private List<BoneMappingConfig> SortBonesInHierarchicalOrder(List<BoneMappingConfig> configs)
    {
        // Create a list with bone configs and their hierarchy depths
        var configsWithDepth = new List<(BoneMappingConfig config, int depth)>();

        foreach (var config in configs)
        {
            Transform boneTransform = animator.GetBoneTransform(config.unityBone);
            if (boneTransform != null)
            {
                int depth = GetBoneHierarchyDepth(boneTransform);
                configsWithDepth.Add((config, depth));
            }
        }

        // Sort by depth (shallower/parent bones first)
        configsWithDepth.Sort((a, b) => a.depth.CompareTo(b.depth));

        // Extract sorted configs
        return configsWithDepth.Select(x => x.config).ToList();
    }

    /// <summary>
    /// Calculates the hierarchy depth of a bone (distance from root).
    /// Root has depth 0, its children have depth 1, etc.
    /// </summary>
    private int GetBoneHierarchyDepth(Transform bone)
    {
        int depth = 0;
        Transform current = bone;

        // Count parent transforms until we reach root
        while (current != null && current.parent != null)
        {
            depth++;
            current = current.parent;
        }

        return depth;
    }

    // ========================================================================================
    // STATISTICAL THICKNESS SCALING SYSTEM
    // ========================================================================================

    /// <summary>
    /// Calculates body build (thickness) factor from T-pose SpineMid width measurement.
    /// Compares the user's actual torso width against the avatar's expected width at that height.
    /// </summary>
    private float EstimateBodyBuildFactor()
    {
        if (_MultiSourceManager == null || _MultiSourceManager.MeasuredSpineMidWidth <= 0f
            || _MultiSourceManager.MeasuredHeight <= 0f)
        {
            return 1.0f;
        }

        float expectedWidth = AVATAR_WIDTH_HEIGHT_RATIO * _MultiSourceManager.MeasuredHeight;
        float buildFactor = _MultiSourceManager.MeasuredSpineMidWidth / expectedWidth;
        Debug.Log($"[THICKNESS SCALING] Measured SpineMid Width = {_MultiSourceManager.MeasuredSpineMidWidth:F4}, Expected Width = {expectedWidth:F4}, Build Factor = {buildFactor:F4}");
        return buildFactor;
    }

    /// <summary>
    /// Computes a per-segment thickness scale factor directly from the measured waist width.
    /// userLimbWidth  = userWaistWidth  / POPULATION_RATIO  (estimated from population data)
    /// avatarLimbWidth = avatarWaistWidth / POPULATION_RATIO  (avatar's expected limb at this scale)
    /// factor = userLimbWidth / avatarLimbWidth
    /// Replace the denominator with a directly measured avatar limb width for per-segment tuning.
    /// </summary>
    private float GetLimbThicknessFactor(HumanBodyBones bone)
    {
        float userWaist  = _MultiSourceManager.MeasuredSpineMidWidth;
        float avatarWaist = AVATAR_WIDTH_HEIGHT_RATIO * _MultiSourceManager.MeasuredHeight;

        switch (bone)
        {
            case HumanBodyBones.LeftUpperArm:
            case HumanBodyBones.RightUpperArm:
                return (userWaist / POPULATION_WAIST_ARM_RATIO) / (avatarWaist / POPULATION_WAIST_ARM_RATIO);

            case HumanBodyBones.LeftLowerArm:
            case HumanBodyBones.RightLowerArm:
                return (userWaist / POPULATION_WAIST_FOREARM_RATIO) / (avatarWaist / POPULATION_WAIST_FOREARM_RATIO);

            case HumanBodyBones.LeftUpperLeg:
            case HumanBodyBones.RightUpperLeg:
                return (userWaist / POPULATION_WAIST_THIGH_RATIO) / (avatarWaist / POPULATION_WAIST_THIGH_RATIO);

            case HumanBodyBones.LeftLowerLeg:
            case HumanBodyBones.RightLowerLeg:
                return (userWaist / POPULATION_WAIST_CALF_RATIO) / (avatarWaist / POPULATION_WAIST_CALF_RATIO);

            default:
                return userWaist / avatarWaist;
        }
    }

    // ========================================================================================
    // RESTART
    // ========================================================================================

    /// <summary>
    /// Resets all runtime measurement and scaling state so a new user can be calibrated.
    /// Called by UI "Restart" button. Does NOT re-initialize infrastructure (FakeUMA, bone configs, etc.).
    /// </summary>
    public void Restart()
    {
        // Reset uniform scaling
        UniformScaleFactor = -1f;
        ClothedBaseAvatar.transform.localScale = Vector3.one;

        // Reset all bone scales to original FBX values
        fakeUMA.ResetAllBoneScales();

        // Clear smoothed measurement history
        _SmoothedKinectBoneLengths.Clear();

        // Clear user measurements
        _KinectUserMeasurements = new HumanoidMeasurements();

        // Re-trigger T-pose measurement
        if (_MultiSourceManager != null)
        {
            _MultiSourceManager.StartMeasurement();
        }
    }
}

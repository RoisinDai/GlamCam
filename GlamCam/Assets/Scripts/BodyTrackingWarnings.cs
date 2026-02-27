using UnityEngine;
using System.Collections.Generic;
using Kinect = Windows.Kinect;

/// <summary>
/// Monitors Kinect body tracking quality and toggles a warning GameObject when:
/// 1. User is rotated too far from facing the camera
/// 2. Limbs are not visible/tracked (out of frame or occluded)
/// 
/// Attach to any GameObject. Assign the Warning object under MainRig to toggle.
/// </summary>
public class BodyTrackingWarnings : MonoBehaviour
{
    [Header("References")]
    [Tooltip("Reference to the BodySourceManager to get Kinect data")]
    public GameObject bodySourceManager;

    [Tooltip("The Warning GameObject under MainRig to show/hide")]
    public GameObject warningObject;

    [Header("Detection Settings")]
    [Tooltip("Maximum allowed rotation angle (degrees) before showing warning")]
    [Range(15f, 60f)]
    public float maxRotationAngle = 50f;

    public bool requireArms = true; // require arms to be tracked

    public bool requireLegs = true; // require legs to be tracked

    [Header("Timing")]
    [Tooltip("How long a warning condition must persist before showing (seconds)")]
    public float warningDelaySeconds = 0.5f;

    [Tooltip("How long after condition clears before hiding warning (seconds)")]
    public float clearDelaySeconds = 0.3f;

    // State
    private bool _showingWarning = false;
    private float _warningTimer = 0f;
    private float _clearTimer = 0f;

    private BodySourceManager _bodyManager;
    private InitializationStateMachine _initStateMachine;

    private ClothingRulesCoordinator _clothingCoordinator;

    // Joints to check for limb tracking
    private static readonly Kinect.JointType[] _ArmJoints = new Kinect.JointType[]
    {
        Kinect.JointType.ShoulderLeft, Kinect.JointType.ElbowLeft, Kinect.JointType.WristLeft,
        Kinect.JointType.ShoulderRight, Kinect.JointType.ElbowRight, Kinect.JointType.WristRight
    };

    private static readonly Kinect.JointType[] _LegJoints = new Kinect.JointType[]
    {
        Kinect.JointType.HipLeft, Kinect.JointType.KneeLeft, Kinect.JointType.AnkleLeft,
        Kinect.JointType.HipRight, Kinect.JointType.KneeRight, Kinect.JointType.AnkleRight
    };

    // Public property for external scripts
    public bool HasWarning => _showingWarning;

    void Start()
    {
        // Find InitializationStateMachine
        _initStateMachine = FindObjectOfType<InitializationStateMachine>();
        if (_initStateMachine == null)
        {
            Debug.LogWarning("[BodyTrackingWarnings] InitializationStateMachine not found - warnings will show in all phases.");
        }

        _clothingCoordinator = FindObjectOfType<ClothingRulesCoordinator>();
        if (_clothingCoordinator == null)
        {
            Debug.LogWarning("[BodyTrackingWarnings] ClothingRulesCoordinator not found - clothing will not be cleared on warning.");
        }

        // Find BodySourceManager
        if (bodySourceManager != null)
        {
            _bodyManager = bodySourceManager.GetComponent<BodySourceManager>();
        }

        if (_bodyManager == null)
        {
            _bodyManager = FindObjectOfType<BodySourceManager>();
        }

        if (_bodyManager == null)
        {
            Debug.LogError("[BodyTrackingWarnings] BodySourceManager not found!");
        }

        // Auto-find warning object if not assigned
        if (warningObject == null)
        {
            var mainRig = GameObject.Find("MainRig");
            if (mainRig != null)
            {
                var warningTransform = mainRig.transform.Find("Warning");
                if (warningTransform != null)
                {
                    warningObject = warningTransform.gameObject;
                    Debug.Log("[BodyTrackingWarnings] Auto-found Warning object under MainRig");
                }
            }
        }

        // Start with warning hidden
        SetWarningVisible(false);
    }

    void Update()
    {
        if (_bodyManager == null) return;

        // Only show warnings during Main phase
        if (_initStateMachine != null && _initStateMachine.CurrentState != InitializationStateMachine.State.Main)
        {
            // Not in Main phase - hide warning and reset timers
            SetWarningVisible(false);
            _warningTimer = 0f;
            _clearTimer = 0f;
            return;
        }

        // Get the tracked body
        Kinect.Body trackedBody = GetTrackedBody();
        if (trackedBody == null)
        {
            // No body tracked - hide warning
            HandleWarningState(false);
            return;
        }

        // Check if there's any problem (rotation OR limbs not tracked)
        bool hasProblem = HasTrackingProblem(trackedBody);

        // Handle warning state with delays to prevent flickering
        HandleWarningState(hasProblem);
    }

    private Kinect.Body GetTrackedBody()
    {
        // Use the same tracked body as AvatarController if available
        if (AvatarController.trackedBody != null && AvatarController.trackedBody.IsTracked)
        {
            return AvatarController.trackedBody;
        }

        // Otherwise find any tracked body
        Kinect.Body[] bodies = _bodyManager.GetData();
        if (bodies == null) return null;

        foreach (var body in bodies)
        {
            if (body != null && body.IsTracked)
            {
                return body;
            }
        }

        return null;
    }

    /// <summary>
    /// Checks if there's any tracking problem (rotation or limbs not tracked).
    /// </summary>
    private bool HasTrackingProblem(Kinect.Body body)
    {
        var joints = body.Joints;

        // Check rotation
        if (IsBodyRotatedTooFar(joints))
        {
            return true;
        }

        // Check limb tracking
        if (!AreLimbsTracked(joints))
        {
            return true;
        }

        return false;
    }

    /// <summary>
    /// Detects if the user's body is rotated too far from facing the camera.
    /// Uses the Z-depth difference between shoulders to estimate rotation.
    /// </summary>
    private bool IsBodyRotatedTooFar(IReadOnlyDictionary<Kinect.JointType, Kinect.Joint> joints)
    {
        var leftShoulder = joints[Kinect.JointType.ShoulderLeft];
        var rightShoulder = joints[Kinect.JointType.ShoulderRight];

        // Need both shoulders tracked
        if (leftShoulder.TrackingState == Kinect.TrackingState.NotTracked ||
            rightShoulder.TrackingState == Kinect.TrackingState.NotTracked)
        {
            return false; // Can't determine rotation, don't warn
        }

        // Calculate Z difference (depth) between shoulders
        float zDiff = Mathf.Abs(leftShoulder.Position.Z - rightShoulder.Position.Z);

        // Calculate X difference (width) between shoulders
        float xDiff = Mathf.Abs(leftShoulder.Position.X - rightShoulder.Position.X);

        // If shoulders have very small X separation, person might be rotated sideways
        if (xDiff < 0.05f)
        {
            return true; // Shoulders nearly overlapping in X = rotated ~90 degrees
        }

        // Calculate rotation angle from shoulder positions
        // tan(angle) = zDiff / xDiff
        float rotationAngle = Mathf.Atan2(zDiff, xDiff) * Mathf.Rad2Deg;

        return rotationAngle > maxRotationAngle;
    }

    /// <summary>
    /// Checks if the required limbs are being tracked.
    /// </summary>
    private bool AreLimbsTracked(IReadOnlyDictionary<Kinect.JointType, Kinect.Joint> joints)
    {
        // Check arms
        if (requireArms)
        {
            int trackedArmJoints = 0;
            foreach (var jointType in _ArmJoints)
            {
                if (joints[jointType].TrackingState == Kinect.TrackingState.Tracked)
                {
                    trackedArmJoints++;
                }
            }

            // Require at least 4 of 6 arm joints to be tracked (allows some tolerance)
            if (trackedArmJoints < 4)
            {
                return false;
            }
        }

        // Check legs
        if (requireLegs)
        {
            int trackedLegJoints = 0;
            foreach (var jointType in _LegJoints)
            {
                if (joints[jointType].TrackingState == Kinect.TrackingState.Tracked)
                {
                    trackedLegJoints++;
                }
            }

            // Require at least 4 of 6 leg joints to be tracked
            if (trackedLegJoints < 4)
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>
    /// Handles warning state with delays to prevent flickering.
    /// </summary>
    private void HandleWarningState(bool hasProblem)
    {
        if (hasProblem)
        {
            // Problem detected
            _clearTimer = 0f;

            if (!_showingWarning)
            {
                // Not currently showing - wait for delay
                _warningTimer += Time.deltaTime;

                if (_warningTimer >= warningDelaySeconds)
                {
                    SetWarningVisible(true);
                }
            }
        }
        else
        {
            // No problem
            _warningTimer = 0f;

            if (_showingWarning)
            {
                // Currently showing - wait for clear delay
                _clearTimer += Time.deltaTime;

                if (_clearTimer >= clearDelaySeconds)
                {
                    SetWarningVisible(false);
                }
            }
        }
    }

    private bool _savedClothing = false;

    private void SetWarningVisible(bool visible)
    {
        if (visible && !_showingWarning)
        {
            // record current clothing before hiding it
            _clothingCoordinator?.SaveCurrentSelections();
            ClothingRulesCoordinator.ClearAllClothing();
            _savedClothing = true;
        }
        else if (!visible && _showingWarning)
        {
            // when warning clears, restore garments we hid earlier
            if (_savedClothing)
            {
                _clothingCoordinator?.RestoreSavedSelections();
                _savedClothing = false;
            }
        }

        _showingWarning = visible;

        if (warningObject != null)
        {
            warningObject.SetActive(visible);
        }
    }

    /// <summary>
    /// Public method to temporarily disable warnings.
    /// </summary>
    public void SetWarningsEnabled(bool enabled)
    {
        this.enabled = enabled;
        if (!enabled)
        {
            SetWarningVisible(false);
        }
    }
}
using System.Linq;
using UnityEngine;
using Kinect = Windows.Kinect;
using Vector3 = UnityEngine.Vector3;


// Responsible for controlling the clothed base avatar, making it track the user's body
public class AvatarController : MonoBehaviour
{
  public Animator animator;
  public GameObject BodySourceManager;
  private BodySourceManager _BodyManager;
  public GameObject ClothedBaseAvatar;
  private Kinect.Body trackedBody; // The body being tracked by the avatar
  public bool enableInverseKinematics = true;

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
  }

  // A callback function to calculate inverse kinematics
  private void OnAnimatorIK(int layerIndex)
  {
    Debug.Log("OnAnimatorIK called with layer: " + layerIndex);
    if (animator == null || trackedBody == null || !enableInverseKinematics) return;

    // Move hands to their goals
    ApplyIK(Kinect.JointType.HandLeft, AvatarIKGoal.LeftHand);
    ApplyIK(Kinect.JointType.HandRight, AvatarIKGoal.RightHand);
  }
    

  // Apply inverse kinematics to the avatar's joints to move them to the goal
  private void ApplyIK(Kinect.JointType joint, AvatarIKGoal goal)
  {
    var kinectJoint = trackedBody.Joints[joint];
    if (kinectJoint.TrackingState == Kinect.TrackingState.NotTracked) return; // Avoid phantom movements

    var kinectJointPos = trackedBody.Joints[joint].Position;
    Vector3 unityPos = BodySourceView.GetVector3FromKinectCoord(kinectJointPos.X, kinectJointPos.Y, kinectJointPos.Z);

    animator.SetIKPositionWeight(goal, 1f);
    animator.SetIKPosition(goal, unityPos);
  }
}

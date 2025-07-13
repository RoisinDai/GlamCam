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

      animator.SetLookAtWeight(1f);
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

    animator.SetIKPositionWeight(goal, 1f);
    animator.SetIKPosition(goal, unityPos);
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
}

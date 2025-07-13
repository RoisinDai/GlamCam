using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Kinect = Windows.Kinect;
using Vector3 = UnityEngine.Vector3;

class UserMeasurements
{
  public float height;
  public float shoulderWidth;
  public float armLength;
}

// Responsible for drawing the 'skeleton' for each frame by 
// connecting the joints from the BodySourceManager.
public class BodySourceView : MonoBehaviour
{
  public Material BoneMaterial;
  public GameObject BodySourceManager;
  public GameObject ClothedBaseAvatar;
  private GameObject UnclothedBaseAvatar;
  private GameObject Armature;
  private float BaseAvatarHeight;
  private const string BASE_AVATAR_NAME = "BaseAvatar";
  private const string ARMATURE = "Armature";
  private Dictionary<ulong, GameObject> _Bodies = new Dictionary<ulong, GameObject>();
  private BodySourceManager _BodyManager;

  private UserMeasurements _UserMeasurements = new UserMeasurements();

  // Maps joints to the joint they are connected to
  private Dictionary<Kinect.JointType, Kinect.JointType> _BoneMap = new Dictionary<Kinect.JointType, Kinect.JointType>()
    {
        // Lower left body
        { Kinect.JointType.FootLeft, Kinect.JointType.AnkleLeft },
        { Kinect.JointType.AnkleLeft, Kinect.JointType.KneeLeft },
        { Kinect.JointType.KneeLeft, Kinect.JointType.HipLeft },
        { Kinect.JointType.HipLeft, Kinect.JointType.SpineBase },
        
        // Lower right body
        { Kinect.JointType.FootRight, Kinect.JointType.AnkleRight },
        { Kinect.JointType.AnkleRight, Kinect.JointType.KneeRight },
        { Kinect.JointType.KneeRight, Kinect.JointType.HipRight },
        { Kinect.JointType.HipRight, Kinect.JointType.SpineBase },
        
        // Left hand, arm, shoulder
        { Kinect.JointType.HandTipLeft, Kinect.JointType.HandLeft },
        { Kinect.JointType.ThumbLeft, Kinect.JointType.HandLeft },
        { Kinect.JointType.HandLeft, Kinect.JointType.WristLeft },
        { Kinect.JointType.WristLeft, Kinect.JointType.ElbowLeft },
        { Kinect.JointType.ElbowLeft, Kinect.JointType.ShoulderLeft },
        { Kinect.JointType.ShoulderLeft, Kinect.JointType.SpineShoulder },
        
        // Right hand, arm, shoulder
        { Kinect.JointType.HandTipRight, Kinect.JointType.HandRight },
        { Kinect.JointType.ThumbRight, Kinect.JointType.HandRight },
        { Kinect.JointType.HandRight, Kinect.JointType.WristRight },
        { Kinect.JointType.WristRight, Kinect.JointType.ElbowRight },
        { Kinect.JointType.ElbowRight, Kinect.JointType.ShoulderRight },
        { Kinect.JointType.ShoulderRight, Kinect.JointType.SpineShoulder },
        
        // Spine, neck, head
        { Kinect.JointType.SpineBase, Kinect.JointType.SpineMid },
        { Kinect.JointType.SpineMid, Kinect.JointType.SpineShoulder },
        { Kinect.JointType.SpineShoulder, Kinect.JointType.Neck },
        { Kinect.JointType.Neck, Kinect.JointType.Head },
    };

  void Start()
  {
    if (ClothedBaseAvatar == null)
    {
      Debug.LogError("ClothedBaseAvatar is STILL null at runtime!");
      return;
    }
    Debug.Log("Clothed Base Avatar: " + ClothedBaseAvatar.name);

    // Set the unclothed base avatar
    UnclothedBaseAvatar = ClothedBaseAvatar.transform.Find(BASE_AVATAR_NAME)?.gameObject;
    Debug.Log("Unclothed Base Avatar: " + UnclothedBaseAvatar.name);

    // Set the armature
    Armature = ClothedBaseAvatar.transform.Find(ARMATURE)?.gameObject;
    Debug.Log("Armature: " + Armature.name);

    // Get height of avatar
    BaseAvatarHeight = GetBaseAvatarHeight(Armature);
    Debug.Log("Avatar height (in Unity 1x): " + BaseAvatarHeight);
  }

  // Updates the body objects
  void Update()
  {
    if (BodySourceManager == null)
    {
      return;
    }

    _BodyManager = BodySourceManager.GetComponent<BodySourceManager>();
    if (_BodyManager == null)
    {
      return;
    }

    Kinect.Body[] data = _BodyManager.GetData();
    if (data == null)
    {
      return;
    }

    // Get the identifiers of the bodies currently in view (tracked)
    List<ulong> trackedIds = new List<ulong>();
    foreach (var body in data)
    {
      if (body == null)
      {
        continue;
      }

      if (body.IsTracked)
      {
        trackedIds.Add(body.TrackingId);
      }
    }

    // Get the identifiers of the bodies that have been seen before
    List<ulong> knownIds = new List<ulong>(_Bodies.Keys);

    // First delete untracked bodies
    foreach (ulong trackingId in knownIds)
    {
      if (!trackedIds.Contains(trackingId))
      {
        Destroy(_Bodies[trackingId]);
        _Bodies.Remove(trackingId);
      }
    }

    foreach (var body in data)
    {
      if (body == null)
      {
        continue;
      }

      // If the body is tracked, create a new body object if it doesn't exist
      // and refresh the body object
      if (body.IsTracked)
      {
        if (!_Bodies.ContainsKey(body.TrackingId))
        {
          _Bodies[body.TrackingId] = CreateBodyObject(body.TrackingId);
        }

        RefreshBodyObject(body, _Bodies[body.TrackingId]);

        // Compute measurements of the person's body WRT Unity World space
        var joints = body.Joints;
        // Vector3 shoulderLeft = GetVector3FromJoint(joints[Kinect.JointType.ShoulderLeft]);
        // Vector3 shoulderRight = GetVector3FromJoint(joints[Kinect.JointType.ShoulderRight]);
        Vector3 head = GetVector3FromJoint(joints[Kinect.JointType.Head]);
        Vector3 footLeft = GetVector3FromJoint(joints[Kinect.JointType.FootLeft]);
        Vector3 footRight = GetVector3FromJoint(joints[Kinect.JointType.FootRight]);
        // Vector3 elbowLeft = GetVector3FromJoint(joints[Kinect.JointType.ElbowLeft]);
        // Vector3 wristLeft = GetVector3FromJoint(joints[Kinect.JointType.WristLeft]);

        // Height
        _UserMeasurements.height = head.y - ((footLeft.y + footRight.y) / 2f); // average foot height
        Debug.Log("Height (Kinect 10x): " + _UserMeasurements.height.ToString("F3"));

        // Shoulder width
        // _UserMeasurements.shoulderWidth = Vector3.Distance(shoulderLeft, shoulderRight);
        // Debug.Log("Shoulder width (Kinect 10x): " + _UserMeasurements.shoulderWidth.ToString("F3"));

        // Arm length (Shoulder -> Elbow -> Wrist)
        // _UserMeasurements.armLength = Vector3.Distance(shoulderLeft, elbowLeft) + Vector3.Distance(elbowLeft, wristLeft);
        // Debug.Log("Arm length (Kinect 10x): " + _UserMeasurements.armLength.ToString("F3"));

        // Move the avatar to the base location of the skeleton (X and Z coordinates)
        Vector3 spineBase = GetVector3FromJoint(joints[Kinect.JointType.SpineBase]);
        // // ClothedBaseAvatar.transform.position = new Vector3(spineBase.x - 15, -5, spineBase.z);
        ClothedBaseAvatar.transform.position = new Vector3(spineBase.x, spineBase.y, spineBase.z);

        // Get the scaling factor to apply to the avatar
        float scaleFactor = 3f; // AvatarHeight*scaleFactor = UserHeight // TODO: Fix this scaling factor
        // float scaleFactor = _UserMeasurements.height / BaseAvatarHeight; // AvatarHeight*scaleFactor = UserHeight
        Debug.Log("User height (Unity units): " + _UserMeasurements.height);
        Debug.Log("Avatar base height: " + BaseAvatarHeight);
        Debug.Log("Scale factor based on height: " + scaleFactor);
        ClothedBaseAvatar.transform.localScale = new Vector3(scaleFactor, scaleFactor, scaleFactor);
      }
    }
  }

  // Create a new body object for the given tracking id
  private GameObject CreateBodyObject(ulong id)
  {
    GameObject body = new GameObject("Body:" + id);

    // Create a cube for each of the 20 joints
    for (Kinect.JointType jt = Kinect.JointType.SpineBase; jt <= Kinect.JointType.ThumbRight; jt++)
    {
      GameObject jointObj = GameObject.CreatePrimitive(PrimitiveType.Cube);

      LineRenderer lr = jointObj.AddComponent<LineRenderer>();
      lr.positionCount = 2;
      lr.material = BoneMaterial;
      lr.startWidth = 0.05f;
      lr.endWidth = 0.05f;


      jointObj.transform.localScale = new Vector3(0.3f, 0.3f, 0.3f);
      jointObj.name = jt.ToString();
      jointObj.transform.parent = body.transform;
    }

    return body;
  }

  // Update the position of each joint. Called for each frame.
  private void RefreshBodyObject(Kinect.Body body, GameObject bodyObject)
  {
    for (Kinect.JointType jt = Kinect.JointType.SpineBase; jt <= Kinect.JointType.ThumbRight; jt++)
    {
      Kinect.Joint sourceJoint = body.Joints[jt];
      Kinect.Joint? targetJoint = null;

      if (_BoneMap.ContainsKey(jt))
      {
        targetJoint = body.Joints[_BoneMap[jt]];
      }

      Transform jointObj = bodyObject.transform.Find(jt.ToString());
      jointObj.localPosition = GetVector3FromJoint(sourceJoint);

      LineRenderer lr = jointObj.GetComponent<LineRenderer>();
      if (targetJoint.HasValue)
      {
        // Connect the joint to its target joint
        lr.SetPosition(0, jointObj.localPosition);
        lr.SetPosition(1, GetVector3FromJoint(targetJoint.Value));

        // Set the color of the line based on the tracking state of source and target joints
        // Green: tracked, red: inferred, black: not tracked
        lr.startColor = GetColorForState(sourceJoint.TrackingState);
        lr.endColor = GetColorForState(targetJoint.Value.TrackingState);
      }
      else
      {
        lr.enabled = false;
      }
    }
  }

  private static Color GetColorForState(Kinect.TrackingState state)
  {
    switch (state)
    {
      case Kinect.TrackingState.Tracked:
        return Color.green;

      case Kinect.TrackingState.Inferred:
        return Color.red;

      default:
        return Color.black;
    }
  }

  // X: left to right
  // Y: down to up
  // Z: forward to back
  // Convert a position in Kinect Coordinate System (meters) to Unity Coordinate System (units)
  // Joint positions are scaled by *10, so the skeleton is drawn at 10x its true size.
  public static Vector3 GetVector3FromKinectCoord(float kinect_x, float kinect_y, float kinect_z)
  {
    return new Vector3(kinect_x * 10, kinect_y * 10, kinect_z * 10);
  }

  private static Vector3 GetVector3FromJoint(Kinect.Joint joint)
  {
    return GetVector3FromKinectCoord(joint.Position.X, joint.Position.Y, joint.Position.Z);
  }

  private float GetBaseAvatarHeight(GameObject armature)
  {
    Transform headTop = armature.transform.FindDeepChild("mixamorig:HeadTop_End");
    Transform footLeft = armature.transform.FindDeepChild("mixamorig:LeftToeBase");
    Transform footRight = armature.transform.FindDeepChild("mixamorig:RightToeBase");

    if (headTop == null || footLeft == null || footRight == null)
    {
      Debug.LogError("Could not find one or more required bones for avatar height calculation.");
      return 0f; // fallback to no scaling
    }

    float footAvgY = (footLeft.position.y + footRight.position.y) / 2f;
    return headTop.position.y - footAvgY;
  }
}

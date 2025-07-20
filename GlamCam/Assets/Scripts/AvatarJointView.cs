using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Kinect = Windows.Kinect;
using Vector3 = UnityEngine.Vector3;

// Responsible for drawing the joints for each frame
// in red.
public class JointView : MonoBehaviour
{
  public Material BoneMaterial;
  public GameObject BodySourceManager;
  public GameObject AvatarJointsCamera; // The camera that will view the joints
  public GameObject AvatarCamera; // The camera that will view the avatar
  public static GameObject ClothedAvatarHips; // The dressed avatar's hips
  private Dictionary<ulong, GameObject> _Bodies = new Dictionary<ulong, GameObject>();
  private BodySourceManager _BodyManager;
  private const int X_OFFSET = -40; // The X offset for where this joint view will be drawn

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

  // Colors map
  // Vibrant, distinct, non-green colors in hex
  Color[] jointColors = new Color[]
  {
    new Color(0.482f, 0.408f, 0.933f, 1f), // #7b68ee - Medium Slate Blue
    new Color(0.933f, 0.510f, 0.933f, 1f), // #ee82ee - Violet
    new Color(1.000f, 0.753f, 0.796f, 1f), // #ffc0cb - Pink
    new Color(1.000f, 0.078f, 0.576f, 1f), // #ff1493 - Deep Pink
    new Color(0.000f, 1.000f, 1.000f, 1f), // #00ffff - Cyan
    new Color(0.000f, 0.000f, 1.000f, 1f), // #0000ff - Blue
    new Color(0.863f, 0.078f, 0.235f, 1f), // #dc143c - Crimson
    new Color(0.541f, 0.169f, 0.886f, 1f), // #8a2be2 - Blue Violet
    new Color(0.859f, 0.439f, 0.576f, 1f), // #db7093 - Pale Violet Red
    new Color(0.545f, 0.000f, 0.545f, 1f), // #8b008b - Dark Magenta
    new Color(1.000f, 0.000f, 1.000f, 1f), // #ff00ff - Magenta
    new Color(1.000f, 0.271f, 0.000f, 1f), // #af2956ff - Red Pink
    new Color(0.941f, 1.000f, 1.000f, 1f), // #c7fefeff - Azure
    new Color(0.000f, 0.545f, 0.545f, 1f), // #008b8b - Dark Cyan
    new Color(0.282f, 0.239f, 0.545f, 1f), // #483d8b - Dark Slate Blue
    new Color(1.000f, 0.431f, 0.780f, 1f), // #ff6ec7 - Hot Pink
    new Color(1.000f, 0.361f, 0.361f, 1f), // #FF5C5C - Coral Red
    new Color(0.275f, 0.541f, 1.000f, 1f), // #468AFF - Sky Blue
    new Color(1.000f, 0.541f, 0.396f, 1f), // #FF8A65 - Light Orange
    new Color(0.910f, 0.451f, 1.000f, 1f), // #E873FF - Light Purple
    new Color(0.400f, 0.267f, 1.000f, 1f), // #6644FF - Royal Purple
    new Color(1.000f, 0.200f, 1.000f, 1f), // #FF33FF - Bright Magenta
    new Color(0.600f, 0.200f, 1.000f, 1f),  // #9933FF - Purple
    new Color(1.000f, 0.302f, 0.000f, 1f), // #ff4d00 - Orange Red
    new Color(0.310f, 0.318f, 0.694f, 1f),  // #6163e5ff - Dark Blue
  };

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

        // Move the AvatarJoints camera to the spine base position of the joint skeleton
        Vector3 spineBase = GetVector3FromJoint(body.Joints[Kinect.JointType.SpineBase]);
        AvatarJointsCamera.transform.position = new Vector3(spineBase.x, spineBase.y - 1f, spineBase.z - 40f);

        // Move the Avatar camera to the corresponding position for the dressed avatar
        AvatarCamera.transform.position = new Vector3(ClothedAvatarHips.transform.position.x, spineBase.y - 1f, spineBase.z - 40f);
      }
    }
  }

  // Create a new body object for the given tracking id
  private GameObject CreateBodyObject(ulong id)
  {
    GameObject body = new GameObject("Body:" + id);

    int jointIndex = 0;
    for (Kinect.JointType jt = Kinect.JointType.SpineBase; jt <= Kinect.JointType.ThumbRight; jt++)
    {
      GameObject jointObj = GameObject.CreatePrimitive(PrimitiveType.Cube);

      // Set color
      Color color = jointColors[jointIndex % jointColors.Length];
      jointObj.GetComponent<Renderer>().material.color = color;

      LineRenderer lr = jointObj.AddComponent<LineRenderer>();
      lr.positionCount = 2;
      lr.material = BoneMaterial;
      lr.startWidth = 0.05f;
      lr.endWidth = 0.05f;

      jointObj.transform.localScale = new Vector3(0.3f, 0.3f, 0.3f);
      jointObj.name = jt.ToString();
      jointObj.transform.parent = body.transform;

      jointIndex++;
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

        // Set the color of the line
        lr.startColor = Color.green;
        lr.endColor = Color.green;
      }
      else
      {
        lr.enabled = false;
      }
    }
  }

  // X: left to right
  // Y: down to up
  // Z: forward to back
  // Convert a position in Kinect Coordinate System (meters) to Unity Coordinate System (units)
  // Joint positions are scaled by *10, so the skeleton is drawn at 10x its true size.
  private static Vector3 GetVector3FromKinectCoord(float kinect_x, float kinect_y, float kinect_z)
  {
    return new Vector3(kinect_x * 10 + X_OFFSET, kinect_y * 10, kinect_z * 10); //
  }

  private static Vector3 GetVector3FromJoint(Kinect.Joint joint)
  {
    return GetVector3FromKinectCoord(joint.Position.X, joint.Position.Y, joint.Position.Z);
  }
}

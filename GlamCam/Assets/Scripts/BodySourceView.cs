using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Kinect = Windows.Kinect;
using Vector3 = UnityEngine.Vector3;

// Responsible for drawing the 'skeleton' for each frame by 
// connecting the joints from the BodySourceManager.
public class BodySourceView : MonoBehaviour
{
  public Material BoneMaterial;
  public GameObject BodySourceManager;
  public Material TransparentMaterial; // Material for joints skeleton to hide it
  private Dictionary<ulong, GameObject> _Bodies = new Dictionary<ulong, GameObject>();
  private BodySourceManager _BodyManager;
  private bool _ShowSkeleton = false; // Flag to toggle skeleton visibility

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

      if (_ShowSkeleton)
      {
        LineRenderer lr = jointObj.AddComponent<LineRenderer>();
        lr.positionCount = 2;
        lr.material = BoneMaterial;
        lr.startWidth = 0.05f;
        lr.endWidth = 0.05f;
      }
      else
      {
        // Apply a transparent material to the object
        jointObj.GetComponent<Renderer>().material = TransparentMaterial;
      }

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

  public static Vector3 GetVector3FromJoint(Kinect.Joint joint)
  {
    return GetVector3FromKinectCoord(joint.Position.X, joint.Position.Y, joint.Position.Z);
  }

}

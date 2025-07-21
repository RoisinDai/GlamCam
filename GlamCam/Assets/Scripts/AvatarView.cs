using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Kinect = Windows.Kinect;
using Vector3 = UnityEngine.Vector3;

// Responsible for controlling the camera
// pointing at the clothed avatar, which
// may or may not have the base avatar visible.
public class AvatarView : MonoBehaviour
{
  public GameObject AvatarCamera; // The camera that will view the avatar
  public GameObject ClothedAvatarHips; // The dressed avatar's hips

  // Updates the body objects
  void Update()
  {
    if (AvatarController.trackedBody == null || !AvatarController.trackedBody.IsTracked)
    {
      // No body currently being tracked by AvatarController
      return;
    }

    // Get the tracked body from AvatarController
    Kinect.Body body = AvatarController.trackedBody;

    // Move the Avatar camera to the spine base position of the joint skeleton
    Vector3 spineBase = BodySourceView.GetVector3FromJoint(body.Joints[Kinect.JointType.SpineBase]);
    AvatarCamera.transform.position = new Vector3(ClothedAvatarHips.transform.position.x, spineBase.y - 1f, spineBase.z - 40f);
  }
}

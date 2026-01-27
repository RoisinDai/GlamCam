using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Kinect = Windows.Kinect;
using Vector3 = UnityEngine.Vector3;

// Creates a 1:1 optical match between Unity's 3D space and the Kinect's video feed
// This allows avatars to appear correctly aligned over the user's body in the video background.
public class AvatarView : MonoBehaviour
{
  public Camera AvatarCamera;

  void Start()
  {
    // 1. Considering the Kinect Sensor to be at the "Origin" of the world,
    //    we place the AvatarCamera at the same location at (0,0,0), looking forward.
    AvatarCamera.transform.position = Vector3.zero;
    AvatarCamera.transform.rotation = Quaternion.identity;

    // 2. Configure the field of view (FOV) of the AvatarCamera
    // The Kinect v2 has a vertical FOV of approximately 53.8 degrees according to https://smeenk.com/kinect-field-of-view-comparison/
    AvatarCamera.fieldOfView = 53.8f;
  }
}

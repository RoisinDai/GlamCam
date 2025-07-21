using UnityEngine;
using System.Net.Sockets;
using System.Collections;
using Windows.Kinect;
using System.Collections.Generic;
using UnityEditor;

// This script streams the camera view and mapped Kinect joint coordinates (in pixel space) over TCP.
// Packet format: [4 bytes: image length][image bytes][4 bytes: joint data length][joint data as JSON string bytes]
public class UnityCameraTcpStreamer : MonoBehaviour
{
    public Camera avartarCamera;
    public BodySourceManager bodySourceManager;

    public string unity2overlay_host = "127.0.0.1";
    public int unity2overlay_port = 5005;

    // Frames per second
    public int frameRate = 30;
    public int jpgQuality = 80;

    private TcpClient client;
    private NetworkStream stream;
    private bool streaming = false;

    void Start()
    {
        Connect();
        if (client != null)
        {
            streaming = true;
            Application.runInBackground = true;
            StartCoroutine(StreamFrames());
        }
    }

    void OnApplicationQuit()
    {
        streaming = false;
        if (stream != null) stream.Close();
        if (client != null) client.Close();
    }

    void Connect()
    {
        try
        {
            client = new TcpClient(unity2overlay_host, unity2overlay_port);
            stream = client.GetStream();
        }
        catch
        {
            client = null;
            stream = null;
        }
    }

    IEnumerator StreamFrames()
    {
        WaitForSeconds wait = new WaitForSeconds(1f / frameRate);
        while (streaming)
        {
            SendCameraFrameAndJoints();
            yield return wait;
        }
    }

    void SendCameraFrameAndJoints()
    {
        // 1. Capture camera image
        byte[] img = CaptureCameraFrame(avartarCamera);

        // 2. Get joint data, map to pixel coordinates
        string jointJson = GetJointPixelCoordinatesAsJson(avartarCamera, bodySourceManager);

        // 3. Send packet: [4 bytes: img length][img bytes][4 bytes: json length][json bytes]
        try
        {
            // Send image
            byte[] lenImg = System.BitConverter.GetBytes(img.Length);
            stream.Write(lenImg, 0, 4);
            stream.Write(img, 0, img.Length);

            // Send joint data
            byte[] jointBytes = System.Text.Encoding.UTF8.GetBytes(jointJson);
            byte[] lenJoint = System.BitConverter.GetBytes(jointBytes.Length);
            stream.Write(lenJoint, 0, 4);
            stream.Write(jointBytes, 0, jointBytes.Length);

            stream.Flush();
        }
        catch
        {
            Debug.LogWarning("Lost connection to Python. Stopping stream.");
            streaming = false;
        }
    }

    byte[] CaptureCameraFrame(Camera cam)
    {
        RenderTexture rt = new RenderTexture(Screen.width, Screen.height, 24);
        cam.targetTexture = rt;
        cam.Render();

        RenderTexture.active = rt;
        Texture2D tex = new Texture2D(Screen.width, Screen.height, TextureFormat.RGB24, false);
        tex.ReadPixels(new Rect(0, 0, Screen.width, Screen.height), 0, 0);
        tex.Apply();

        cam.targetTexture = null;
        RenderTexture.active = null;
        Destroy(rt);

        byte[] imgBytes = tex.EncodeToJPG(jpgQuality);
        Destroy(tex);

        return imgBytes;
    }

    string GetJointPixelCoordinatesAsJson(Camera cam, BodySourceManager bsm)
    {
        Body[] bodies = bsm.GetData();
        if (bodies == null)
        {
            return "{}";
        }

        int screenHeight = Screen.height;

        // Use first tracked body only
        foreach (var body in bodies)
        {
            if (body != null && body.IsTracked)
            {
                var jointDict = new Dictionary<string, object>();

                foreach (Windows.Kinect.JointType jt in System.Enum.GetValues(typeof(Windows.Kinect.JointType)))
                {
                    var joint = body.Joints[jt];
                    // Kinect coordinates are in meters; convert to Unity world, then to pixel
                    Vector3 unityWorld = BodySourceView.GetVector3FromJoint(joint);
                    Vector3 screenPt = cam.WorldToScreenPoint(unityWorld);

                    // y = 0 is at the bottom of the screen in Unity
                    jointDict[jt.ToString()] = new Dictionary<string, float>
                    {
                        { "x", screenPt.x },
                        { "y", screenHeight - screenPt.y }
                    };
                }
                return Json.Serialize(jointDict);
            }
        }

        // No tracked body
        return "{}";
    }
}

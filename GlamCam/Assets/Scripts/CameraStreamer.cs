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

    private TcpClient client;
    private NetworkStream stream;
    private bool streaming = false;

    // Reusable rendering objects for performance
    private RenderTexture rt;
    private Texture2D tex;
    private Rect readRect;
    private int w, h;

    void Awake()
    {
        // Configure camera for transparent background as early as possible
        if (avartarCamera != null)
        {
            avartarCamera.clearFlags = CameraClearFlags.SolidColor;
            avartarCamera.backgroundColor = new Color(0, 0, 0, 0); // Fully transparent
            Debug.Log("Camera configured for transparency: clearFlags=" + avartarCamera.clearFlags + ", backgroundColor=" + avartarCamera.backgroundColor);
        }
    }

    void Start()
    {
        w = Screen.width;
        h = Screen.height;

        rt = new RenderTexture(w, h, 24, RenderTextureFormat.ARGB32);
        rt.antiAliasing = 1; // keep simple; can increase later

        tex = new Texture2D(w, h, TextureFormat.RGBA32, false);
        readRect = new Rect(0, 0, w, h);

        // Double-check camera settings in Start() in case they were overridden
        if (avartarCamera != null)
        {
            avartarCamera.clearFlags = CameraClearFlags.SolidColor;
            avartarCamera.backgroundColor = new Color(0, 0, 0, 0); // Transparent background
            Debug.Log("Camera settings confirmed: clearFlags=" + avartarCamera.clearFlags + ", backgroundColor=" + avartarCamera.backgroundColor);
        }

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

        if (tex != null) Destroy(tex);
        if (rt != null) Destroy(rt);
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
        // Ensure camera is configured for transparency before rendering
        if (cam.clearFlags != CameraClearFlags.SolidColor || cam.backgroundColor.a != 0)
        {
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = new Color(0, 0, 0, 0);
            Debug.LogWarning("Camera settings were incorrect, corrected to transparent background");
        }

        // Render into the persistent RT
        var prevTarget = cam.targetTexture;
        var prevActive = RenderTexture.active;

        cam.targetTexture = rt;
        cam.Render();

        RenderTexture.active = rt;

        // Read into the persistent Texture2D
        tex.ReadPixels(readRect, 0, 0, false);
        tex.Apply(false, false);

        // Restore state
        cam.targetTexture = prevTarget;
        RenderTexture.active = prevActive;

        // Encode PNG (keeps alpha)
        return tex.EncodeToPNG();
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

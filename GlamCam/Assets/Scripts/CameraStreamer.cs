using UnityEngine;
using System.Net.Sockets;
using System.Collections;

// This script streams frames from two Unity cameras over TCP to a Python server.
// It captures frames from both cameras, encodes them as JPEG, and sends them in a specific format:
// [4 bytes: lengthA][bytesA][4 bytes: lengthB][bytesB]
// The script runs in the background and streams at a specified frame rate.
public class UnityCameraTcpMultiStreamer : MonoBehaviour
{
    public Camera cameraA; // Assign in inspector
    public Camera cameraB; // Assign in inspector
    public string host = "127.0.0.1";
    public int port = 5005;
    public int frameRate = 15; // Frames per second
    public int jpgQuality = 80;

    private TcpClient client = null;
    private NetworkStream stream = null;
    private bool streaming = false;

    void Start()
    {
        Connect();
        streaming = true;
        Application.runInBackground = true;
        StartCoroutine(StreamFrames());
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
          client = new TcpClient(host, port);
        }
        catch
        {
          client = null;
        }
        if (client != null)
        {
          stream = client.GetStream();
        }
    }

    IEnumerator StreamFrames()
    {
        WaitForSeconds wait = new WaitForSeconds(1f / frameRate);
        while (streaming)
        {
            SendBothCameraFrames();
            yield return wait;
        }
    }

    void SendBothCameraFrames()
    {
        // Capture both cameras
        byte[] imgA = CaptureCameraFrame(cameraA);
        byte[] imgB = CaptureCameraFrame(cameraB);

        // Prepare the packet:
        // [4 bytes: lengthA][bytesA][4 bytes: lengthB][bytesB]
        try
        {
            // Send length and bytes for cameraA
            byte[] lenA = System.BitConverter.GetBytes(imgA.Length);
            stream.Write(lenA, 0, 4);
            stream.Write(imgA, 0, imgA.Length);

            // Send length and bytes for cameraB
            byte[] lenB = System.BitConverter.GetBytes(imgB.Length);
            stream.Write(lenB, 0, 4);
            stream.Write(imgB, 0, imgB.Length);

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
}

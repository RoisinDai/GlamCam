using UnityEngine;
using System.Collections;
using Windows.Kinect;

public class MultiSourceManager : MonoBehaviour {
    public int ColorWidth { get; private set; }
    public int ColorHeight { get; private set; }
    
    // For silhouette capture - assign in Inspector
    public GameObject BodySourceManagerObject;
    
    private KinectSensor _Sensor;
    private MultiSourceFrameReader _Reader;
    private CoordinateMapper _Mapper;
    private Texture2D _ColorTexture;
    private ushort[] _DepthData;
    private byte[] _ColorData;
    private byte[] _BodyIndexData;
    
    private const int DEPTH_WIDTH = 512;
    private const int DEPTH_HEIGHT = 424;

    public Texture2D GetColorTexture()
    {
        return _ColorTexture;
    }
    
    public ushort[] GetDepthData()
    {
        return _DepthData;
    }

    public byte[] GetBodyIndexData()
    {
        return _BodyIndexData;
    }

    void Start () 
    {
        _Sensor = KinectSensor.GetDefault();
        
        if (_Sensor != null) 
        {
            _Reader = _Sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.BodyIndex);
            
            var colorFrameDesc = _Sensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Rgba);
            ColorWidth = colorFrameDesc.Width;
            ColorHeight = colorFrameDesc.Height;
            
            _ColorTexture = new Texture2D(colorFrameDesc.Width, colorFrameDesc.Height, TextureFormat.RGBA32, false);
            _ColorData = new byte[colorFrameDesc.BytesPerPixel * colorFrameDesc.LengthInPixels];
            
            var depthFrameDesc = _Sensor.DepthFrameSource.FrameDescription;
            _DepthData = new ushort[depthFrameDesc.LengthInPixels];
            _BodyIndexData = new byte[depthFrameDesc.LengthInPixels]; // 512 x 424
            
            _Mapper = _Sensor.CoordinateMapper;
            
            if (!_Sensor.IsOpen)
            {
                _Sensor.Open();
            }
        }
        
        Debug.Log("MultiSourceManager: Press B to capture BodyIndex silhouette + skeleton");
    }
    
    void Update () 
    {
        if (_Reader != null) 
        {
            var frame = _Reader.AcquireLatestFrame();
            if (frame != null)
            {
                var colorFrame = frame.ColorFrameReference.AcquireFrame();
                if (colorFrame != null)
                {
                    var depthFrame = frame.DepthFrameReference.AcquireFrame();
                    if (depthFrame != null)
                    {
                        var bodyIndexFrame = frame.BodyIndexFrameReference.AcquireFrame();
                        if (bodyIndexFrame != null)
                        {
                            colorFrame.CopyConvertedFrameDataToArray(_ColorData, ColorImageFormat.Rgba);
                            _ColorTexture.LoadRawTextureData(_ColorData);
                            _ColorTexture.Apply();
                            
                            depthFrame.CopyFrameDataToArray(_DepthData);
                            bodyIndexFrame.CopyFrameDataToArray(_BodyIndexData);
                            
                            bodyIndexFrame.Dispose();
                            bodyIndexFrame = null;
                        }
                        
                        depthFrame.Dispose();
                        depthFrame = null;
                    }
                
                    colorFrame.Dispose();
                    colorFrame = null;
                }
                
                frame = null;
            }
        }
        
        // Press B to capture silhouette + skeleton
        if (Input.GetKeyDown(KeyCode.B))
        {
            CaptureBodyIndexWithSkeleton();
        }
    }
    
    /// <summary>
    /// Captures silhouette + skeleton and saves to Desktop.
    /// </summary>
    private void CaptureBodyIndexWithSkeleton()
    {
        if (_BodyIndexData == null)
        {
            Debug.LogError("No BodyIndex data available");
            return;
        }
        
        Texture2D texture = new Texture2D(DEPTH_WIDTH, DEPTH_HEIGHT, TextureFormat.RGBA32, false);
        Color32[] colors = new Color32[DEPTH_WIDTH * DEPTH_HEIGHT];
        
        int bodyPixelCount = 0;
        
        // Draw silhouette (green = body, black = background)
        for (int i = 0; i < _BodyIndexData.Length; i++)
        {
            if (_BodyIndexData[i] != 255)
            {
                colors[i] = new Color32(0, 150, 0, 255);
                bodyPixelCount++;
            }
            else
            {
                colors[i] = new Color32(0, 0, 0, 255);
            }
        }
        
        // Draw skeleton joints (red dots)
        if (BodySourceManagerObject != null && _Mapper != null)
        {
            var bodyManager = BodySourceManagerObject.GetComponent<BodySourceManager>();
            if (bodyManager != null)
            {
                Body[] bodies = bodyManager.GetData();
                if (bodies != null)
                {
                    foreach (Body body in bodies)
                    {
                        if (body != null && body.IsTracked)
                        {
                            foreach (JointType jt in System.Enum.GetValues(typeof(JointType)))
                            {
                                Joint joint = body.Joints[jt];
                                if (joint.TrackingState == TrackingState.NotTracked) continue;
                                
                                DepthSpacePoint pt = _Mapper.MapCameraPointToDepthSpace(joint.Position);
                                DrawDot(colors, (int)pt.X, (int)pt.Y, 4, new Color32(255, 0, 0, 255));
                            }
                        }
                    }
                }
            }
        }
        
        texture.SetPixels32(colors);
        texture.Apply();
        
        // Save to Desktop
        byte[] pngData = texture.EncodeToPNG();
        string path = System.Environment.GetFolderPath(System.Environment.SpecialFolder.Desktop) + "/BodyIndex_Capture.png";
        System.IO.File.WriteAllBytes(path, pngData);
        
        Debug.Log($"Saved to {path} - {bodyPixelCount} body pixels ({(bodyPixelCount * 100f / _BodyIndexData.Length):F1}%)");
        
        Destroy(texture);
    }
    
    private void DrawDot(Color32[] colors, int cx, int cy, int r, Color32 color)
    {
        for (int dy = -r; dy <= r; dy++)
        {
            for (int dx = -r; dx <= r; dx++)
            {
                int x = cx + dx;
                int y = cy + dy;
                if (x >= 0 && x < DEPTH_WIDTH && y >= 0 && y < DEPTH_HEIGHT && dx*dx + dy*dy <= r*r)
                {
                    colors[y * DEPTH_WIDTH + x] = color;
                }
            }
        }
    }
    
    void OnApplicationQuit()
    {
        if (_Reader != null)
        {
            _Reader.Dispose();
            _Reader = null;
        }
        
        if (_Sensor != null)
        {
            if (_Sensor.IsOpen)
            {
                _Sensor.Close();
            }
            
            _Sensor = null;
        }
    }
}

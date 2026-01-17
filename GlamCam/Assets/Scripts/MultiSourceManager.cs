using UnityEngine;
using System.Collections;
using Windows.Kinect;

public class MultiSourceManager : MonoBehaviour {
    public int ColorWidth { get; private set; }
    public int ColorHeight { get; private set; }
    
    private KinectSensor _Sensor;
    private MultiSourceFrameReader _Reader;
    private CoordinateMapper _Mapper;
    private Texture2D _ColorTexture;
    private ushort[] _DepthData;
    private byte[] _ColorData;
    private byte[] _BodyIndexData;
    
    // For color-to-depth mapping (Body Mask)
    private DepthSpacePoint[] _ColorMappedToDepthPoints;
    
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
    
    public CoordinateMapper GetCoordinateMapper()
    {
        return _Mapper;
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
            
            // For Body Mask - mapping color pixels to depth space
            _ColorMappedToDepthPoints = new DepthSpacePoint[colorFrameDesc.Width * colorFrameDesc.Height];
            
            if (!_Sensor.IsOpen)
            {
                _Sensor.Open();
            }
        }
        
        Debug.Log("MultiSourceManager: Started successfully. Press M or click button to capture.");
    }
    else
    {
        Debug.LogError("MultiSourceManager: Kinect sensor not found!");
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
        
        // Press M to capture Body Mask (color with background removed)
        if (Input.GetKeyDown(KeyCode.M))
        {
            Debug.Log("M key pressed - capturing body mask...");
            CaptureBodyMask();
        }
    }
    
    // GUI button as fallback for key press
    void OnGUI()
    {
        if (GUI.Button(new Rect(10, 10, 200, 50), "Capture Body Mask (M)"))
        {
            Debug.Log("Button clicked - capturing body mask...");
            CaptureBodyMask();
        }
    }
    
    /// <summary>
    /// Captures Body Mask - color image with background removed (Lab 05 approach).
    /// Maps color pixels to depth space and checks BodyIndex to keep only body pixels.
    /// </summary>
    private void CaptureBodyMask()
    {
        Debug.Log("CaptureBodyMask: Starting capture...");
        
        // Checkpoint 1: Check frame data
        if (_ColorData == null)
        {
            Debug.LogError("CaptureBodyMask: _ColorData is null - no color frame received");
            return;
        }
        if (_DepthData == null)
        {
            Debug.LogError("CaptureBodyMask: _DepthData is null - no depth frame received");
            return;
        }
        if (_BodyIndexData == null)
        {
            Debug.LogError("CaptureBodyMask: _BodyIndexData is null - no body index frame received");
            return;
        }
        Debug.Log("CaptureBodyMask: All frame data available");
        
        // Checkpoint 2: Check mapper
        if (_Mapper == null)
        {
            Debug.LogError("CaptureBodyMask: CoordinateMapper not available");
            return;
        }
        Debug.Log("CaptureBodyMask: CoordinateMapper ready");
        
        // Map color frame to depth space
        _Mapper.MapColorFrameToDepthSpace(_DepthData, _ColorMappedToDepthPoints);
        
        // Create output texture at color resolution
        Texture2D texture = new Texture2D(ColorWidth, ColorHeight, TextureFormat.RGBA32, false);
        Color32[] colors = new Color32[ColorWidth * ColorHeight];
        
        int bodyPixelCount = 0;
        
        // For each color pixel, check if it corresponds to a body
        for (int colorIndex = 0; colorIndex < _ColorMappedToDepthPoints.Length; colorIndex++)
        {
            float colorMappedToDepthX = _ColorMappedToDepthPoints[colorIndex].X;
            float colorMappedToDepthY = _ColorMappedToDepthPoints[colorIndex].Y;
            
            bool isBody = false;
            
            // Check if this color pixel maps to a valid depth point
            if (!float.IsNegativeInfinity(colorMappedToDepthX) && 
                !float.IsNegativeInfinity(colorMappedToDepthY))
            {
                int depthX = (int)(colorMappedToDepthX + 0.5f);
                int depthY = (int)(colorMappedToDepthY + 0.5f);
                
                // Check bounds
                if (depthX >= 0 && depthX < DEPTH_WIDTH && 
                    depthY >= 0 && depthY < DEPTH_HEIGHT)
                {
                    int depthIndex = (depthY * DEPTH_WIDTH) + depthX;
                    
                    // Check if this pixel is a body (not 0xff/255)
                    if (_BodyIndexData[depthIndex] != 255)
                    {
                        isBody = true;
                        bodyPixelCount++;
                    }
                }
            }
            
            if (isBody)
            {
                // Keep the original color (RGBA format: R, G, B, A)
                int byteIndex = colorIndex * 4;
                colors[colorIndex] = new Color32(
                    _ColorData[byteIndex],     // R
                    _ColorData[byteIndex + 1], // G
                    _ColorData[byteIndex + 2], // B
                    255                         // A (fully opaque)
                );
            }
            else
            {
                // Background - make black/transparent
                colors[colorIndex] = new Color32(0, 0, 0, 255);
            }
        }
        
        texture.SetPixels32(colors);
        texture.Apply();
        
        Debug.Log($"CaptureBodyMask: Processed image - {bodyPixelCount} body pixels found");
        
        // Checkpoint 3: Save to Desktop
        try
        {
            byte[] pngData = texture.EncodeToPNG();
            string path = System.Environment.GetFolderPath(System.Environment.SpecialFolder.Desktop) + "/BodyMask_Capture.png";
            Debug.Log($"CaptureBodyMask: Saving to {path}");
            System.IO.File.WriteAllBytes(path, pngData);
            Debug.Log($"CaptureBodyMask: SUCCESS! File saved to {path}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"CaptureBodyMask: Failed to save file - {e.Message}");
        }
        
        Destroy(texture);
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

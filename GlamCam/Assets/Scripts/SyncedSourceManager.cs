using UnityEngine;
using System.Collections;
using Windows.Kinect;

// Responsible for getting synchronized color, depth, and body data from the Kinect sensor
// for each frame using MultiSourceFrameReader.
// Data can be retrieved from the GetColorTexture(), GetDepthData(), and GetBodyData() methods.
public class SyncedSourceManager : MonoBehaviour
{
    // Frame dimensions
    public int ColorWidth { get; private set; }
    public int ColorHeight { get; private set; }
    public int DepthWidth { get; private set; }
    public int DepthHeight { get; private set; }
    
    private KinectSensor _Sensor;
    private MultiSourceFrameReader _Reader;
    
    // Frame data storage
    private Texture2D _ColorTexture;
    private ushort[] _DepthData;
    private byte[] _ColorData;
    private Body[] _BodyData = null;

    // Public accessors following the same pattern as other managers
    public Texture2D GetColorTexture()
    {
        return _ColorTexture;
    }
    
    public ushort[] GetDepthData()
    {
        return _DepthData;
    }
    
    public Body[] GetBodyData()
    {
        return _BodyData;
    }

    public KinectSensor GetSensor()
    {
        return _Sensor;
    }

    void Start()
    {
        _Sensor = KinectSensor.GetDefault();
        
        if (_Sensor != null)
        {
            // Create multi-source reader for all three frame types (synchronized)
            _Reader = _Sensor.OpenMultiSourceFrameReader(
                FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Body
            );
            
            // Initialize color frame data (from MultiSourceManager pattern)
            var colorFrameDesc = _Sensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Rgba);
            ColorWidth = colorFrameDesc.Width;
            ColorHeight = colorFrameDesc.Height;
            _ColorTexture = new Texture2D(colorFrameDesc.Width, colorFrameDesc.Height, TextureFormat.RGBA32, false);
            _ColorData = new byte[colorFrameDesc.BytesPerPixel * colorFrameDesc.LengthInPixels];
            
            // Initialize depth frame data (from MultiSourceManager pattern)
            var depthFrameDesc = _Sensor.DepthFrameSource.FrameDescription;
            DepthWidth = depthFrameDesc.Width;
            DepthHeight = depthFrameDesc.Height;
            _DepthData = new ushort[depthFrameDesc.LengthInPixels];
            
            if (!_Sensor.IsOpen)
            {
                _Sensor.Open();
            }
        }
    }

    void Update()
    {
        if (_Reader != null)
        {
            var multiFrame = _Reader.AcquireLatestFrame();
            if (multiFrame != null)
            {
                // Get all three frame references
                var colorFrame = multiFrame.ColorFrameReference.AcquireFrame();
                var depthFrame = multiFrame.DepthFrameReference.AcquireFrame();
                var bodyFrame = multiFrame.BodyFrameReference.AcquireFrame();

                // Process color frame (from MultiSourceManager pattern)
                if (colorFrame != null)
                {
                    colorFrame.CopyConvertedFrameDataToArray(_ColorData, ColorImageFormat.Rgba);
                    _ColorTexture.LoadRawTextureData(_ColorData);
                    _ColorTexture.Apply();
                    
                    colorFrame.Dispose();
                    colorFrame = null;
                }
                
                // Process depth frame (from MultiSourceManager pattern)
                if (depthFrame != null)
                {
                    depthFrame.CopyFrameDataToArray(_DepthData);
                    
                    depthFrame.Dispose();
                    depthFrame = null;
                }
                
                // Process body frame (from BodySourceManager pattern)
                if (bodyFrame != null)
                {
                    if (_BodyData == null)
                    {
                        _BodyData = new Body[_Sensor.BodyFrameSource.BodyCount];
                    }
                    
                    bodyFrame.GetAndRefreshBodyData(_BodyData);
                    
                    bodyFrame.Dispose();
                    bodyFrame = null;
                }
                
                multiFrame = null;
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
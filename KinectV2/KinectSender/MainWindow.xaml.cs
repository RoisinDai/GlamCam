using Microsoft.Kinect;
using System;
using System.ComponentModel;
using System.Globalization;
using System.IO;
using System.Net.Sockets;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using Newtonsoft.Json;

namespace Microsoft.Samples.Kinect.ColorBasics
{
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        private KinectSensor kinectSensor = null;
        private ColorFrameReader colorFrameReader = null;
        private BodyFrameReader bodyFrameReader = null;
        private WriteableBitmap colorBitmap = null;
        private DrawingGroup drawingGroup;
        private DrawingImage imageSource;
        private string statusText = null;
        private TcpClient client = null;
        private NetworkStream stream = null;
        private int port = 5006;
        private string host = "127.0.0.1";
        private int handPort = 5009;
        private string handHost = "127.0.0.1";
        private TcpClient handClient = null;
        private NetworkStream handStream = null;

        // Body tracking
        private Body[] bodies = null;
        private List<Tuple<JointType, JointType>> bones;
        private List<Pen> bodyColors;
        private int displayWidth;
        private int displayHeight;
        private CoordinateMapper coordinateMapper = null;
        private const float InferredZPositionClamp = 0.1f;
        private const double HandSize = 30;
        private const double JointThickness = 6;
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));
        private readonly Brush inferredJointBrush = Brushes.Yellow;
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        // Store latest bodies for overlay
        private Body[] latestBodiesForOverlay = null;

        public MainWindow()
        {
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the color frames
            this.colorFrameReader = this.kinectSensor.ColorFrameSource.OpenReader();
            this.colorFrameReader.FrameArrived += this.ColorFrameArrived;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();
            this.bodyFrameReader.FrameArrived += this.BodyFrameArrived;

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();
            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));
            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));
            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));
            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));
            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();
            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // create the colorFrameDescription from the ColorFrameSource using Bgra format
            FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);

            // create the bitmap to display
            this.colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

            // Create the drawing group and image source for overlay
            this.drawingGroup = new DrawingGroup();
            this.imageSource = new DrawingImage(this.drawingGroup);

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Try to setup TCP connection to send data
            try
            {
                this.client = new TcpClient(host, port);
            }
            catch (Exception)
            {
                this.client = null;
            }
            if (this.client != null)
            {
                this.stream = client.GetStream();
            }

            // Try to setup TCP connection to send hand data
            try
            {
                this.handClient = new TcpClient(handHost, handPort);
            }
            catch (Exception)
            {
                this.handClient = null;
            }
            if (this.handClient != null)
            {
                this.handStream = handClient.GetStream();
            }

            this.DataContext = this;
            this.InitializeComponent();
        }

        public event PropertyChangedEventHandler PropertyChanged;

        public ImageSource ImageSource
        {
            get { return this.imageSource; }
        }

        public string StatusText
        {
            get { return this.statusText; }
            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;
                    this.PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("StatusText"));
                }
            }
        }

        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.colorFrameReader != null)
            {
                this.colorFrameReader.Dispose();
                this.colorFrameReader = null;
            }
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }
            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
            if (this.stream != null)
            {
                this.stream.Close();
                this.stream = null;
            }
            if (this.client != null)
            {
                this.client.Close();
                this.client = null;
            }
        }

        // Store latest bodies for overlay drawing
        private void BodyFrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }
                    bodyFrame.GetAndRefreshBodyData(this.bodies);

                    // Deep copy bodies array for overlay to avoid threading issues
                    this.latestBodiesForOverlay = new Body[bodyFrame.BodyCount];
                    for (int i = 0; i < bodyFrame.BodyCount; i++)
                    {
                        this.latestBodiesForOverlay[i] = this.bodies[i];
                    }
                }
            }
        }

        /// <summary>
        /// Helper to check if a point is within the color frame bounds
        /// </summary>
        private bool IsPointInColorBounds(Point pt)
        {
            return pt.X >= 0 && pt.X < this.colorBitmap.PixelWidth &&
                   pt.Y >= 0 && pt.Y < this.colorBitmap.PixelHeight;
        }

        private void ColorFrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            using (ColorFrame colorFrame = e.FrameReference.AcquireFrame())
            {
                if (colorFrame == null)
                    return;

                FrameDescription colorFrameDescription = colorFrame.FrameDescription;
                using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                {
                    this.colorBitmap.Lock();
                    if ((colorFrameDescription.Width == this.colorBitmap.PixelWidth) && (colorFrameDescription.Height == this.colorBitmap.PixelHeight))
                    {
                        colorFrame.CopyConvertedFrameDataToIntPtr(
                            this.colorBitmap.BackBuffer,
                            (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4),
                            ColorImageFormat.Bgra);
                        this.colorBitmap.AddDirtyRect(new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight));
                    }
                    this.colorBitmap.Unlock();
                }

                // Draw overlay with color frame as background
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    dc.DrawImage(this.colorBitmap, new Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight));
                    if (this.latestBodiesForOverlay != null)
                    {
                        int penIndex = 0;
                        foreach (Body body in this.latestBodiesForOverlay)
                        {
                            Pen drawPen = this.bodyColors[penIndex++ % this.bodyColors.Count];
                            if (body != null && body.IsTracked)
                            {
                                IReadOnlyDictionary<JointType, Joint> joints = body.Joints;
                                Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();
                                foreach (JointType jointType in joints.Keys)
                                {
                                    CameraSpacePoint position = joints[jointType].Position;
                                    if (position.Z < 0)
                                        position.Z = InferredZPositionClamp;
                                    ColorSpacePoint colorSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(position);
                                    jointPoints[jointType] = new Point(colorSpacePoint.X, colorSpacePoint.Y);
                                }
                                DrawBody(joints, jointPoints, dc, drawPen);
                                DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                                DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                            }
                        }
                    }
                    dc.PushClip(new RectangleGeometry(new Rect(0.0, 0.0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight)));
                }

                // Send color frame and pixel joint coordinates to Python
                SendColorFrameAndPixelJointsToPython(colorFrame, colorFrameDescription);

                // Send hand joints coordinates to website
                SendHandJointsToHandClient();
            }
        }

        /// <summary>
        /// Sends the color frame as JPEG and the joint pixel coordinates (as JSON) to Python through the TCP stream.
        /// Message format:
        /// [4] totalLen | [4] jpgLen | [jpgLen] jpgBytes | [4] jointsLen | [jointsLen] jointsBytes
        /// </summary>
        private void SendColorFrameAndPixelJointsToPython(ColorFrame colorFrame, FrameDescription colorFrameDescription)
        {
            if (this.stream != null && this.stream.CanWrite)
            {
                try
                {
                    // 1. Encode color frame to JPEG
                    var tempBitmap = new WriteableBitmap(
                        colorFrameDescription.Width,
                        colorFrameDescription.Height,
                        96.0, 96.0, System.Windows.Media.PixelFormats.Bgra32, null);

                    tempBitmap.Lock();
                    colorFrame.CopyConvertedFrameDataToIntPtr(
                        tempBitmap.BackBuffer,
                        (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4),
                        ColorImageFormat.Bgra);
                    tempBitmap.AddDirtyRect(new Int32Rect(0, 0, tempBitmap.PixelWidth, tempBitmap.PixelHeight));
                    tempBitmap.Unlock();

                    byte[] jpgBytes;
                    var encoder = new JpegBitmapEncoder();
                    encoder.Frames.Add(BitmapFrame.Create(tempBitmap));
                    using (var ms = new MemoryStream())
                    {
                        encoder.Save(ms);
                        jpgBytes = ms.ToArray();
                    }

                    // 2. Gather pixel joint data (from latestBodiesForOverlay)
                    List<object> bodiesList = new List<object>();
                    if (this.latestBodiesForOverlay != null)
                    {
                        foreach (Body body in this.latestBodiesForOverlay)
                        {
                            if (body != null && body.IsTracked)
                            {
                                var jointsDict = new Dictionary<string, object>();
                                foreach (var kv in body.Joints)
                                {
                                    // Pixel coordinates
                                    CameraSpacePoint position = kv.Value.Position;
                                    if (position.Z < 0)
                                        position.Z = InferredZPositionClamp;
                                    ColorSpacePoint colorSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(position);

                                    jointsDict[kv.Key.ToString()] = new
                                    {
                                        X = colorSpacePoint.X,
                                        Y = colorSpacePoint.Y,
                                        State = kv.Value.TrackingState.ToString()
                                    };
                                }
                                bodiesList.Add(jointsDict);
                            }
                        }
                    }
                    string jointsJson = JsonConvert.SerializeObject(bodiesList);
                    byte[] jointsBytes = Encoding.UTF8.GetBytes(jointsJson);

                    // 3. Compose a message with lengths
                    int totalLen = 4 + jpgBytes.Length + 4 + jointsBytes.Length;
                    byte[] totalLenBytes = BitConverter.GetBytes(totalLen);
                    byte[] jpgLenBytes = BitConverter.GetBytes(jpgBytes.Length);
                    byte[] jointsLenBytes = BitConverter.GetBytes(jointsBytes.Length);

                    // 4. Send message
                    lock (this.stream)
                    {
                        this.stream.Write(totalLenBytes, 0, 4);
                        this.stream.Write(jpgLenBytes, 0, 4);
                        this.stream.Write(jpgBytes, 0, jpgBytes.Length);
                        this.stream.Write(jointsLenBytes, 0, 4);
                        this.stream.Write(jointsBytes, 0, jointsBytes.Length);
                    }
                    System.Diagnostics.Debug.WriteLine($"Frame and pixel joints sent: {jpgBytes.Length} bytes image, {jointsBytes.Length} bytes joints");
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine("Error sending frame and pixel joints: " + ex.ToString());
                }
            }
        }

        public void SendHandJointsToHandClient()
        {
            // Assume handClient and handStream are already connected and valid
            if (this.handStream != null && this.handStream.CanWrite)
            {
                try
                {
                    List<object> handsList = new List<object>();
                    if (this.latestBodiesForOverlay != null)
                    {
                        foreach (Body body in this.latestBodiesForOverlay)
                        {
                            if (body != null && body.IsTracked)
                            {
                                var handJointsDict = new Dictionary<string, object>();
                                foreach (var kv in body.Joints)
                                {
                                    var jointType = kv.Key;
                                    if (jointType == JointType.HandLeft ||
                                        jointType == JointType.HandRight ||
                                        jointType == JointType.HandTipLeft ||
                                        jointType == JointType.HandTipRight)
                                    {
                                        // Get pixel coordinates
                                        CameraSpacePoint position = kv.Value.Position;
                                        if (position.Z < 0)
                                            position.Z = InferredZPositionClamp;
                                        ColorSpacePoint colorSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(position);

                                        handJointsDict[jointType.ToString()] = new
                                        {
                                            X = colorSpacePoint.X,
                                            Y = colorSpacePoint.Y,
                                            State = kv.Value.TrackingState.ToString()
                                        };
                                    }
                                }
                                if (handJointsDict.Count > 0)
                                    handsList.Add(handJointsDict);
                            }
                        }
                    }
                    string handsJson = JsonConvert.SerializeObject(handsList);
                    byte[] handsBytes = Encoding.UTF8.GetBytes(handsJson);

                    // Send [4] length + [data]
                    int totalLen = 4 + handsBytes.Length;
                    byte[] totalLenBytes = BitConverter.GetBytes(totalLen);
                    byte[] dataLenBytes = BitConverter.GetBytes(handsBytes.Length);

                    lock (this.handStream)
                    {
                        this.handStream.Write(totalLenBytes, 0, 4);
                        this.handStream.Write(dataLenBytes, 0, 4);
                        this.handStream.Write(handsBytes, 0, handsBytes.Length);
                    }
                    System.Diagnostics.Debug.WriteLine($"Hand joints sent: {handsBytes.Length} bytes");
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine("Error sending hand joints: " + ex.ToString());
                }
            }
        }

        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            foreach (var bone in this.bones)
            {
                DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;
                TrackingState trackingState = joints[jointType].TrackingState;
                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }
                Point pt = jointPoints[jointType];
                // Only draw joint if within color frame bounds
                if (drawBrush != null && IsPointInColorBounds(pt))
                {
                    drawingContext.DrawEllipse(drawBrush, null, pt, JointThickness, JointThickness);
                }
            }
        }

        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }
            // Only draw bone if both points are within color frame bounds
            if (jointPoints.ContainsKey(jointType0) && jointPoints.ContainsKey(jointType1))
            {
                Point pt0 = jointPoints[jointType0];
                Point pt1 = jointPoints[jointType1];
                if (IsPointInColorBounds(pt0) && IsPointInColorBounds(pt1))
                {
                    Pen drawPen = this.inferredBonePen;
                    if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
                    {
                        drawPen = drawingPen;
                    }
                    drawingContext.DrawLine(drawPen, pt0, pt1);
                }
            }
        }

        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            // Only draw hand if within color frame bounds
            if (IsPointInColorBounds(handPosition))
            {
                switch (handState)
                {
                    case HandState.Closed:
                        drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                        break;
                    case HandState.Open:
                        drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                        break;
                    case HandState.Lasso:
                        drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                        break;
                }
            }
        }

        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }
    }
}
//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.SilhouetteBasics
{
    using System;
    using System.ComponentModel;
    using System.Globalization;
    using System.IO;
    using System.Linq;
    using System.Text;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;

    /// <summary>
    /// Container class for body measurements from skeleton joints
    /// </summary>
    class HumanoidMeasurements
    {
        public double height;
        public double upperArmLength; // Shoulder to elbow
        public double lowerArmLength; // Elbow to wrist
        public double upperLegLength; // Hip to knee
        public double lowerLegLength; // Knee to foot (ankle)
        public double napeToWaist;    // SpineShoulder to SpineMid
        public double shoulderDist;   // Avg(SpineShoulder to ShoulderLeft/Right)
        public double waistToHip;     // SpineBase to SpineMid
        public double neckHeight;     // Head to Neck
    }

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Depth frame dimensions
        /// </summary>
        private const int DEPTH_WIDTH = 512;
        private const int DEPTH_HEIGHT = 424;
        private const float DEPTH_HORIZONTAL_FOV = 70.6f; // Kinect v2 depth camera horizontal FOV in degrees
        private const int MORPHOLOGICAL_RADIUS = 2; // Radius for morphological cleanup operations

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Reader for body index frames (silhouette)
        /// </summary>
        private BodyIndexFrameReader bodyIndexFrameReader = null;

        /// <summary>
        /// Reader for body frames (skeleton/joints)
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Reader for depth frames
        /// </summary>
        private DepthFrameReader depthFrameReader = null;

        /// <summary>
        /// Coordinate mapper for spatial transformations
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Description of the body index frame
        /// </summary>
        private FrameDescription bodyIndexFrameDescription = null;

        /// <summary>
        /// Description of the depth frame
        /// </summary>
        private FrameDescription depthFrameDescription = null;

        /// <summary>
        /// Bitmap to display silhouette
        /// </summary>
        private WriteableBitmap silhouetteBitmap = null;

        /// <summary>
        /// Intermediate storage for body index frame data
        /// </summary>
        private byte[] bodyIndexData = null;

        /// <summary>
        /// Intermediate storage for cleaned body index data (after morphological operations)
        /// </summary>
        private byte[] cleanedBodyIndexData = null;

        /// <summary>
        /// Intermediate storage for depth frame data
        /// </summary>
        private ushort[] depthData = null;

        /// <summary>
        /// Storage for body data from BodyFrame
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// Current tracked body
        /// </summary>
        private Body trackedBody = null;

        /// <summary>
        /// Intermediate storage for silhouette pixels (RGBA format)
        /// </summary>
        private byte[] silhouettePixels = null;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // Get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            if (this.kinectSensor != null)
            {
                // Get coordinate mapper
                this.coordinateMapper = this.kinectSensor.CoordinateMapper;

                // Get frame descriptions
                this.bodyIndexFrameDescription = this.kinectSensor.BodyIndexFrameSource.FrameDescription;
                this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

                // Open readers
                this.bodyIndexFrameReader = this.kinectSensor.BodyIndexFrameSource.OpenReader();
                this.bodyIndexFrameReader.FrameArrived += this.Reader_BodyIndexFrameArrived;

                this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();
                this.bodyFrameReader.FrameArrived += this.Reader_BodyFrameArrived;

                this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();
                this.depthFrameReader.FrameArrived += this.Reader_DepthFrameArrived;

                // Allocate storage arrays
                this.bodyIndexData = new byte[this.bodyIndexFrameDescription.Width * this.bodyIndexFrameDescription.Height];
                this.cleanedBodyIndexData = new byte[this.bodyIndexFrameDescription.Width * this.bodyIndexFrameDescription.Height];
                this.depthData = new ushort[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
                this.bodies = new Body[this.kinectSensor.BodyFrameSource.BodyCount];
                this.silhouettePixels = new byte[this.bodyIndexFrameDescription.Width * this.bodyIndexFrameDescription.Height * 4]; // RGBA

                // Create bitmap for silhouette display (RGBA format)
                this.silhouetteBitmap = new WriteableBitmap(
                    this.bodyIndexFrameDescription.Width,
                    this.bodyIndexFrameDescription.Height,
                    96.0,
                    96.0,
                    PixelFormats.Bgra32,
                    null);

                // Set IsAvailableChanged event notifier
                this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

                // Open the sensor
                this.kinectSensor.Open();
            }

            // Set the status text
            this.StatusText = this.kinectSensor != null && this.kinectSensor.IsAvailable
                ? "Kinect sensor ready"
                : "No sensor found";

            // Use the window object as the view model in this simple example
            this.DataContext = this;

            // Initialize the components (controls) of the window
            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.silhouetteBitmap;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // Notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyIndexFrameReader != null)
            {
                this.bodyIndexFrameReader.Dispose();
                this.bodyIndexFrameReader = null;
            }

            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.depthFrameReader != null)
            {
                this.depthFrameReader.Dispose();
                this.depthFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the user clicking on the screenshot button
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void ScreenshotButton_Click(object sender, RoutedEventArgs e)
        {
            if (this.silhouetteBitmap != null)
            {
                // Create a png bitmap encoder which knows how to save a .png file
                BitmapEncoder encoder = new PngBitmapEncoder();

                // Create frame from the writable bitmap and add to encoder
                encoder.Frames.Add(BitmapFrame.Create(this.silhouetteBitmap));

                string time = System.DateTime.UtcNow.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

                string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

                string path = Path.Combine(myPhotos, "KinectScreenshot-Silhouette-" + time + ".png");

                // Write the new file to disk
                try
                {
                    // FileStream is IDisposable
                    using (FileStream fs = new FileStream(path, FileMode.Create))
                    {
                        encoder.Save(fs);
                    }

                    this.StatusText = string.Format(CultureInfo.CurrentCulture, "Saved screenshot to {0}", path);
                }
                catch (IOException)
                {
                    this.StatusText = string.Format(CultureInfo.CurrentCulture, "Failed to save screenshot to {0}", path);
                }
            }
        }

        /// <summary>
        /// Handles the body index frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_BodyIndexFrameArrived(object sender, BodyIndexFrameArrivedEventArgs e)
        {
            bool bodyIndexFrameProcessed = false;

            using (BodyIndexFrame bodyIndexFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyIndexFrame != null)
                {
                    // Copy frame data to array
                    bodyIndexFrame.CopyFrameDataToArray(this.bodyIndexData);

                    bodyIndexFrameProcessed = true;
                }
            }

            if (bodyIndexFrameProcessed)
            {
                this.ProcessBodyIndexData();
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_BodyFrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool bodyFrameProcessed = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    // Get body data
                    bodyFrame.GetAndRefreshBodyData(this.bodies);

                    // Get first tracked body
                    this.trackedBody = this.bodies.FirstOrDefault(b => b != null && b.IsTracked);

                    bodyFrameProcessed = true;
                }
            }

            if (bodyFrameProcessed)
            {
                this.UpdateMeasurements();
            }
        }

        /// <summary>
        /// Handles the depth frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_DepthFrameArrived(object sender, DepthFrameArrivedEventArgs e)
        {
            bool depthFrameProcessed = false;

            using (DepthFrame depthFrame = e.FrameReference.AcquireFrame())
            {
                if (depthFrame != null)
                {
                    // Copy frame data to array
                    depthFrame.CopyFrameDataToArray(this.depthData);

                    depthFrameProcessed = true;
                }
            }

            if (depthFrameProcessed)
            {
                this.UpdateMeasurements();
            }
        }

        /// <summary>
        /// Processes body index data and converts it to a displayable bitmap
        /// BodyIndexData values: 0-5 = body pixels (different body IDs), 255 = background
        /// Applies morphological cleanup to remove noise
        /// </summary>
        private void ProcessBodyIndexData()
        {
            if (this.bodyIndexData == null || this.silhouetteBitmap == null)
            {
                return;
            }

            // Apply morphological cleanup (opening: erosion followed by dilation)
            // This removes small noise spots and smooths the silhouette
            this.ApplyMorphologicalCleanup(this.bodyIndexData, this.cleanedBodyIndexData, MORPHOLOGICAL_RADIUS);

            // Convert cleaned body index data to RGBA pixels
            // White = body (255, 255, 255, 255), Black = background (0, 0, 0, 255)
            for (int i = 0; i < this.cleanedBodyIndexData.Length; i++)
            {
                int pixelIndex = i * 4; // RGBA = 4 bytes per pixel

                if (this.cleanedBodyIndexData[i] != 255)
                {
                    // Body pixel - WHITE
                    this.silhouettePixels[pixelIndex] = 255;     // B
                    this.silhouettePixels[pixelIndex + 1] = 255; // G
                    this.silhouettePixels[pixelIndex + 2] = 255; // R
                    this.silhouettePixels[pixelIndex + 3] = 255; // A
                }
                else
                {
                    // Background - BLACK
                    this.silhouettePixels[pixelIndex] = 0;     // B
                    this.silhouettePixels[pixelIndex + 1] = 0; // G
                    this.silhouettePixels[pixelIndex + 2] = 0;   // R
                    this.silhouettePixels[pixelIndex + 3] = 255; // A
                }
            }

            // Render pixels to bitmap
            this.RenderSilhouettePixels();
        }

        /// <summary>
        /// Applies morphological opening (erosion followed by dilation) to clean up noise
        /// Erosion removes small isolated pixels, dilation restores body shape
        /// </summary>
        /// <param name="input">Input body index data</param>
        /// <param name="output">Output cleaned body index data</param>
        /// <param name="radius">Morphological operation radius</param>
        private void ApplyMorphologicalCleanup(byte[] input, byte[] output, int radius)
        {
            if (input == null || output == null || input.Length != output.Length)
            {
                return;
            }

            byte[] tempBuffer = new byte[input.Length];

            // Step 1: Erosion - removes small noise spots
            this.MorphologicalErosion(input, tempBuffer, radius);

            // Step 2: Dilation - restores body shape after erosion
            this.MorphologicalDilation(tempBuffer, output, radius);
        }

        /// <summary>
        /// Performs morphological erosion operation
        /// A pixel is kept (body) only if all pixels in its neighborhood are body pixels
        /// This removes small isolated noise spots
        /// </summary>
        /// <param name="input">Input body index data</param>
        /// <param name="output">Output eroded data</param>
        /// <param name="radius">Erosion radius</param>
        private void MorphologicalErosion(byte[] input, byte[] output, int radius)
        {
            for (int y = 0; y < DEPTH_HEIGHT; y++)
            {
                for (int x = 0; x < DEPTH_WIDTH; x++)
                {
                    int index = y * DEPTH_WIDTH + x;
                    bool isBody = input[index] != 255; // true if body pixel

                    // Check if all pixels in the neighborhood are body pixels
                    if (isBody)
                    {
                        bool allNeighborsAreBody = true;

                        for (int dy = -radius; dy <= radius && allNeighborsAreBody; dy++)
                        {
                            for (int dx = -radius; dx <= radius; dx++)
                            {
                                int nx = x + dx;
                                int ny = y + dy;

                                // Check bounds
                                if (nx >= 0 && nx < DEPTH_WIDTH && ny >= 0 && ny < DEPTH_HEIGHT)
                                {
                                    int neighborIndex = ny * DEPTH_WIDTH + nx;
                                    if (input[neighborIndex] == 255) // Background pixel found
                                    {
                                        allNeighborsAreBody = false;
                                        break;
                                    }
                                }
                                else
                                {
                                    // Outside bounds - treat as background
                                    allNeighborsAreBody = false;
                                    break;
                                }
                            }
                        }

                        output[index] = allNeighborsAreBody ? input[index] : (byte)255; // Keep body or set to background
                    }
                    else
                    {
                        output[index] = 255; // Background stays background
                    }
                }
            }
        }

        /// <summary>
        /// Performs morphological dilation operation
        /// A pixel becomes body if any pixel in its neighborhood is a body pixel
        /// This restores body shape after erosion
        /// </summary>
        /// <param name="input">Input body index data</param>
        /// <param name="output">Output dilated data</param>
        /// <param name="radius">Dilation radius</param>
        private void MorphologicalDilation(byte[] input, byte[] output, int radius)
        {
            for (int y = 0; y < DEPTH_HEIGHT; y++)
            {
                for (int x = 0; x < DEPTH_WIDTH; x++)
                {
                    int index = y * DEPTH_WIDTH + x;
                    bool isBody = input[index] != 255; // true if body pixel

                    // Check if any pixel in the neighborhood is a body pixel
                    if (isBody)
                    {
                        output[index] = input[index]; // Body pixel stays body
                    }
                    else
                    {
                        bool hasBodyNeighbor = false;

                        for (int dy = -radius; dy <= radius && !hasBodyNeighbor; dy++)
                        {
                            for (int dx = -radius; dx <= radius; dx++)
                            {
                                int nx = x + dx;
                                int ny = y + dy;

                                // Check bounds
                                if (nx >= 0 && nx < DEPTH_WIDTH && ny >= 0 && ny < DEPTH_HEIGHT)
                                {
                                    int neighborIndex = ny * DEPTH_WIDTH + nx;
                                    if (input[neighborIndex] != 255) // Body pixel found
                                    {
                                        hasBodyNeighbor = true;
                                        break;
                                    }
                                }
                            }
                        }

                        output[index] = hasBodyNeighbor ? (byte)0 : (byte)255; // Become body if neighbor is body, else stay background
                    }
                }
            }
        }

        /// <summary>
        /// Renders silhouette pixels into the writeableBitmap
        /// </summary>
        private void RenderSilhouettePixels()
        {
            if (this.silhouetteBitmap != null)
            {
                this.silhouetteBitmap.WritePixels(
                    new Int32Rect(0, 0, this.silhouetteBitmap.PixelWidth, this.silhouetteBitmap.PixelHeight),
                    this.silhouettePixels,
                    this.silhouetteBitmap.PixelWidth * 4, // stride (4 bytes per pixel for BGRA32)
                    0);
            }
        }

        /// <summary>
        /// Updates body width measurements using skeleton joints and coordinate mapping
        /// Uses cleaned body index data for more accurate measurements
        /// Also calculates all body part measurements from skeleton
        /// </summary>
        private void UpdateMeasurements()
        {
            if (this.trackedBody == null || !this.trackedBody.IsTracked)
            {
                this.MeasurementText.Text = "No body tracked";
                return;
            }

            // Calculate all body part measurements from skeleton
            HumanoidMeasurements measurements = this.MeasureKinectUserBodyParts(this.trackedBody);

            // Use cleaned body index data for width measurement (after morphological cleanup)
            byte[] measurementData = this.cleanedBodyIndexData != null ? this.cleanedBodyIndexData : this.bodyIndexData;

            double widthInMeters = 0;
            if (measurementData != null && this.depthData != null && this.coordinateMapper != null)
            {
                // Get SpineMid joint for width measurement
                var spineMidJoint = this.trackedBody.Joints[JointType.SpineMid];

                if (spineMidJoint.TrackingState != TrackingState.NotTracked)
                {
                    // Measure width at SpineMid using silhouette
                    widthInMeters = this.MeasureWidthAtJoint(spineMidJoint);
                }
            }

            // Display all measurements
            StringBuilder sb = new StringBuilder();
            sb.AppendLine(string.Format(CultureInfo.CurrentCulture, "Height: {0:F3} m ({1:F1} cm)", measurements.height, measurements.height * 100));
            sb.AppendLine(string.Format(CultureInfo.CurrentCulture, "Upper Arm: {0:F3} m | Lower Arm: {1:F3} m", measurements.upperArmLength, measurements.lowerArmLength));
            sb.AppendLine(string.Format(CultureInfo.CurrentCulture, "Upper Leg: {0:F3} m | Lower Leg: {1:F3} m", measurements.upperLegLength, measurements.lowerLegLength));
            sb.AppendLine(string.Format(CultureInfo.CurrentCulture, "Nape to Waist: {0:F3} m | Shoulder Dist: {1:F3} m", measurements.napeToWaist, measurements.shoulderDist));
            sb.AppendLine(string.Format(CultureInfo.CurrentCulture, "Waist to Hip: {0:F3} m | Neck Height: {1:F3} m", measurements.waistToHip, measurements.neckHeight));
            
            if (widthInMeters > 0)
            {
                sb.AppendLine(string.Format(CultureInfo.CurrentCulture, "SpineMid Width (from silhouette): {0:F3} m ({1:F1} cm)", widthInMeters, widthInMeters * 100));
            }

            this.MeasurementText.Text = sb.ToString();
        }

        /// <summary>
        /// Measures all body parts from skeleton joints (similar to Unity AvatarController)
        /// </summary>
        /// <param name="body">The tracked body</param>
        /// <returns>HumanoidMeasurements object with all measurements in meters</returns>
        private HumanoidMeasurements MeasureKinectUserBodyParts(Body body)
        {
            HumanoidMeasurements measurements = new HumanoidMeasurements();
            var joints = body.Joints;

            // Height: Head to average of feet
            var headJoint = joints[JointType.Head];
            var footLeftJoint = joints[JointType.FootLeft];
            var footRightJoint = joints[JointType.FootRight];

            if (headJoint.TrackingState != TrackingState.NotTracked &&
                footLeftJoint.TrackingState != TrackingState.NotTracked &&
                footRightJoint.TrackingState != TrackingState.NotTracked)
            {
                double headY = headJoint.Position.Y;
                double avgFootY = (footLeftJoint.Position.Y + footRightJoint.Position.Y) * 0.5;
                measurements.height = headY - avgFootY;
            }

            // Nape to Waist: SpineShoulder to SpineMid
            var spineShoulderJoint = joints[JointType.SpineShoulder];
            var spineMidJoint = joints[JointType.SpineMid];
            if (spineShoulderJoint.TrackingState != TrackingState.NotTracked &&
                spineMidJoint.TrackingState != TrackingState.NotTracked)
            {
                measurements.napeToWaist = Distance3D(
                    spineShoulderJoint.Position,
                    spineMidJoint.Position);
            }

            // Shoulder Distance: Average of distances from SpineShoulder to ShoulderLeft/Right
            var shoulderLeftJoint = joints[JointType.ShoulderLeft];
            var shoulderRightJoint = joints[JointType.ShoulderRight];
            if (spineShoulderJoint.TrackingState != TrackingState.NotTracked &&
                shoulderLeftJoint.TrackingState != TrackingState.NotTracked &&
                shoulderRightJoint.TrackingState != TrackingState.NotTracked)
            {
                double distLeft = Distance3D(spineShoulderJoint.Position, shoulderLeftJoint.Position);
                double distRight = Distance3D(spineShoulderJoint.Position, shoulderRightJoint.Position);
                measurements.shoulderDist = (distLeft + distRight) * 0.5;
            }

            // Waist to Hip: SpineBase to SpineMid
            var spineBaseJoint = joints[JointType.SpineBase];
            if (spineBaseJoint.TrackingState != TrackingState.NotTracked &&
                spineMidJoint.TrackingState != TrackingState.NotTracked)
            {
                measurements.waistToHip = Distance3D(
                    spineBaseJoint.Position,
                    spineMidJoint.Position);
            }

            // Neck Height: Head to Neck
            var neckJoint = joints[JointType.Neck];
            if (headJoint.TrackingState != TrackingState.NotTracked &&
                neckJoint.TrackingState != TrackingState.NotTracked)
            {
                measurements.neckHeight = Distance3D(
                    headJoint.Position,
                    neckJoint.Position);
            }

            // Upper Arm Length: Shoulder to Elbow (average of left and right)
            var elbowLeftJoint = joints[JointType.ElbowLeft];
            var elbowRightJoint = joints[JointType.ElbowRight];
            if (shoulderLeftJoint.TrackingState != TrackingState.NotTracked &&
                elbowLeftJoint.TrackingState != TrackingState.NotTracked &&
                shoulderRightJoint.TrackingState != TrackingState.NotTracked &&
                elbowRightJoint.TrackingState != TrackingState.NotTracked)
            {
                double upperArmLeft = Distance3D(shoulderLeftJoint.Position, elbowLeftJoint.Position);
                double upperArmRight = Distance3D(shoulderRightJoint.Position, elbowRightJoint.Position);
                measurements.upperArmLength = (upperArmLeft + upperArmRight) * 0.5;
            }

            // Lower Arm Length: Elbow to Wrist (average of left and right)
            var wristLeftJoint = joints[JointType.WristLeft];
            var wristRightJoint = joints[JointType.WristRight];
            if (elbowLeftJoint.TrackingState != TrackingState.NotTracked &&
                wristLeftJoint.TrackingState != TrackingState.NotTracked &&
                elbowRightJoint.TrackingState != TrackingState.NotTracked &&
                wristRightJoint.TrackingState != TrackingState.NotTracked)
            {
                double lowerArmLeft = Distance3D(elbowLeftJoint.Position, wristLeftJoint.Position);
                double lowerArmRight = Distance3D(elbowRightJoint.Position, wristRightJoint.Position);
                measurements.lowerArmLength = (lowerArmLeft + lowerArmRight) * 0.5;
            }

            // Upper Leg Length: Hip to Knee (average of left and right)
            var hipLeftJoint = joints[JointType.HipLeft];
            var hipRightJoint = joints[JointType.HipRight];
            var kneeLeftJoint = joints[JointType.KneeLeft];
            var kneeRightJoint = joints[JointType.KneeRight];
            if (hipLeftJoint.TrackingState != TrackingState.NotTracked &&
                kneeLeftJoint.TrackingState != TrackingState.NotTracked &&
                hipRightJoint.TrackingState != TrackingState.NotTracked &&
                kneeRightJoint.TrackingState != TrackingState.NotTracked)
            {
                double upperLegLeft = Distance3D(hipLeftJoint.Position, kneeLeftJoint.Position);
                double upperLegRight = Distance3D(hipRightJoint.Position, kneeRightJoint.Position);
                measurements.upperLegLength = (upperLegLeft + upperLegRight) * 0.5;
            }

            // Lower Leg Length: Knee to Ankle (average of left and right)
            var ankleLeftJoint = joints[JointType.AnkleLeft];
            var ankleRightJoint = joints[JointType.AnkleRight];
            if (kneeLeftJoint.TrackingState != TrackingState.NotTracked &&
                ankleLeftJoint.TrackingState != TrackingState.NotTracked &&
                kneeRightJoint.TrackingState != TrackingState.NotTracked &&
                ankleRightJoint.TrackingState != TrackingState.NotTracked)
            {
                double lowerLegLeft = Distance3D(kneeLeftJoint.Position, ankleLeftJoint.Position);
                double lowerLegRight = Distance3D(kneeRightJoint.Position, ankleRightJoint.Position);
                measurements.lowerLegLength = (lowerLegLeft + lowerLegRight) * 0.5;
            }

            return measurements;
        }

        /// <summary>
        /// Calculates 3D Euclidean distance between two CameraSpacePoints
        /// </summary>
        /// <param name="p1">First point</param>
        /// <param name="p2">Second point</param>
        /// <returns>Distance in meters</returns>
        private double Distance3D(CameraSpacePoint p1, CameraSpacePoint p2)
        {
            double dx = p1.X - p2.X;
            double dy = p1.Y - p2.Y;
            double dz = p1.Z - p2.Z;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        /// <summary>
        /// Measures the body width at a specific joint by scanning the silhouette horizontally.
        /// Uses CoordinateMapper to convert 3D joint position to 2D depth space coordinates.
        /// Uses cleaned body index data for more accurate measurements.
        /// </summary>
        /// <param name="joint">The joint at which to measure width</param>
        /// <returns>Width in meters, or 0 if measurement failed</returns>
        private double MeasureWidthAtJoint(Joint joint)
        {
            if (joint.TrackingState == TrackingState.NotTracked)
            {
                return 0;
            }

            // Use cleaned body index data for measurements (after morphological cleanup)
            byte[] measurementData = this.cleanedBodyIndexData != null ? this.cleanedBodyIndexData : this.bodyIndexData;

            if (measurementData == null || this.depthData == null || this.coordinateMapper == null)
            {
                return 0;
            }

            // Convert joint position (3D camera space) to depth space (2D pixels)
            DepthSpacePoint depthPoint = this.coordinateMapper.MapCameraPointToDepthSpace(joint.Position);

            int centerX = (int)(depthPoint.X + 0.5f);
            int centerY = (int)(depthPoint.Y + 0.5f);

            // Check bounds
            if (centerX < 0 || centerX >= DEPTH_WIDTH || centerY < 0 || centerY >= DEPTH_HEIGHT)
            {
                return 0;
            }

            // Scan LEFT from center to find left edge
            int leftEdge = centerX;
            for (int x = centerX; x >= 0; x--)
            {
                int index = centerY * DEPTH_WIDTH + x;
                if (measurementData[index] == 255) // Not a body pixel
                {
                    leftEdge = x + 1;
                    break;
                }
                if (x == 0) leftEdge = 0;
            }

            // Scan RIGHT from center to find right edge
            int rightEdge = centerX;
            for (int x = centerX; x < DEPTH_WIDTH; x++)
            {
                int index = centerY * DEPTH_WIDTH + x;
                if (measurementData[index] == 255) // Not a body pixel
                {
                    rightEdge = x - 1;
                    break;
                }
                if (x == DEPTH_WIDTH - 1) rightEdge = DEPTH_WIDTH - 1;
            }

            int widthInPixels = rightEdge - leftEdge + 1;

            // Get depth at the center point (in millimeters)
            int centerIndex = centerY * DEPTH_WIDTH + centerX;
            float depthMm = this.depthData[centerIndex];

            if (depthMm <= 0)
            {
                return 0;
            }

            // Convert pixels to meters using depth and FOV
            double widthInMeters = this.PixelsToMeters(widthInPixels, depthMm);

            return widthInMeters;
        }

        /// <summary>
        /// Converts a horizontal pixel distance to real-world meters at a given depth.
        /// </summary>
        /// <param name="pixels">Pixel width to convert</param>
        /// <param name="depthMm">Depth in millimeters</param>
        /// <returns>Width in meters</returns>
        private double PixelsToMeters(int pixels, float depthMm)
        {
            double depthM = depthMm / 1000.0;
            double halfFovRad = (DEPTH_HORIZONTAL_FOV / 2.0) * (Math.PI / 180.0); // Convert degrees to radians
            double frameWidthAtDepth = 2.0 * depthM * Math.Tan(halfFovRad);
            double metersPerPixel = frameWidthAtDepth / DEPTH_WIDTH;
            return pixels * metersPerPixel;
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // On failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable
                ? "Kinect sensor ready"
                : "Sensor not available";
        }
    }
}

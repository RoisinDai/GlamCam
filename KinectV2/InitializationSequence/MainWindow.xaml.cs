//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.SilhouetteBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Linq;
    using System.Text;
    using System.Threading;
    using System.Threading.Tasks;
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
        /// Reader for color frames
        /// </summary>
        private ColorFrameReader colorFrameReader = null;

        /// <summary>
        /// Bitmap to display color frame
        /// </summary>
        private WriteableBitmap colorBitmap = null;

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
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Current body measurements (stored for model generation)
        /// </summary>
        private HumanoidMeasurements currentMeasurements = null;

        /// <summary>
        /// Flag to track if we've already captured measurements
        /// </summary>
        private bool measurementsCaptured = false;

        /// <summary>
        /// Flag to track if model generation is in progress
        /// </summary>
        private bool modelGenerationInProgress = false;

        /// <summary>
        /// Counter for consecutive frames where T-pose is detected
        /// </summary>
        private int tPoseFrameCount = 0;

        /// <summary>
        /// Number of consecutive T-pose frames required before capturing (2 seconds at ~30fps)
        /// </summary>
        private const int REQUIRED_TPOSE_FRAMES = 60;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // Get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            // Debug logging
            Console.WriteLine("========== Kinect Initialization ==========");
            Console.WriteLine($"KinectSensor.GetDefault() returned: {(this.kinectSensor != null ? "NOT NULL" : "NULL")}");
            if (this.kinectSensor != null)
            {
                Console.WriteLine($"Kinect IsAvailable: {this.kinectSensor.IsAvailable}");
                Console.WriteLine($"Kinect IsOpen: {this.kinectSensor.IsOpen}");
                Console.WriteLine($"Kinect UniqueKinectId: {this.kinectSensor.UniqueKinectId}");
            }
            Console.WriteLine("==========================================");

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

                this.colorFrameReader = this.kinectSensor.ColorFrameSource.OpenReader();
                this.colorFrameReader.FrameArrived += this.Reader_ColorFrameArrived;

                // Create bitmap for color display
                FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
                this.colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

                // Allocate storage arrays
                this.bodyIndexData = new byte[this.bodyIndexFrameDescription.Width * this.bodyIndexFrameDescription.Height];
                this.cleanedBodyIndexData = new byte[this.bodyIndexFrameDescription.Width * this.bodyIndexFrameDescription.Height];
                this.depthData = new ushort[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
                this.bodies = new Body[this.kinectSensor.BodyFrameSource.BodyCount];

                // Set IsAvailableChanged event notifier
                this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

                // Open the sensor
                this.kinectSensor.Open();
            }

            // Set the status text
            this.StatusText = this.kinectSensor != null && this.kinectSensor.IsAvailable
                ? "Kinect sensor ready - Waiting for body tracking..."
                : "No sensor found - Using hardcoded measurements";

            // Use the window object as the view model in this simple example
            this.DataContext = this;

            // Initialize the components (controls) of the window
            this.InitializeComponent();

            // Set the image source for color display
            if (this.colorBitmap != null)
            {
                this.ColorImageView.Source = this.colorBitmap;
            }

            // Wait for Kinect to become available before falling back to hardcoded measurements
            if (this.kinectSensor == null)
            {
                // No Kinect sensor found at all - use hardcoded immediately
                Console.WriteLine("[FALLBACK] No Kinect sensor detected - using hardcoded measurements");
                var timer = new System.Windows.Threading.DispatcherTimer();
                timer.Interval = TimeSpan.FromSeconds(1);
                timer.Tick += (s, e) =>
                {
                    timer.Stop();
                    this.TriggerHardcodedMeasurements();
                };
                timer.Start();
            }
            else if (!this.kinectSensor.IsAvailable)
            {
                // Kinect exists but not available yet - wait up to 10 seconds for it to become available
                Console.WriteLine("[WAITING] Kinect sensor found but not available yet. Waiting up to 10 seconds...");
                int waitSeconds = 0;
                var timer = new System.Windows.Threading.DispatcherTimer();
                timer.Interval = TimeSpan.FromSeconds(1);
                timer.Tick += (s, e) =>
                {
                    waitSeconds++;
                    Console.WriteLine($"[WAITING] Checking Kinect availability... ({waitSeconds}/10 seconds)");

                    if (this.kinectSensor.IsAvailable)
                    {
                        timer.Stop();
                        Console.WriteLine("[SUCCESS] Kinect sensor is now available!");
                        this.StatusText = "Kinect sensor ready - Waiting for body tracking...";
                    }
                    else if (waitSeconds >= 10)
                    {
                        timer.Stop();
                        Console.WriteLine("[FALLBACK] Kinect did not become available after 10 seconds - using hardcoded measurements");
                        this.TriggerHardcodedMeasurements();
                    }
                };
                timer.Start();
            }
            else
            {
                // Kinect is already available
                Console.WriteLine("[SUCCESS] Kinect sensor is available and ready!");
            }
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;


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

            if (this.colorFrameReader != null)
            {
                this.colorFrameReader.Dispose();
                this.colorFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
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

            if (bodyIndexFrameProcessed && !this.measurementsCaptured)
            {
                // Process body index data for measurements only (skip rendering)
                this.ProcessBodyIndexDataForMeasurements();
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

            if (bodyFrameProcessed && !this.measurementsCaptured)
            {
                // Only update measurements if we haven't captured yet
                this.UpdateMeasurements();
            }
        }

        /// <summary>
        /// Handles the color frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_ColorFrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            using (ColorFrame colorFrame = e.FrameReference.AcquireFrame())
            {
                if (colorFrame != null)
                {
                    FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                    {
                        this.colorBitmap.Lock();

                        // Verify data and write to the display bitmap
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
                }
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

            if (depthFrameProcessed && !this.measurementsCaptured)
            {
                // Only update measurements if we haven't captured yet
                this.UpdateMeasurements();
            }
        }

        /// <summary>
        /// Processes body index data for measurements only (no rendering)
        /// Applies morphological cleanup to remove noise for accurate measurements
        /// </summary>
        private void ProcessBodyIndexDataForMeasurements()
        {
            if (this.bodyIndexData == null)
            {
                return;
            }

            // Apply morphological cleanup (opening: erosion followed by dilation)
            // This removes small noise spots and smooths the silhouette for better measurements
            this.ApplyMorphologicalCleanup(this.bodyIndexData, this.cleanedBodyIndexData, MORPHOLOGICAL_RADIUS);
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
        /// Updates body width measurements using skeleton joints and coordinate mapping
        /// Uses cleaned body index data for more accurate measurements
        /// Also calculates all body part measurements from skeleton
        /// </summary>
        private void UpdateMeasurements()
        {
            if (this.trackedBody == null || !this.trackedBody.IsTracked)
            {
                if (!this.measurementsCaptured && !this.modelGenerationInProgress)
                {
                    this.MeasurementText.Text = "PLEASE STAND IN T-POSE\n\n" +
                        "Instructions:\n" +
                        "• Face the camera\n" +
                        "• Stand upright\n" +
                        "• Arms straight out to sides\n" +
                        "• Legs slightly apart\n\n" +
                        "Waiting for body detection...";
                }
                return;
            }

            // Calculate all body part measurements from skeleton
            HumanoidMeasurements measurements = this.MeasureKinectUserBodyParts(this.trackedBody);
            
            // Store measurements for model generation (freeze once we've captured)
            if (!this.measurementsCaptured && !this.modelGenerationInProgress)
            {
                this.currentMeasurements = measurements;
            }

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

            // Check if we have valid measurements
            bool hasValidMeasurements = measurements.height > 0 &&
                measurements.upperArmLength > 0 &&
                measurements.lowerArmLength > 0 &&
                measurements.upperLegLength > 0 &&
                measurements.lowerLegLength > 0;

            // Check T-pose and update counter
            bool isInTPose = hasValidMeasurements && this.IsInTPose(this.trackedBody);

            if (!this.measurementsCaptured && !this.modelGenerationInProgress)
            {
                if (isInTPose)
                {
                    this.tPoseFrameCount++;
                }
                else
                {
                    this.tPoseFrameCount = 0; // Reset if not in T-pose
                }
            }

            // Display measurements or T-pose instructions
            StringBuilder sb = new StringBuilder();

            if (!this.measurementsCaptured && !this.modelGenerationInProgress)
            {
                // Show T-pose status and countdown
                if (isInTPose && this.tPoseFrameCount < REQUIRED_TPOSE_FRAMES)
                {
                    // Calculate progress percentage
                    double progressPercent = (double)this.tPoseFrameCount / REQUIRED_TPOSE_FRAMES * 100;
                    double secondsRemaining = (REQUIRED_TPOSE_FRAMES - this.tPoseFrameCount) / 30.0; // Assuming ~30fps

                    sb.AppendLine("✓ T-POSE DETECTED!");
                    sb.AppendLine();
                    sb.AppendLine("HOLD STEADY...");
                    sb.AppendLine();
                    sb.AppendLine(string.Format("Progress: {0:F0}%", progressPercent));
                    sb.AppendLine(string.Format("Time remaining: {0:F1}s", secondsRemaining));
                    sb.AppendLine();
                    sb.AppendLine("Keep your arms extended");
                    sb.AppendLine("and body upright!");
                }
                else if (!isInTPose)
                {
                    // Show T-pose instructions
                    sb.AppendLine("PLEASE STAND IN T-POSE");
                    sb.AppendLine();
                    sb.AppendLine("Instructions:");
                    sb.AppendLine("• Face the camera");
                    sb.AppendLine("• Stand upright");
                    sb.AppendLine("• Arms straight out to sides");
                    sb.AppendLine("• Legs slightly apart");
                    sb.AppendLine();

                    if (hasValidMeasurements)
                    {
                        sb.AppendLine("Body detected! Extend your arms...");
                    }
                    else
                    {
                        sb.AppendLine("Waiting for body detection...");
                    }
                }

                // Show current measurements below instructions
                if (hasValidMeasurements)
                {
                    sb.AppendLine();
                    sb.AppendLine("--- Current Measurements ---");
                    sb.AppendLine(string.Format(CultureInfo.CurrentCulture, "Height: {0:F3} m ({1:F1} cm)", measurements.height, measurements.height * 100));
                    sb.AppendLine(string.Format(CultureInfo.CurrentCulture, "Upper Arm: {0:F3} m | Lower Arm: {1:F3} m", measurements.upperArmLength, measurements.lowerArmLength));
                    sb.AppendLine(string.Format(CultureInfo.CurrentCulture, "Upper Leg: {0:F3} m | Lower Leg: {1:F3} m", measurements.upperLegLength, measurements.lowerLegLength));
                }
            }
            else
            {
                // After capture, show all measurements normally
                sb.AppendLine(string.Format(CultureInfo.CurrentCulture, "Height: {0:F3} m ({1:F1} cm)", measurements.height, measurements.height * 100));
                sb.AppendLine(string.Format(CultureInfo.CurrentCulture, "Upper Arm: {0:F3} m | Lower Arm: {1:F3} m", measurements.upperArmLength, measurements.lowerArmLength));
                sb.AppendLine(string.Format(CultureInfo.CurrentCulture, "Upper Leg: {0:F3} m | Lower Leg: {1:F3} m", measurements.upperLegLength, measurements.lowerLegLength));
                sb.AppendLine(string.Format(CultureInfo.CurrentCulture, "Nape to Waist: {0:F3} m | Shoulder Dist: {1:F3} m", measurements.napeToWaist, measurements.shoulderDist));
                sb.AppendLine(string.Format(CultureInfo.CurrentCulture, "Waist to Hip: {0:F3} m | Neck Height: {1:F3} m", measurements.waistToHip, measurements.neckHeight));

                if (widthInMeters > 0)
                {
                    sb.AppendLine(string.Format(CultureInfo.CurrentCulture, "SpineMid Width (from silhouette): {0:F3} m ({1:F1} cm)", widthInMeters, widthInMeters * 100));
                }
            }

            this.MeasurementText.Text = sb.ToString();

            // Automatically capture and generate when T-pose is held for required frames
            if (!this.measurementsCaptured && !this.modelGenerationInProgress &&
                this.tPoseFrameCount >= REQUIRED_TPOSE_FRAMES)
            {
                double? widthForLog = widthInMeters > 0 ? (double?)widthInMeters : null;
                LogMeasurementsToConsole(measurements, widthForLog, "Kinect");
                this.CaptureMeasurementsAndGenerate();
            }
        }

        /// <summary>
        /// Validates if the tracked body is in a proper T-pose
        /// </summary>
        /// <param name="body">The tracked body to validate</param>
        /// <returns>True if body is in T-pose, false otherwise</returns>
        private bool IsInTPose(Body body)
        {
            if (body == null || !body.IsTracked)
            {
                return false;
            }

            var joints = body.Joints;

            // Check that all critical joints are tracked
            JointType[] criticalJoints = new JointType[]
            {
                JointType.Head, JointType.Neck, JointType.SpineShoulder,
                JointType.ShoulderLeft, JointType.ShoulderRight,
                JointType.ElbowLeft, JointType.ElbowRight,
                JointType.WristLeft, JointType.WristRight,
                JointType.SpineMid, JointType.SpineBase,
                JointType.HipLeft, JointType.HipRight,
                JointType.KneeLeft, JointType.KneeRight,
                JointType.AnkleLeft, JointType.AnkleRight
            };

            foreach (var jointType in criticalJoints)
            {
                if (joints[jointType].TrackingState == TrackingState.NotTracked)
                {
                    return false;
                }
            }

            // Check arm extension (T-pose)
            // Left arm should be extended horizontally
            var shoulderLeft = joints[JointType.ShoulderLeft].Position;
            var elbowLeft = joints[JointType.ElbowLeft].Position;
            var wristLeft = joints[JointType.WristLeft].Position;

            // Right arm should be extended horizontally
            var shoulderRight = joints[JointType.ShoulderRight].Position;
            var elbowRight = joints[JointType.ElbowRight].Position;
            var wristRight = joints[JointType.WristRight].Position;

            // Check that elbows and wrists are roughly at shoulder level (Y coordinate)
            // Allow 0.2m tolerance (about 8 inches)
            const float yTolerance = 0.2f;

            bool leftArmExtended =
                Math.Abs(elbowLeft.Y - shoulderLeft.Y) < yTolerance &&
                Math.Abs(wristLeft.Y - shoulderLeft.Y) < yTolerance &&
                wristLeft.X < elbowLeft.X && // Wrist should be further left than elbow
                elbowLeft.X < shoulderLeft.X; // Elbow should be further left than shoulder

            bool rightArmExtended =
                Math.Abs(elbowRight.Y - shoulderRight.Y) < yTolerance &&
                Math.Abs(wristRight.Y - shoulderRight.Y) < yTolerance &&
                wristRight.X > elbowRight.X && // Wrist should be further right than elbow
                elbowRight.X > shoulderRight.X; // Elbow should be further right than shoulder

            // Check that body is upright (spine should be mostly vertical)
            var spineBase = joints[JointType.SpineBase].Position;
            var spineShoulder = joints[JointType.SpineShoulder].Position;

            // Horizontal offset should be small compared to vertical height
            float spineHorizontalOffset = Math.Abs(spineShoulder.X - spineBase.X);
            float spineVerticalHeight = Math.Abs(spineShoulder.Y - spineBase.Y);

            bool bodyUpright = spineVerticalHeight > 0.3f && // Reasonable torso height
                               spineHorizontalOffset / spineVerticalHeight < 0.3f; // Not leaning too much

            return leftArmExtended && rightArmExtended && bodyUpright;
        }

        /// <summary>
        /// Writes all measurements to the console (terminal).
        /// </summary>
        /// <param name="m">Body measurements in meters</param>
        /// <param name="spineMidWidthM">Optional SpineMid width from silhouette (meters). Omit if not measured.</param>
        /// <param name="source">Optional label, e.g. "Kinect" or "hardcoded"</param>
        private void LogMeasurementsToConsole(HumanoidMeasurements m, double? spineMidWidthM = null, string source = null)
        {
            Console.WriteLine("---------- Measurements ----------");
            if (!string.IsNullOrEmpty(source))
            {
                Console.WriteLine("  Source: {0}", source);
            }

            Console.WriteLine(string.Format(CultureInfo.InvariantCulture, "  Height: {0:F3} m ({1:F1} cm)", m.height, m.height * 100));
            Console.WriteLine(string.Format(CultureInfo.InvariantCulture, "  Upper Arm: {0:F3} m | Lower Arm: {1:F3} m", m.upperArmLength, m.lowerArmLength));
            Console.WriteLine(string.Format(CultureInfo.InvariantCulture, "  Upper Leg: {0:F3} m | Lower Leg: {1:F3} m", m.upperLegLength, m.lowerLegLength));
            Console.WriteLine(string.Format(CultureInfo.InvariantCulture, "  Nape to Waist: {0:F3} m | Shoulder Dist: {1:F3} m", m.napeToWaist, m.shoulderDist));
            Console.WriteLine(string.Format(CultureInfo.InvariantCulture, "  Waist to Hip: {0:F3} m | Neck Height: {1:F3} m", m.waistToHip, m.neckHeight));

            if (spineMidWidthM.HasValue && spineMidWidthM.Value > 0)
            {
                Console.WriteLine(string.Format(CultureInfo.InvariantCulture, "  SpineMid Width (silhouette): {0:F3} m ({1:F1} cm)", spineMidWidthM.Value, spineMidWidthM.Value * 100));
            }

            Console.WriteLine("----------------------------------");
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

        /// <summary>
        /// Triggers hardcoded measurements when no Kinect is available
        /// </summary>
        private void TriggerHardcodedMeasurements()
        {
            if (this.measurementsCaptured || this.modelGenerationInProgress)
            {
                return;
            }

            // Create hardcoded measurements (in meters) - only these 5 are passed to MakeHuman script
            this.currentMeasurements = new HumanoidMeasurements
            {
                height = 1.69,           // 169.0 cm
                upperArmLength = 0.29,   // 29.0 cm
                lowerArmLength = 0.25,   // 25.0 cm
                upperLegLength = 0.385,   // 38.5 cm
                lowerLegLength = 0.48,   // 48.0 cm
                napeToWaist = 0.30,       // Not used in script
                shoulderDist = 0.40,     // Not used in script
                waistToHip = 0.15,       // Not used in script
                neckHeight = 0.10         // Not used in script
            };
            
            // Update display with hardcoded measurements
            // StringBuilder sb = new StringBuilder();
            // sb.AppendLine(string.Format(CultureInfo.CurrentCulture, "Height: {0:F3} m ({1:F1} cm)", this.currentMeasurements.height, this.currentMeasurements.height * 100));
            // sb.AppendLine(string.Format(CultureInfo.CurrentCulture, "Upper Arm: {0:F3} m | Lower Arm: {1:F3} m", this.currentMeasurements.upperArmLength, this.currentMeasurements.lowerArmLength));
            // sb.AppendLine(string.Format(CultureInfo.CurrentCulture, "Upper Leg: {0:F3} m | Lower Leg: {1:F3} m", this.currentMeasurements.upperLegLength, this.currentMeasurements.lowerLegLength));
            // sb.AppendLine(string.Format(CultureInfo.CurrentCulture, "Nape to Waist: {0:F3} m | Shoulder Dist: {1:F3} m", this.currentMeasurements.napeToWaist, this.currentMeasurements.shoulderDist));
            // sb.AppendLine(string.Format(CultureInfo.CurrentCulture, "Waist to Hip: {0:F3} m | Neck Height: {1:F3} m", this.currentMeasurements.waistToHip, this.currentMeasurements.neckHeight));
            // sb.AppendLine("(Using hardcoded measurements - no Kinect)");
            // this.MeasurementText.Text = sb.ToString();
            
            LogMeasurementsToConsole(this.currentMeasurements, null, "hardcoded (no Kinect)");
            this.CaptureMeasurementsAndGenerate();
        }

        /// <summary>
        /// Captures measurements and automatically generates the model
        /// </summary>
        private async void CaptureMeasurementsAndGenerate()
        {
            if (this.measurementsCaptured || this.modelGenerationInProgress)
            {
                return;
            }

            if (this.currentMeasurements == null)
            {
                return;
            }

            // Mark as captured immediately to prevent duplicate calls
            this.measurementsCaptured = true;
            this.modelGenerationInProgress = true;

            this.StatusText = "✓ Measurements captured! Generating your 3D model...";

            // Update measurement display to show generation status
            this.MeasurementText.Text = "✓ MEASUREMENTS CAPTURED!\n\n" +
                "Generating your 3D model...\n\n" +
                "Please wait, this may take 1-2 minutes.";

            // Wait a brief moment to ensure measurements are stable
            await Task.Delay(500);

            // Automatically generate the model
            await Task.Run(() =>
            {
                this.Dispatcher.Invoke(() =>
                {
                    this.GenerateMakeHumanModel(this.currentMeasurements);
                });
            });
        }


        /// <summary>
        /// Generates a MakeHuman model using the provided measurements
        /// </summary>
        /// <param name="measurements">Body measurements in meters</param>
        private void GenerateMakeHumanModel(HumanoidMeasurements measurements)
        {
            try
            {
                this.StatusText = "Generating model...";

                // Hardcoded paths
                string makehumanProjectRoot = "C:\\Users\\roisi\\Desktop\\makehuman";
                string makehumanScript = "C:\\Users\\roisi\\Desktop\\makehuman\\makehuman\\generate_human.py";
                string rigPath = "C:\\Users\\roisi\\OneDrive\\Documents\\makehuman\\v1py3\\data\\rigs\\Unity_Rig\\unity.mhskel";
                string clothesDir = "C:\\Users\\roisi\\OneDrive\\Documents\\makehuman\\v1py3\\data\\clothes";
                string outputDir = "C:\\Users\\roisi\\Desktop\\GlamCam\\GlamCam\\Assets\\Base Models";
                string mhmDir = "C:\\Users\\roisi\\Desktop\\makehuman\\makehuman\\tmp";

                // Create directories if they don't exist
                if (!Directory.Exists(outputDir))
                {
                    Directory.CreateDirectory(outputDir);
                }

                if (!Directory.Exists(mhmDir))
                {
                    Directory.CreateDirectory(mhmDir);
                }

                // Python executable (use python3 on macOS/Linux, python on Windows)
                string pythonExe = "python3";
                if (Environment.OSVersion.Platform == PlatformID.Win32NT)
                {
                    // Try venv first
                    string venvPython = Path.Combine(makehumanProjectRoot, "venv", "Scripts", "python.exe");
                    if (File.Exists(venvPython))
                    {
                        pythonExe = venvPython;
                    }
                    else
                    {
                        pythonExe = "python";
                    }
                }
                else
                {
                    // macOS/Linux - try venv first
                    string venvPython = Path.Combine(makehumanProjectRoot, "venv", "bin", "python3");
                    if (File.Exists(venvPython))
                    {
                        pythonExe = venvPython;
                    }
                }

                // Convert measurements from meters to centimeters
                double heightCm = measurements.height * 100.0;
                double upperArmCm = measurements.upperArmLength * 100.0;
                double lowerArmCm = measurements.lowerArmLength * 100.0;
                double upperLegCm = measurements.upperLegLength * 100.0;
                double lowerLegCm = measurements.lowerLegLength * 100.0;

                // Log measurements passed to MakeHuman script to Console
                Console.WriteLine("[MakeHuman] Passing measured values to script:");
                Console.WriteLine(string.Format(CultureInfo.InvariantCulture, "  height={0:F1} cm, upper-arm={1:F1} cm, lower-arm={2:F1} cm, upper-leg={3:F1} cm, lower-leg={4:F1} cm",
                    heightCm, upperArmCm, lowerArmCm, upperLegCm, lowerLegCm));

                // Build command-line arguments
                string arguments = string.Format(
                    CultureInfo.InvariantCulture,
                    "\"{0}\" --height {1:F1} --upper-arm {2:F1} --lower-arm {3:F1} --upper-leg {4:F1} --lower-leg {5:F1} --rig-path \"{6}\" --clothes-dir \"{7}\" --output-dir \"{8}\" --mhm-dir \"{9}\" --pose tpose --tolerance 0.3",
                    makehumanScript,
                    heightCm,
                    upperArmCm,
                    lowerArmCm,
                    upperLegCm,
                    lowerLegCm,
                    rigPath,
                    clothesDir,
                    outputDir,
                    mhmDir);

                this.StatusText = "Starting MakeHuman script...";

                // Create process start info
                ProcessStartInfo startInfo = new ProcessStartInfo
                {
                    FileName = pythonExe,
                    Arguments = arguments,
                    UseShellExecute = false,
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    CreateNoWindow = true,
                    WorkingDirectory = makehumanProjectRoot,
                    StandardOutputEncoding = Encoding.UTF8,
                    StandardErrorEncoding = Encoding.UTF8
                };

                // Force UTF-8 encoding for Python output to handle Unicode characters
                startInfo.EnvironmentVariables["PYTHONIOENCODING"] = "utf-8";

                // Start the process
                using (Process process = Process.Start(startInfo))
                {
                    if (process == null)
                    {
                        throw new Exception("Failed to start MakeHuman script process");
                    }

                    this.StatusText = "Generating model (this may take a while)...";

                    // Read output asynchronously
                    StringBuilder output = new StringBuilder();
                    StringBuilder error = new StringBuilder();

                    process.OutputDataReceived += (sender, e) =>
                    {
                        if (!string.IsNullOrEmpty(e.Data))
                        {
                            output.AppendLine(e.Data);
                            Console.WriteLine($"[MAKEHUMAN OUTPUT] {e.Data}");
                        }
                    };

                    process.ErrorDataReceived += (sender, e) =>
                    {
                        if (!string.IsNullOrEmpty(e.Data))
                        {
                            error.AppendLine(e.Data);
                            Console.WriteLine($"[MAKEHUMAN ERROR] {e.Data}");
                        }
                    };

                    // Start reading output asynchronously
                    process.BeginOutputReadLine();
                    process.BeginErrorReadLine();

                    // Wait for completion with timeout (5 minutes)
                    bool completed = process.WaitForExit(300000);

                    if (!completed)
                    {
                        process.Kill();
                        throw new Exception("MakeHuman script timed out after 5 minutes");
                    }

                    if (process.ExitCode != 0)
                    {
                        string errorMsg = error.ToString();
                        if (string.IsNullOrEmpty(errorMsg))
                        {
                            errorMsg = output.ToString();
                        }
                        throw new Exception($"MakeHuman script failed with exit code {process.ExitCode}:\n{errorMsg}");
                    }

                    // Check if model file was created
                    string modelFile = Path.Combine(outputDir, "clothed_avatar.fbx");
                    if (File.Exists(modelFile))
                    {
                        this.StatusText = "Model generated successfully!";

                        // Update measurement display to show success
                        this.MeasurementText.Text = "✓ SUCCESS!\n\n" +
                            "Your 3D model has been generated!\n\n" +
                            "Closing in 2 seconds...";

                        // Log success to console
                        Console.WriteLine("========================================");
                        Console.WriteLine("[SUCCESS] Model generated successfully!");
                        Console.WriteLine($"  Model file: {modelFile}");
                        Console.WriteLine("========================================");

                        // Close the window automatically after a short delay
                        var closeTimer = new System.Windows.Threading.DispatcherTimer();
                        closeTimer.Interval = TimeSpan.FromSeconds(2);
                        closeTimer.Tick += (s, e) =>
                        {
                            closeTimer.Stop();
                            Console.WriteLine("[INFO] Closing application...");
                            this.Close();
                        };
                        closeTimer.Start();
                    }
                    else
                    {
                        this.StatusText = "Warning: Model file not found";

                        // Update measurement display to show warning
                        this.MeasurementText.Text = "⚠ WARNING\n\n" +
                            "Model file was not found.\n\n" +
                            "Please check the console output for errors.\n\n" +
                            "Closing in 3 seconds...";

                        // Log warning to console
                        Console.WriteLine("========================================");
                        Console.WriteLine("[WARNING] Model file not found!");
                        Console.WriteLine($"  Expected: {modelFile}");
                        Console.WriteLine("  Please check the script output for errors.");
                        Console.WriteLine("========================================");

                        // Close the window after a short delay even on failure
                        var closeTimer = new System.Windows.Threading.DispatcherTimer();
                        closeTimer.Interval = TimeSpan.FromSeconds(3);
                        closeTimer.Tick += (s, e) =>
                        {
                            closeTimer.Stop();
                            Console.WriteLine("[INFO] Closing application...");
                            this.Close();
                        };
                        closeTimer.Start();
                    }
                }
            }
            catch (Exception ex)
            {
                this.StatusText = "Error generating model";
                this.modelGenerationInProgress = false;

                // Update measurement display to show error
                this.MeasurementText.Text = "✗ ERROR\n\n" +
                    "Failed to generate model.\n\n" +
                    $"Error: {ex.Message}\n\n" +
                    "Check console for details.\n\n" +
                    "Closing in 3 seconds...";

                // Log error to console
                Console.WriteLine("========================================");
                Console.WriteLine("[ERROR] Failed to generate model!");
                Console.WriteLine($"  Error: {ex.Message}");
                Console.WriteLine($"  Stack trace: {ex.StackTrace}");
                Console.WriteLine("========================================");

                // Close the window after a short delay even on error
                var closeTimer = new System.Windows.Threading.DispatcherTimer();
                closeTimer.Interval = TimeSpan.FromSeconds(3);
                closeTimer.Tick += (s, e) =>
                {
                    closeTimer.Stop();
                    Console.WriteLine("[INFO] Closing application after error...");
                    this.Close();
                };
                closeTimer.Start();
            }
            finally
            {
                this.modelGenerationInProgress = false;
            }
        }
    }
}

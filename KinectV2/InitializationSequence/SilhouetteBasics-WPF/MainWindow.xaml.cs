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
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;

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
        /// </summary>
        private void ProcessBodyIndexData()
        {
            if (this.bodyIndexData == null || this.silhouetteBitmap == null)
            {
                return;
            }

            // Convert body index data to RGBA pixels
            // White = body (255, 255, 255, 255), Black = background (0, 0, 0, 255)
            for (int i = 0; i < this.bodyIndexData.Length; i++)
            {
                int pixelIndex = i * 4; // RGBA = 4 bytes per pixel

                if (this.bodyIndexData[i] != 255)
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
        /// </summary>
        private void UpdateMeasurements()
        {
            if (this.trackedBody == null || !this.trackedBody.IsTracked)
            {
                this.MeasurementText.Text = "No body tracked";
                return;
            }

            if (this.bodyIndexData == null || this.depthData == null || this.coordinateMapper == null)
            {
                return;
            }

            // Get SpineMid joint for width measurement
            var spineMidJoint = this.trackedBody.Joints[JointType.SpineMid];

            if (spineMidJoint.TrackingState == TrackingState.NotTracked)
            {
                this.MeasurementText.Text = "Joint not tracked";
                return;
            }

            // Measure width at SpineMid
            double widthInMeters = this.MeasureWidthAtJoint(spineMidJoint);

            if (widthInMeters > 0)
            {
                // Display measurement
                this.MeasurementText.Text = string.Format(
                    CultureInfo.CurrentCulture,
                    "Body Width at SpineMid: {0:F3} meters ({1:F1} cm)",
                    widthInMeters,
                    widthInMeters * 100);
            }
            else
            {
                this.MeasurementText.Text = "Measurement unavailable";
            }
        }

        /// <summary>
        /// Measures the body width at a specific joint by scanning the silhouette horizontally.
        /// Uses CoordinateMapper to convert 3D joint position to 2D depth space coordinates.
        /// </summary>
        /// <param name="joint">The joint at which to measure width</param>
        /// <returns>Width in meters, or 0 if measurement failed</returns>
        private double MeasureWidthAtJoint(Joint joint)
        {
            if (joint.TrackingState == TrackingState.NotTracked)
            {
                return 0;
            }

            if (this.bodyIndexData == null || this.depthData == null || this.coordinateMapper == null)
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
                if (this.bodyIndexData[index] == 255) // Not a body pixel
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
                if (this.bodyIndexData[index] == 255) // Not a body pixel
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

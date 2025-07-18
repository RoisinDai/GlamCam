namespace Microsoft.Samples.Kinect.ColorBasics
{
    using Microsoft.Kinect;
    using System;
    using System.ComponentModel;
    using System.Globalization;
    using System.IO;
    using System.Net.Sockets;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Reader for color frames
        /// </summary>
        private ColorFrameReader colorFrameReader = null;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap colorBitmap = null;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// TCP used to send color frames
        /// </summary>
        private TcpClient client = null;

        /// <summary>
        /// Network stream for sending data
        /// </summary>
        private NetworkStream stream = null;

        /// <summary>
        /// TCP host and port config for sending data
        /// </summary>
        private int port = 5005;
        private String host = "127.0.0.1";

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            // open the reader for the color frames
            this.colorFrameReader = this.kinectSensor.ColorFrameSource.OpenReader();

            // wire handler for frame arrival
            this.colorFrameReader.FrameArrived += this.ColorFrameArrived;

            // create the colorFrameDescription from the ColorFrameSource using Bgra format
            FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);

            // create the bitmap to display
            this.colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

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

            // If client successfully connected, obtain it's stream
            if (this.client != null)
            {
                this.stream = client.GetStream();
            }

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
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
                return this.colorBitmap;
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

                    // notify any bound elements that the text has changed
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
            if (this.colorFrameReader != null)
            {
                // ColorFrameReder is IDisposable
                this.colorFrameReader.Dispose();
                this.colorFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }

            if (this.kinectSensor != null)
            {
                this.stream.Close();
                this.client.Close();
            }
        }

        /// <summary>
        /// Process each received colored frame from Kinect.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ColorFrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            using (ColorFrame colorFrame = e.FrameReference.AcquireFrame())
            {
                if (colorFrame == null)
                    return;

                FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                // ----------- 1. Render to the window -------------
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

                // ----------- 2. Send frame to Python asynchronously -------------
                if (this.stream != null && this.stream.CanWrite)
                {
                    try
                    {
                        // Create a WriteableBitmap for JPEG encoding
                        var tempBitmap = new WriteableBitmap(
                            colorFrameDescription.Width,
                            colorFrameDescription.Height,
                            96.0, 96.0, System.Windows.Media.PixelFormats.Bgra32, null);

                        // Proper usage: Lock before writing to BackBuffer
                        tempBitmap.Lock();

                        colorFrame.CopyConvertedFrameDataToIntPtr(
                            tempBitmap.BackBuffer,
                            (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4),
                            ColorImageFormat.Bgra);

                        // Mark the bitmap as dirty (entire area)
                        tempBitmap.AddDirtyRect(new Int32Rect(0, 0, tempBitmap.PixelWidth, tempBitmap.PixelHeight));

                        tempBitmap.Unlock();

                        // Encode to JPEG
                        byte[] jpgBytes;
                        var encoder = new JpegBitmapEncoder();
                        encoder.Frames.Add(BitmapFrame.Create(tempBitmap));
                        using (var ms = new MemoryStream())
                        {
                            encoder.Save(ms);
                            jpgBytes = ms.ToArray();
                        }

                        byte[] lenBytes = BitConverter.GetBytes(jpgBytes.Length);

                        //Send
                        try
                        {
                            lock (this.stream) // thread safety
                            {
                                this.stream.Write(lenBytes, 0, 4);
                                this.stream.Write(jpgBytes, 0, jpgBytes.Length);
                            }
                            System.Diagnostics.Debug.WriteLine($"Frame sent: {jpgBytes.Length} bytes");
                        }
                        catch (Exception ex)
                        {
                            System.Diagnostics.Debug.WriteLine("Error sending frame: " + ex.ToString());
                        }
                    }
                    catch (Exception ex)
                    {
                        System.Diagnostics.Debug.WriteLine("Frame encoding error: " + ex.ToString());
                    }
                }
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }
    }
}

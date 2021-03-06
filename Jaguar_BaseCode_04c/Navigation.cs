using System.Linq;
using System.Text;


using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Threading;
using System.IO;
//  find color
using System.Data;
using System.Drawing;
//using System.Linq;
//using System.Text;
//using System.Threading.Tasks;
using System.Windows.Forms;
using WindowsFormsApplication3;
using System.Runtime.InteropServices;
using Microsoft.DirectX.DirectInput;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.UI;
using Emgu.CV.GPU;
//not sure we use this
using Emgu.Util;
using Emgu.CV.ML;
using System.Diagnostics;

namespace DrRobot.JaguarControl
{
    public class Navigation
    {
        #region Navigation Variables
        public long[] LaserData = new long[DrRobot.JaguarControl.JaguarCtrl.DISDATALEN];
        public double initialX = 0;//-3;
        public double initialY = 0;//5.5;
        public double initialT = 0;
        public double x, y, t;
        public double x_est, y_est, t_est;
        public double desiredX, desiredY, desiredT;
        public double x_prev, y_prev, t_prev;

        public Boolean withinEpsilon = false;
        public long lastUpdate = 0;

        public double currentEncoderPulseL, currentEncoderPulseR;
        public double lastEncoderPulseL, lastEncoderPulseR;
        public double wheelDistanceR, wheelDistanceL;
        public double tiltAngle, zoom;
        public double currentAccel_x, currentAccel_y, currentAccel_z;
        public double lastAccel_x, lastAccel_y, lastAccel_z;
        public double currentGyro_x, currentGyro_y, currentGyro_z;
        public double last_v_x, last_v_y;
        public double filteredAcc_x, filteredAcc_y;

        public int robotType, controllerType;
        enum ROBOT_TYPE { SIMULATED, REAL };
        enum CONTROLLERTYPE { MANUALCONTROL, POINTTRACKER, EXPERIMENT };
        public bool motionPlanRequired, displayParticles, displayNodes, displaySimRobot;
        private JaguarCtrl jaguarControl;
        private AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel realJaguar;
        private AxDDrRobotSentinel_Simulator simulatedJaguar;
        private Thread controlThread;
        private short motorSignalL, motorSignalR;
        private short desiredRotRateR, desiredRotRateL;
        public bool runThread = true;
        public bool loggingOn;
        StreamWriter logFile;
        public int deltaT = 10;
        private static int encoderMax = 32767;
        public int pulsesPerRotation = 190;
        public double wheelRadius = 0.089;
        public double robotRadius = 0.242;//0.232
        private double angleTravelled, distanceTravelled;
        private double diffEncoderPulseL, diffEncoderPulseR;
        private double maxVelocity = 0.23;//0.21;//0.21;//0.23;//0.21;//0.25;
        private double Kpho = 1;
        private double Kalpha = 8;//4;//4;//8
        private double Kbeta = -0.5;//-0.5//-1.0;
        const double alphaTrackingAccuracy = 0.10;
        const double betaTrackingAccuracy = 0.1;
        const double phoTrackingAccuracy = 0.10;
        double time = 0;
        DateTime startTime;

        private string currentLog = "realData";
        private string streamPath_;

        public short K_P = 15;//15;
        public short K_I = 0;//0;
        public short K_D = 3;//3;
        public short frictionComp = 8750;//8750;
        public double e_sum_R, e_sum_L;
        public double u_R = 0;
        public double u_L = 0;
        public double e_R = 0;
        public double e_L = 0;
        public double e_L_last = 0;
        public double e_R_last = 0;

        public double rotRateL, rotRateR;
        public double K_p, K_i, K_d, maxErr;

        public double accCalib_x = 18;
        public double accCalib_y = 4;

        // PF Variables
        public Map map;
        public Particle[] particles;
        public Particle[] propagatedParticles;
        public int numParticles = 3000;
        public double K_wheelRandomness = 0.15;//0.25
        public Random random = new Random();
        public bool newLaserData = false;
        public double laserMaxRange = 4.0;
        public double laserMinRange = 0.2;
        public double[] laserAngles;
        private int laserCounter;
        public int laserStepSize = 3;
        public Boolean newMovement = false;

        public double PFLastEncoderL = 0;
        public double PFLastEncoderR = 0;
        public double PFEncoderDiffL = 0;
        public double PFEncoderDiffR = 0;

        public class Particle
        {
            public double x, y, t, w;

            public Particle()
            {
            }
        }

        #endregion

 #region FindColor Variables
        //The form size handelers need to be changed.
        bool firstTimeResize = true;
        int oFormWidth;
        int oFormHeight;
        int oTableLayoutPaneWidth;
        int oTableLayoutPaneHeight;
        Image<Bgr, Byte> imgOrigional;
        Image<Gray, Byte> imgSmoothed;
        Image<Gray, Byte> imgGrayFiltered;
        Image<Gray, Byte> imgCanny;
        Image<Bgr, Byte> imgCircles;
        Image<Bgr, Byte> imgLines;
        Image<Bgr, Byte> imgTrisRecPoly;
        int hueMinVal = 0;
        int hueMaxVal = 180;
        int satMinVal = 0;
        int satMaxVal = 255;

        public AxAXISMEDIACONTROLLib.AxAxisMediaControl AMC;
        private Emgu.CV.UI.ImageBox Gray;

        bool robotCamOn = false;
        //camera handlers
        bool jaguarCamCapture = false;
        bool fromFile = true;
        #region robotCam
        // using vars here instead of the xml that the other codebase uses
        string robotName = "DrRobot";

        string cameraIP = "192.168.0.65";
        string cameraPort = "8081";
        string cameraUname = "root";
        string cameraPass = "drrobot";
        private Capture _capture;
        private bool streaming;
        private static Process p1 = null;
        // vars for robot control non existent
        //slider variables


        #endregion

        #region Navigation Setup
        
        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            map = new Map(this);
            particles = new Particle[numParticles];
            propagatedParticles = new Particle[numParticles];
            // Create particles
            for (int i = 0; i < numParticles; i++)
            {
                particles[i] = new Particle();
                propagatedParticles[i] = new Particle();
            }

            this.Initialize();


            // Start Control Thread
            controlThread = new Thread(new ThreadStart(runControlLoop));
            controlThread.Start();
        }

        // All class variables are initialized here
        // This is called every time the reset button is pressed
        public void Initialize()
        {
            initialX = map.xOffset + 11;// 3;// 11;
            initialY = map.yOffset - 41;// 4; //41;
            initialT = 0;// -3.14;// 0

            map.currentRegion = 0;
            currentWaypoint = 0;
            
            waypoints = new double[origwaypoints.Length];
            for (int i = 0; i < waypoints.Length; i += 3)
            {
                waypoints[i] = origwaypoints[i] + map.xOffset;
                waypoints[i + 1] = origwaypoints[i + 1] + map.yOffset;
                waypoints[i + 2] = origwaypoints[i + 2];
            }

            // Initialize state estimates
            x = initialX;
            y = initialY;
            t = initialT;

            // Initialize state estimates
            x_est = initialX;
            y_est = initialY;
            t_est = initialT;

            x_prev = x;
            y_prev = y;
            t_prev = t;

            // Set desired state
            desiredX = 0;// initialX;
            desiredY = 0;// initialY;
            desiredT = 0;// initialT;

            // Reset Localization Variables
            wheelDistanceR = 0;
            wheelDistanceL = 0;

            // Zero actuator signals
            motorSignalL = 0;
            motorSignalR = 0;
            loggingOn = false;

            // Set random start for particles
            InitializeParticles();

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;

            laserAngles = new double[LaserData.Length];
            for (int i = 0; i < LaserData.Length; i++)                
                laserAngles[i] = DrRobot.JaguarControl.JaguarCtrl.startAng + DrRobot.JaguarControl.JaguarCtrl.stepAng * i;

        }

        // This function is called from the dialogue window "Reset Button"
        // click function. It resets all variables.
        public void Reset()
        {
            simulatedJaguar.Reset();
            GetFirstEncoderMeasurements();
            CalibrateIMU();
            Initialize();
        }
        #endregion


        #region Main Loop

        /************************ MAIN CONTROL LOOP ***********************/
        // This is the main control function called from the control loop
        // in the RoboticsLabDlg application. This is called at every time
        // step.
        // Students should choose what type of localization and control 
        // method to use. 
        public void runControlLoop()
        {
            // Wait
            Thread.Sleep(500);

            // Don't run until we have gotten our first encoder measurements to difference with
            GetFirstEncoderMeasurements();

            // Run infinite Control Loop
            while (runThread)
            {
            	//Try to process an image from the robot's camera
            	 try
                {
                    //PictureBox pb1 = new PictureBox();
                    //pb1.ImageLocation = "http://www.dotnetperls.com/favicon.ico";
                    //pb1.SizeMode = PictureBoxSizeMode.AutoSize;
                    //this.Controls.Add(pb1);
                    
                    //AMC.MediaUsername = cameraUname;
                    //AMC.MediaPassword = cameraPass;
                    //AMC.MediaType = "mjpeg";
                    //AMC.MediaURL = "http://" + cameraIP + ":" + cameraPort + "/axis-cgi/mjpg/video.cgi";
                    //Console.WriteLine(AMC.MediaURL);
                    //AMC.Play();

                   
                    object buffer;
                    int bufferSize;
                    jaguarControl.myAMC.GetCurrentImage(0, out buffer, out bufferSize);
                    Byte[] buf = new Byte[bufferSize];
                    Array.Copy((Array)buffer, (Array)buf, bufferSize);
                    Bitmap bmp;
                    using (MemoryStream stream = new MemoryStream(buf))
                        bmp = Bitmap.FromStream(stream) as Bitmap;
                    Image<Bgr, Byte> image = new Image<Bgr, Byte>(bmp);
                    imgOrigional = image;
                    ProcessFrame(image);
                    //Console.WriteLine("image yes");
                }

                catch {// Console.WriteLine("image no"); }
                // ****************** Additional Student Code: Start ************

                // Students can select what type of localization and control
                // functions to call here. For lab 1, we just call the function
                // WallPositioning to have the robot maintain a constant distance
                // to the wall (see lab manual).
                
                // Update Sensor Readings
                UpdateSensorMeasurements();

                // Determine the change of robot position, orientation (lab 2)	
                MotionPrediction();

                // Update the global state of the robot - x,y,t (lab 2)
                LocalizeRealWithOdometry();

                // Update the global state of the robot - x,y,t (lab 2)
                //LocalizeRealWithIMU();
                

                // Estimate the global state of the robot -x_est, y_est, t_est (lab 4)
                //if (newLaserData && (Math.Abs(diffEncoderPulseL) > 0 || Math.Abs(diffEncoderPulseR) > 0))
                //{
                    LocalizeEstWithParticleFilter();
                //}


                // If using the point tracker, call the function
                if (jaguarControl.controlMode == jaguarControl.AUTONOMOUS)
                {

                    // Check if we need to create a new trajectory
                    if (motionPlanRequired)
                    {
                        // Construct a new trajectory (lab 5)
                        PRMMotionPlanner();
                        motionPlanRequired = false;
                    }

                    // Drive the robot to a desired Point (lab 3)
                    //FlyToSetPoint();

                    // Follow the trajectory instead of a desired point (lab 3)
                    TrackTrajectory();

                    // Actuate motors based actuateMotorL and actuateMotorR
                    if (jaguarControl.Simulating())
                    {
                        CalcSimulatedMotorSignals();
                        ActuateMotorsWithVelControl();
                    }
                    else 
                    {
                        // Determine the desired PWM signals for desired wheel speeds
                        CalcMotorSignals();
                        ActuateMotorsWithPWMControl();
                    }

                }
                else
                {
                    e_sum_L = 0;
                    e_sum_R = 0;
                }
                
                // ****************** Additional Student Code: End   ************

                // Log data
                LogData();

                // Sleep to approximate 20 Hz update rate
                Thread.Sleep(deltaT); //not sure if this works anymore..... -wf
            }
        }


        public void CalibrateIMU()
        {

            accCalib_x = 0;
            accCalib_y = 0;
            int numMeasurements = 100;
            for (int i = 0; i < numMeasurements; i++)
            {
                accCalib_x += currentAccel_x;
                accCalib_y += currentAccel_y;

                Thread.Sleep(deltaT);
            }
            accCalib_x = accCalib_x / numMeasurements;
            accCalib_y = accCalib_y /numMeasurements;


        }


        // Before starting the control loop, the code checks to see if 
        // the robot needs to get the first encoder measurements
        public void GetFirstEncoderMeasurements()
        {
            if (!jaguarControl.Simulating())
            {
                // Get last encoder measurements
                bool gotFirstEncoder = false;
                int counter = 0;
                while (!gotFirstEncoder && counter < 10)
                {
                    try
                    {
                        currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                        currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();
                        lastEncoderPulseL = currentEncoderPulseL;
                        lastEncoderPulseR = currentEncoderPulseR;
                        PFLastEncoderL = currentEncoderPulseL;
                        PFLastEncoderR = currentEncoderPulseR;
                        gotFirstEncoder = true;

                        currentAccel_x = jaguarControl.getAccel_x();
                        currentAccel_y = jaguarControl.getAccel_y();
                        currentAccel_z = jaguarControl.getAccel_z();
                        lastAccel_x = currentAccel_x;
                        lastAccel_y = currentAccel_y;
                        lastAccel_z = currentAccel_z;
                        last_v_x = 0;
                        last_v_y = 0;

                    }
                    catch (Exception e) { }
                    counter++;
                    Thread.Sleep(100);
                }
            }
            else
            {
                currentEncoderPulseL = 0;
                currentEncoderPulseR = 0;
                lastEncoderPulseL = 0;
                lastEncoderPulseR = 0;
                lastAccel_x = 0;
                lastAccel_y = 0;
                lastAccel_z = 0;
                last_v_x = 0;
                last_v_y = 0;

            }
        }

        // At every iteration of the control loop, this function will make 
        // sure all the sensor measurements are up to date before
        // makeing control decisions.
        public void UpdateSensorMeasurements()
        {
            // For simulations, update the simulated measurements
            if (jaguarControl.Simulating())
            {
                jaguarControl.simulatedJaguar.UpdateSensors(deltaT);

                // Get most recent encoder measurements
                currentEncoderPulseL = simulatedJaguar.GetEncoderPulse4();
                currentEncoderPulseR = simulatedJaguar.GetEncoderPulse5();

                // Get most recent laser scanner measurements
                laserCounter = laserCounter + deltaT;
                if (laserCounter >= 1500)
                {
                    for (int i = 0; i < LaserData.Length; i=i+laserStepSize)
                    {
                        LaserData[i] = (long)(1000 * map.GetClosestWallDistance(x, y, t -1.57 + laserAngles[i]));
                    }
                    laserCounter = 0;
                    newLaserData = true;
                }
            }
            else
            {
                // Get most recenct encoder measurements
                try
                {

                    // Update IMU Measurements
                    currentAccel_x = jaguarControl.getAccel_x();
                    currentAccel_y = jaguarControl.getAccel_y();
                    currentAccel_z = jaguarControl.getAccel_z();
                   
                    // Update Encoder Measurements
                    currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                    currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();

                }
                catch (Exception e)
                {
                }
            }
        }

        // At every iteration of the control loop, this function calculates
        // the PWM signal for corresponding desired wheel speeds
        public void CalcSimulatedMotorSignals()
        {

            motorSignalL = (short)(desiredRotRateL);
            motorSignalR = (short)(desiredRotRateR);

        }

        public void CalcMotorSignals()
        {
            if (lastUpdate == 0)
            {
                lastUpdate = DateTime.Now.Ticks - 100000;
            }
            int timeDiff = (int)((DateTime.Now.Ticks - lastUpdate) / 10000);

            short zeroOutput = 16383;
            short maxPosOutput = 32767;

            double K_u = 150;// 145;// 163;// 140;
            double T_u = 15;// 29;// 8;
            double K_p = 0.6 /*0.625*/ * K_u;// 25;
            double K_i = 2 * K_p / T_u;// 0.1;
            double K_d = K_p * T_u / 8;// 1;

            double maxErr = 8000 / timeDiff;


            e_L = desiredRotRateL - diffEncoderPulseL / timeDiff;
            e_R = desiredRotRateR - diffEncoderPulseR / timeDiff;

            e_sum_L = .9 * e_sum_L + e_L * timeDiff;
            e_sum_R = .9 * e_sum_R + e_R * timeDiff;

            e_sum_L = Math.Max(-maxErr, Math.Min(e_sum_L, maxErr));
            e_sum_R = Math.Max(-maxErr, Math.Min(e_sum_R, maxErr));

            u_L = ((K_p * e_L) + (K_i * e_sum_L) + (K_d * (e_L - e_L_last) / timeDiff));
            e_L_last = e_L;

            u_R = ((K_p * e_R) + (K_i * e_sum_R) + (K_d * (e_R - e_R_last) / timeDiff));
            e_R_last = e_R;
            // The following settings are used to help develop the controller in simulation.
            // They will be replaced when the actual jaguar is used.
            //motorSignalL = (short)(zeroOutput + desiredRotRateL * 100);// (zeroOutput + u_L);
            //motorSignalR = (short)(zeroOutput - desiredRotRateR * 100);//(zeroOutput - u_R);

            motorSignalL = (short)(zeroOutput + u_L);
            motorSignalR = (short)(zeroOutput - u_R);

            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalR));

            lastUpdate = DateTime.Now.Ticks;
        }

        // At every iteration of the control loop, this function sends
        // the width of a pulse for PWM control to the robot motors
        public void ActuateMotorsWithPWMControl()
        { 
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            else
            {
                jaguarControl.realJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            }
        }

        // At every iteration of the control loop, this function sends
        // desired wheel velocities (in pulses / second) to the robot motors
        public void ActuateMotorsWithVelControl()
        {
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            else
                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
        }
        #endregion


        #region Logging Functions

        private int logNum = 1;

        // This function is called from a dialogue window "Record" button
        // It creates a new file and sets the logging On flag to true
        public void TurnLoggingOn()
        {
            //int fileCnt= 0;
            String date = DateTime.Now.Year.ToString() + "-" + DateTime.Now.Month.ToString() + "-" + DateTime.Now.Day.ToString() + "-" + DateTime.Now.Minute.ToString();
            ToString();
            streamPath_ = "JaguarData_" + date + ".csv";
            logFile = File.CreateText(streamPath_);
            string header = "iteration, x, y, t";
            Console.WriteLine("Writing to " + streamPath_);
            // uncomment if we want a header
            //logFile.WriteLine(header);
            logFile.Close();
            startTime = DateTime.Now;
            loggingOn = true;
        }

        // This function is called from a dialogue window "Record" button
        // It closes the log file and sets the logging On flag to false
        public void TurnLoggingOff()
        {
            if (logFile != null)
                logFile.Close();
            loggingOn = false;
            ++logNum;
        }

        // This function is called at every iteration of the control loop
        // IF the loggingOn flag is set to true, the function checks how long the 
        // logging has been running and records this time
        int iteration = 0;
        private void LogData()
        {
            if (loggingOn)
            {
                TimeSpan ts = DateTime.Now - startTime;
                time = ts.TotalSeconds;
                //double distanceFromWall = LaserData[113];
                //String newData = time.ToString() + ", " + x.ToString() + ", " + y.ToString() + ", " + t.ToString();                
                String newData = "";
                //if (iteration % 5 == 0) {
                    
                    //for (int i = 0; i < numParticles; ++i)
                    //{
                    //    newData += iteration.ToString() + ", " + particles[i].x.ToString() + ", " + particles[i].y.ToString() + ", " + particles[i].t.ToString() + "\n";
                    //}
                //}
                //iteration++;
                newData += x.ToString() + ", " + y.ToString() + ", " + x_est.ToString() + ", " + y_est.ToString();

                logFile = File.AppendText(streamPath_);
                logFile.WriteLine(newData);
                logFile.Close();
            }
        }
        #endregion


        # region Control Functions
//This function processes a given image. It finds the largest object with a given Hue and Saturation
//and creates a Bounding Rectangle around it. The robot can localize based on the size and location 
//of this Bounding Rectangle.
public void ProcessFrame(Image<Bgr, Byte> frame)
        {
            Image<Hsv, Byte> hsvframe = frame.Convert<Hsv, Byte>();
            Image<Gray, Byte> hueframe = hsvframe.Split()[0];
            Image<Gray, Byte> satframe = hsvframe.Split()[1];

            Gray huethreshl = new Gray(hueMinVal);
            Gray huethreshu = new Gray(hueMaxVal);

            if (hueMinVal > hueMaxVal)
            {
                Image<Gray, Byte> hueframelow = hueframe.InRange(new Gray(0), huethreshu);
                Image<Gray, Byte> hueframehigh = hueframe.InRange(huethreshl, new Gray(180));
                hueframe = hueframelow.Or(hueframehigh);
            }
            else
            {
                hueframe = hueframe.InRange(huethreshl, huethreshu);
            }

            Gray.Image = hueframe;
            Gray satthreshl = new Gray(satMinVal);
            Gray satthreshu = new Gray(satMaxVal);
            satframe = satframe.InRange(satthreshl, satthreshu);


            Image<Gray, Byte> blobframe = satframe.And(hueframe);
            imgGrayFiltered = blobframe;

            //blobframe = blobframe.Erode(2);
            //blobframe = blobframe.Dilate(1);

            // imgGrayFiltered = frame.Convert<Gray, Byte>();
            imgSmoothed = imgGrayFiltered.PyrDown().PyrUp();
            imgSmoothed._SmoothGaussian(3);

            //about filteroncolor. do we implement it? or is it built in

            //morevalriables
            double dbgraycannythres = 160.0;
            double dbgraycirclethres = 100.0;
            double dbgraythreslinking = 80.0;
            Gray graycannythres = new Gray(160);
            Gray graycirclethres = new Gray(100);
            Gray graythreslinking = new Gray(80);

            imgCanny = imgSmoothed.Canny(dbgraycirclethres, dbgraythreslinking);

            #region polygons
            Contour<Point> contours = imgGrayFiltered.FindContours(); //find sequence of contours
            List<Triangle2DF> fsttriangles = new List<Triangle2DF>();
            List<MCvBox2D> fstrectangles = new List<MCvBox2D>();
            List<Contour<Point>> fstpolygons = new List<Contour<Point>>();
            //provided detection/////////////////////////////////////////////////
            double largestArea = 0;
            Contour<Point> largestContour = contours;
            while (contours != null)
            {
                double nextArea = contours.Area;
                if (nextArea > largestArea)
                {
                    largestArea = nextArea;
                    largestContour = contours;
                }
                contours = contours.HNext;
            }
            if (largestContour != null)
            {
                Rectangle boundingRect = largestContour.BoundingRectangle;
                imgTrisRecPoly = imgOrigional;
                imgTrisRecPoly.Draw(boundingRect, new Bgr(Color.Yellow), 1);
                imgTrisRecPoly.Draw(largestContour, new Bgr(Color.Green), 1);

                System.Drawing.Size size = boundingRect.Size;
                Console.WriteLine("the size is " + size);
            }


            #endregion
            //assign variables
            
            //Original.Image = imgOrigional;
            Gray.Image = imgGrayFiltered;
           // Canny.Image = imgCanny;
           // Circles.Image = imgCircles;
           // Lines.Image = imgLines;
           // RectPolys.Image = imgTrisRecPoly;
             
        }

        // This function is called at every iteration of the control loop
        // It will drive the robot forward or backward to position the robot 
        // 1 meter from the wall.
        private void WallPositioning()
        {

            // Here is the distance measurement for the central laser beam 
            double centralLaserRange = LaserData[113];

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            // Send Control signals, put negative on left wheel control

 

            // ****************** Additional Student Code: End   ************                
        }


        // This function is called at every iteration of the control loop
        // if used, this function can drive the robot to any desired
        // robot state. It does not check for collisions
        private void FlyToSetPoint()
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate desiredRotRateR and 
            // desoredRotRateL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!
            double deltaX = desiredX - x_est;
            double deltaY = desiredY - y_est;
            double pho = Math.Sqrt(Math.Pow(deltaX, 2) + Math.Pow(deltaY, 2));
            double alpha = AngleDiff(t_est, Math.Atan2(deltaY, deltaX));
            int isBackwards = 1;
            // Check if its optimal to travel backwards!
            /*if (Math.Abs(alpha) > Math.PI / 2)
            {
                alpha = AngleDiff(t_est, Math.Atan2(-deltaY, -deltaX));
                isBackwards = -1;
            }*/

            double beta = AngleDiff(t_est, -alpha);
            // adding desired T
            beta = AngleDiff(-desiredT, beta);

            double desiredV = isBackwards * Kpho * pho;
            double desiredW = Kalpha * alpha + Kbeta * beta;

            double omega2 = (desiredW - (desiredV / robotRadius)) / 2;
            double omega1 = desiredW - omega2;
            double desiredVelR = omega1 * 2 * robotRadius / wheelRadius;
            double desiredVelL = omega2 * 2 * robotRadius / wheelRadius;
            double maxRadVel = maxVelocity / wheelRadius;
            if (Math.Abs(desiredVelR) > maxRadVel)
            {
                desiredVelL = maxRadVel * desiredVelL / Math.Abs(desiredVelR);
                desiredVelR = (desiredVelR < 0) ? -maxRadVel : maxRadVel;
            }
            if (Math.Abs(desiredVelL) > maxRadVel)
            {
                desiredVelR = maxRadVel * desiredVelR / Math.Abs(desiredVelL);
                desiredVelL = (desiredVelL < 0) ? -maxRadVel : maxRadVel;
            }

            desiredRotRateL = (short)-(desiredVelL / (2 * Math.PI) * pulsesPerRotation);
            desiredRotRateR = (short)(desiredVelR / (2 * Math.PI) * pulsesPerRotation);

            if (Math.Abs(pho) < 0.5)//.1)
            {
                withinEpsilon = true;
            }

            if (withinEpsilon)
            {
                double thetaError = AngleDiff(desiredT, t_est);
                double epsilon = 0.2;// 0.175;
                short spinSpeed = (short)(/*60*//*50*/ /*37*/ 60 + Math.Abs(thetaError) * 20/*15*/ / Math.PI);

                if (thetaError > 0 && Math.Abs(thetaError) > epsilon)
                {
                    // turn right
                    desiredRotRateL = (short)spinSpeed;
                    desiredRotRateR = (short)(-spinSpeed);
                }
                else if (thetaError < 0 && Math.Abs(thetaError) > epsilon)
                {
                    // turn left
                    desiredRotRateL = (short)(-spinSpeed);
                    desiredRotRateR = (short)spinSpeed;
                }
                else
                {
                    desiredRotRateL = 0;
                    desiredRotRateR = 0;
                }
            }

            if (desiredRotRateL == 0 && desiredRotRateR == 0 && withinEpsilon)
            {
                withinEpsilon = false;
            }
            // ****************** Additional Student Code: End   ************
        }

        private double AngleDiff(double angle1, double angle2)
        {
            double difference = angle2 - angle1;
            while (difference < -Math.PI) difference += Math.PI * 2;
            while (difference > Math.PI) difference -= Math.PI * 2;
            return difference;
        }


        //private double[] waypoints = { 2, 1, 1, 3, 2, 0, 4, 1, -1, 5, 0, 3.14, 0, 0, 3.14 };
        // for lab 4 test
        //private double[] waypoints = {0.5, -0.5, -1.4, 1, -4, -1, 4, -4.5, 0 };
        private double[] origwaypoints = {
        /*0, -4, -3.14, //regionStart
        -3.9, -4, -3.14, //regionStart
        -3.9, -4, -1.57, //regionStart2
        -3.9, -11.5, -1.57, //regionStart2
        -3.9, -11.5, 0, //region1
        0, -11.5, -0.4, 
        0, -13.5, -1.5, 
        0.25, -15.5, -1.6, 
        1.75, -17.5, -1.5, 
        1.75, -19.5, -1.5, // waypoint 10 for counting
        1.75, -21.5, -1.5, 
        2, -23.5, -1.7,
        3, -27.5, -1.57,
        4, -31, -3.14, // wp3
        0.5, -31, -2.7,
        0.5, -31.5, -1.57,
        0.5, -33, -1.57,
        0.5, -35, -1.57, //wp4
        -1, -41, -1.57, // midpoint veering right
        0.5, -48.5, -1.57, //wp5
        0.5, -48.5, 1.57, //turn around
        0.5, -44.5, 1.57, //get out
        0.5, -44.5, 0,
        6, -44.5, 0,
        11, -41, 0, */// 25 for counting. Comment out here for partway
        13, -41, 0, //wp center right 6th milestone
        16.5, -41, 0,
        16.5, -41, 1.57, //turn up
        16.5, -39, 1.57,
        16.5, -35, 1.57, // 30 for counting
        16.5, -32, 1.57,
        16.5, -30, 1.57, 
        16.5, -28, 1.57, //final location outside of columns
        21.5, -27, 1.57,
        21.5, -24, 1.57,
        20, -21, 1.57, // before waypoint
        20, -19.25, 1.57, // AT REGION 7
        20, -19.25, -1.57, // turn around
        20, -24.5, -1.57, //backing out 
        20, -24.5, 0, //turn right dir
        23, -24.5, 0, //get out of area
        26, -24.5, 0,
        30, -24.5, 0, //intermediate sprint
        34, -24.5, 0,
        40, -24.5, 0, // SPRINT
        40, -24.5, 1.57, // TURN UP!
        40, -14.88, 1.57 //MADE IT
        };
        private double[] waypoints;
        private int currentWaypoint = 0;
        private Boolean reachedDest = false;
        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {

            desiredX = waypoints[currentWaypoint];
            desiredY = waypoints[currentWaypoint + 1];
            desiredT = waypoints[currentWaypoint + 2];
            double deltaX = desiredX - x_est;
            double deltaY = desiredY - y_est;
            double deltaT = AngleDiff(desiredT, t_est);
            double distToDest = Math.Sqrt(Math.Pow(deltaX, 2) + Math.Pow(deltaY, 2));
            if (distToDest < 0.6)
            {
                reachedDest = true;
            }
            //Console.WriteLine("current wp: " + currentWaypoint + " deltaT: " + deltaT + " distToDest: " + distToDest + " waypoints length: " + waypoints.Length);
            
            if (Math.Abs(deltaT) < 0.5 && reachedDest && currentWaypoint < waypoints.Length - 3)
            {
                reachedDest = false;
                currentWaypoint += 3;
                desiredX = waypoints[currentWaypoint];
                desiredY = waypoints[currentWaypoint + 1];
                desiredT = waypoints[currentWaypoint + 2];
                Console.WriteLine("currentwp: " + currentWaypoint + " going to X: " + desiredX + " Y: " + desiredY + " T: " + desiredT);
                if (map.currentRegion < map.regions.Length-1)
                {
                    ++map.currentRegion;
                }
                //Thread.Sleep(2000);
            }
            FlyToSetPoint();
            
        }

        // THis function is called to construct a collision-free trajectory for the robot to follow
        private void PRMMotionPlanner()
        {

        }


        #endregion


        #region Localization Functions
        /************************ LOCALIZATION ***********************/

        // This function will grab the most recent encoder measurements
        // from either the simulator or the robot (whichever is activated)
        // and use those measurements to predict the RELATIVE forward 
        // motion and rotation of the robot. These are referred to as
        // distanceTravelled and angleTravelled respectively.
        public void MotionPrediction()
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated distanceTravelled and angleTravelled.
            // You can set and use variables like diffEncoder1, currentEncoderPulse1,
            // wheelDistanceL, wheelRadius, encoderResolution etc. These are defined
            // in the Robot.h file.

            diffEncoderPulseL = currentEncoderPulseL - lastEncoderPulseL;
            diffEncoderPulseR = -(currentEncoderPulseR - lastEncoderPulseR);
            // check for rollover
            if (Math.Abs(diffEncoderPulseL) > 75 * pulsesPerRotation)
            {
                diffEncoderPulseL = diffEncoderPulseL < 0 ? diffEncoderPulseL + encoderMax : encoderMax - diffEncoderPulseL;
            }
            if (Math.Abs(diffEncoderPulseR) > 75 * pulsesPerRotation)
            {
                diffEncoderPulseR = diffEncoderPulseR < 0 ? diffEncoderPulseR + encoderMax : encoderMax - diffEncoderPulseR;
            }
            // update last encoder pulse
            lastEncoderPulseL = currentEncoderPulseL;
            lastEncoderPulseR = currentEncoderPulseR;
            // calculate wheel distance and change in distance and angle travelled
            wheelDistanceL = (2 * Math.PI * wheelRadius * diffEncoderPulseL) / pulsesPerRotation;
            wheelDistanceR = (2 * Math.PI * wheelRadius * diffEncoderPulseR) / pulsesPerRotation;
            angleTravelled = (wheelDistanceR - wheelDistanceL) / (2 * robotRadius);
            distanceTravelled = (wheelDistanceR + wheelDistanceL) / 2;      

            // ****************** Additional Student Code: End   ************
        }

        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithOdometry()
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi

            // Update the actual
            x += distanceTravelled * Math.Cos(t + (angleTravelled / 2));
            y += distanceTravelled * Math.Sin(t + (angleTravelled / 2));
            t += angleTravelled;
            // Make sure t stays between pi and -pi
            if (t > Math.PI)
            {
                t -= 2 * Math.PI;
            }
            if (t < -Math.PI)
            {
                t += 2 * Math.PI;
            }
            // ****************** Additional Student Code: End   ************
        }
        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithIMU()
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi


            // ****************** Additional Student Code: End   ************
        }



        public void LocalizeEstWithParticleFilter()
        {
            // To start, just set the estimated to be the actual for simulations
            // This will not be necessary when running the PF lab
       
            // ****************** Additional Student Code: Start ************
            
            if (Math.Abs(x - x_prev) > 0.00 || Math.Abs(y - y_prev) > 0.00 || Math.Abs(t - t_prev) > 0.00)
            {
                newMovement = true;
                x_prev = x;
                y_prev = y;
                t_prev = t;
            }

            // Put code here to calculate x_est, y_est, t_est using a PF
            double totalWeight = 0;




            for (int i = 0; i < numParticles; ++i)
            {
                PFEncoderDiffL = (currentEncoderPulseL - PFLastEncoderL);
                PFEncoderDiffR = -(currentEncoderPulseR - PFLastEncoderR);
                double tempDiffL = PFEncoderDiffL;
                double tempDiffR = PFEncoderDiffR;

                // check for rollover
                if (Math.Abs(PFEncoderDiffL) > 75 * pulsesPerRotation)
                {
                    PFEncoderDiffL = PFEncoderDiffL < 0 ? PFEncoderDiffL + encoderMax : encoderMax - PFEncoderDiffL;
                }
                if (Math.Abs(PFEncoderDiffR) > 75 * pulsesPerRotation)
                {
                    PFEncoderDiffR = PFEncoderDiffR < 0 ? PFEncoderDiffR + encoderMax : encoderMax - PFEncoderDiffR;
                }

                // calculate wheel distance and change in distance and angle travelled
                double PFDistanceL = (2 * Math.PI * wheelRadius * PFEncoderDiffL) / pulsesPerRotation;
                double PFDistanceR = (2 * Math.PI * wheelRadius * PFEncoderDiffR) / pulsesPerRotation;

                //double PFDistanceR = GaussianDist(wheelDistanceR, wheelDistanceR * 0.2);
                //double PFDistanceL = GaussianDist(wheelDistanceL, wheelDistanceL * 0.2);
                
                PFDistanceL = GaussianDist(PFDistanceL, PFDistanceL * 0.25);//0.15);//0.10);//0.2);
                PFDistanceR = GaussianDist(PFDistanceR, PFDistanceR * 0.25);//0.15);//0.10);//0.2);

                double estAngleTravelled = (PFDistanceR - PFDistanceL) / (2 * robotRadius);
                double estDistanceTravelled = (PFDistanceR + PFDistanceL) / 2;

                double partDeltaX = estDistanceTravelled * Math.Cos(particles[i].t + (estAngleTravelled / 2));
                double partDeltaY = estDistanceTravelled * Math.Sin(particles[i].t + (estAngleTravelled / 2));
                double partDeltaT = estAngleTravelled;
                propagatedParticles[i].x = particles[i].x + partDeltaX; 
                propagatedParticles[i].y = particles[i].y + partDeltaY;
                propagatedParticles[i].t = particles[i].t + partDeltaT;

                if (newLaserData && newMovement)
                {
                    CalculateWeight(i);
                    totalWeight += propagatedParticles[i].w;
                }     
            }


            // resample particles
            // first we make copies based on the ratio of current particle weight to total weight
            double xTotal = 0;
            double yTotal = 0;
            double tTotal = 0;
            if (newLaserData && newMovement)
            {
                List<int> tempParticles = new List<int>();
                double stepSize = 1.0 / (numParticles * 5.0);
                for (int i = 0; i < numParticles; ++i)
                {
                    double weightProp = propagatedParticles[i].w / totalWeight;
                    for (double j = 0; j <= 1.000; j += stepSize)
                    {
                        if (weightProp < j)
                        {
                            break;
                        }
                        tempParticles.Add(i);
                    }
                }

                // randomly choose the particles for the resampling
                // and then calculate State Estimate
                for (int i = 0; i < numParticles; ++i)
                {
                    int particleIndex = random.Next(0, tempParticles.Count);
                    particles[i].x = propagatedParticles[tempParticles[particleIndex]].x;
                    particles[i].y = propagatedParticles[tempParticles[particleIndex]].y;
                    particles[i].t = propagatedParticles[tempParticles[particleIndex]].t;
                    particles[i].w = propagatedParticles[tempParticles[particleIndex]].w;
                    xTotal += particles[i].x;
                    yTotal += particles[i].y;
                    tTotal += particles[i].t + Math.PI;
                }
                newLaserData = false;
                newMovement = false;
            }
            else
            {
                for (int i = 0; i < numParticles; ++i)
                {
                    particles[i].x = propagatedParticles[i].x;
                    particles[i].y = propagatedParticles[i].y;
                    particles[i].t = propagatedParticles[i].t;
                    particles[i].w = propagatedParticles[i].w;
                    xTotal += particles[i].x;
                    yTotal += particles[i].y;
                    tTotal += particles[i].t + Math.PI;
                }
            }
            x_est = xTotal / numParticles;
            y_est = yTotal / numParticles;
            t_est = tTotal / numParticles;
            t_est -= Math.PI;

            // update last encoder pulse
            PFLastEncoderL = currentEncoderPulseL;
            PFLastEncoderR = currentEncoderPulseR;
            // ****************** Additional Student Code: End   ************
        }

        // taken from http://stackoverflow.com/questions/218060/random-gaussian-variables
        public double GaussianDist(double mean, double stdDev)
        {
            double u1 = random.NextDouble(); //these are uniform(0,1) random doubles
            double u2 = random.NextDouble();
            double randStdNormal = Math.Sqrt(-2.0 * Math.Log(u1)) *
                         Math.Sin(2.0 * Math.PI * u2); //random normal(0,1)
            double randNormal =
                         mean + stdDev * randStdNormal; //random normal(mean,stdDev^2)

            return randNormal;
        }

        // Particle filters work by setting the weight associated with each
        // particle, according to the difference between the real robot 
        // range measurements and the predicted measurements associated 
        // with the particle.
        // This function should calculate the weight associated with particle p.
        void CalculateWeight(int p)
        {
	        // ****************** Additional Student Code: Start ************

	        // Put code here to calculated weight. Feel free to use the
	        // function map.GetClosestWallDistance from Map.cs.
            propagatedParticles[p].w = 0;
            double variance = 0.030; // meters
            for (int i = 0; i < LaserData.Length; i = i + laserStepSize)
            {
                double sampledRange = LaserData[i] / 1000.0;
                if (sampledRange < 0.05)
                {
                    //Console.WriteLine("TERRIBLE LASER DATA: " + sampledRange);
                    continue;
                }
                double expectedRange = map.GetClosestWallDistance(
                    propagatedParticles[p].x, 
                    propagatedParticles[p].y, 
                    propagatedParticles[p].t - 1.57 + laserAngles[i]);
                double weight = (1.0 / (Math.Sqrt(variance) * Math.Sqrt(2 * Math.PI))) *
                    Math.Pow(Math.E, (-Math.Pow(sampledRange - expectedRange, 2) / (2 * variance)));
                propagatedParticles[p].w += weight;
            }
        }



        // This function is used to initialize the particle states 
        // for particle filtering. It should pick a random location in the 
        // environment for each particle by calling SetRandomPos
        void InitializeParticles() {
	        // Set particles in random locations and orientations within environment
	        for (int i=0; i< numParticles; i++){

		        // Either set the particles at known start position [0 0 0],  
		        // or set particles at random locations.
                if (jaguarControl.startMode == jaguarControl.UNKNOWN)
    		        SetRandomPos(i);
                else if (jaguarControl.startMode == jaguarControl.KNOWN)
		            SetStartPos(i);
	        }         
        }



        // For particle p, this function will select a valid position. It should
        // select the position randomly, with equal likelihood of being anywhere 
        // in the environement. Should work for rectangular environments to make 
        // things easier.

        void SetRandomPos(int p){

	        // ****************** Additional Student Code: Start ************

	        // Put code here to calculated the position, orientation of 
            // particles[p]. Feel free to use the random.NextDouble() function. 
	        // It might be helpful to use boundaries defined in the
	        // Map.cs file (e.g. map.minX)
            particles[p].x = random.Next((int)map.minX, (int)map.maxX + 1) + random.NextDouble();
            particles[p].y = random.Next((int)map.minY, (int)map.maxY + 1) + random.NextDouble();
            particles[p].t = random.Next(-314159, 314159) / 100000.0;
            // ****************** Additional Student Code: End   ************
        }

        // For particle p, this function will select a start predefined position. 
        void SetStartPos(int p){
	        particles[p].x = initialX;
	        particles[p].y = initialY;
	        particles[p].t = initialT;
        }



        // Random number generator with gaussian distribution
        // Often random guassian numbers are used in particle filters. This
        // function might help.

        double RandomGaussian()
        {
	        double U1, U2, V1=0, V2;
	        double S = 2.0;
	        while(S >= 1.0) 
	        {
		        U1 = random.NextDouble();
                U2 = random.NextDouble();
		        V1 = 2.0*U1-1.0;
		        V2 = 2.0*U2-1.0;
		        S = Math.Pow(V1,2) + Math.Pow(V2,2);
	        }
	        double gauss = V1*Math.Sqrt((-2.0*Math.Log(S))/S);
	        return gauss;
        }



        // Get the sign of a number
        double Sgn(double a)
        {
	        if (a>0)
                return 1.0;
	        else if (a<0)
                return -1.0;
	        else
                return 0.0;
        }

        #endregion

    }
}

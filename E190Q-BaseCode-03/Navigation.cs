using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.IO;

namespace DrRobot.JaguarControl
{
    public class Navigation
    {
        #region Navigation Variables
        public long[] LaserData = new long[DrRobot.JaguarControl.JaguarCtrl.DISDATALEN];
        public double initialX, initialY, initialT;
        public double x, y, t;
        public double x_est, y_est, t_est;
        public double desiredX, desiredY, desiredT;

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
        private double maxVelocity = 0.25;
        private double Kpho = 1;
        private double Kalpha = 2; //8
        private double Kbeta = -0.5;//-0.5//-1.0;
        const double alphaTrackingAccuracy = 0.10;
        const double betaTrackingAccuracy = 0.1;
        const double phoTrackingAccuracy = 0.10;
        double time = 0;
        DateTime startTime;

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

        public double accCalib_x = 18;
        public double accCalib_y = 4;

        #endregion


        #region Navigation Setup
        
        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            this.Initialize();


            // Start Control Thread
            controlThread = new Thread(new ThreadStart(runControlLoop));
            controlThread.Start();
        }

        // All class variables are initialized here
        // This is called every time the reset button is pressed
        public void Initialize()
        {
            // Initialize state estimates
            x = 0;//initialX;
            y = 0;//initialY;
            t = 0;//initialT;

            // Initialize state estimates
            x_est = 0;//initialX;
            y_est = 0;//initialY;
            t_est = 0;//initialT;

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
            //InitializeParticles();

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;
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
                LocalizeEstWithParticleFilter();


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

                    // Drive the robot to 1meter from the wall. Otherwise, comment it out after lab 1. 
                    //WallPositioning();

                    // Drive the robot to a desired Point (lab 3)
                    FlyToSetPoint();

                    // Follow the trajectory instead of a desired point (lab 3)
                    // TrackTrajectory();

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

                // Get most recenct encoder measurements
                currentEncoderPulseL = simulatedJaguar.GetEncoderPulse4();
                currentEncoderPulseR = simulatedJaguar.GetEncoderPulse5();
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
            int timeDiff = (int)((DateTime.Now.Ticks - lastUpdate)/10000);

            short zeroOutput = 16383;
            short maxPosOutput = 32767;
            /*
            double K_u = 120;
            double T_u = 15;
            double K_p = 0.6 * K_u;// 25;
            double K_i = 2*K_p/T_u;// 0.1;
            double K_d = K_p*T_u/8;// 1;*/
            double K_p = 70;// 25;
            double K_i = 7;// 5;// 0.1;
            double K_d = 1;

            double maxErr = 8000 / timeDiff;


            e_L = desiredRotRateL - diffEncoderPulseL / timeDiff;
            e_R = desiredRotRateR - diffEncoderPulseR / timeDiff;

            e_sum_L = .9 * e_sum_L + e_L * deltaT;
            e_sum_R = .9 * e_sum_R + e_R * deltaT;

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
            {
                // Setup Control
                jaguarControl.realJaguar.SetDcMotorVelocityControlPID(3, K_P, K_D, K_I);
                jaguarControl.realJaguar.SetDcMotorVelocityControlPID(4, K_P, K_D, K_I);

                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            }
        }
        #endregion


        #region Logging Functions

        // This function is called from a dialogue window "Record" button
        // It creates a new file and sets the logging On flag to true
        public void TurnLoggingOn()
        {
            //int fileCnt= 0;
            String date = DateTime.Now.Year.ToString() + "-" + DateTime.Now.Month.ToString() + "-" + DateTime.Now.Day.ToString() + "-" + DateTime.Now.Minute.ToString();
            ToString();
            logFile = File.CreateText("JaguarData_" + date + ".txt");
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
        }

        // This function is called at every iteration of the control loop
        // IF the loggingOn flag is set to true, the function checks how long the 
        // logging has been running and records this time
        private void LogData()
        {
            if (loggingOn)
            {
                TimeSpan ts = DateTime.Now - startTime;
                time = ts.TotalSeconds;
                 String newData = time.ToString() + " " + x.ToString() + " " + y.ToString() + " " + t.ToString() ;

                logFile.WriteLine(newData);
            }
        }
        #endregion


        # region Control Functions

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

            // Put code here to calculate motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            double deltaX = desiredX - x_est;
            double deltaY = desiredY - y_est;
            double pho = Math.Sqrt(Math.Pow(deltaX, 2) + Math.Pow(deltaY, 2));
            double alpha = AngleDiff(t_est, Math.Atan2(deltaY, deltaX));
            int isBackwards = 1;
            // Check if its optimal to travel backwards!
            if (Math.Abs(alpha) > Math.PI/2) {
                alpha = AngleDiff(t_est, Math.Atan2(-deltaY, -deltaX));
                isBackwards = -1;
            }

            double beta = AngleDiff(t_est, -alpha);
            // adding desired T
            beta = AngleDiff(-desiredT, beta);

            double desiredV = isBackwards * Kpho * pho;
            double desiredW = Kalpha * alpha + Kbeta * beta;

            double omega2 = (desiredW  - (desiredV/robotRadius)) / 2;
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

            desiredRotRateL = (short)-(desiredVelL/(2*Math.PI)*pulsesPerRotation);
            desiredRotRateR = (short)(desiredVelR/(2*Math.PI)*pulsesPerRotation);

            if (Math.Abs(pho) < .1)
            {
                withinEpsilon = true;
            }

            if (withinEpsilon)
            {
                double thetaError = AngleDiff(desiredT, t_est);
                double epsilon = 0.75;

                if (thetaError > 0 && Math.Abs(thetaError) > epsilon)
                {
                    // turn right
                    desiredRotRateL = 100;
                    desiredRotRateR = -100;
                }
                else if (thetaError < 0 && Math.Abs(thetaError) > epsilon)
                {
                    // turn left
                    desiredRotRateL = -100;
                    desiredRotRateR = 100;
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
            while (difference < -Math.PI) difference += Math.PI*2;
            while (difference > Math.PI) difference -= Math.PI * 2;
            return difference;
        }



        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {
            desiredX = 1;
            desiredY = 0;
            desiredT = 0;
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
        public void MotionPrediction()//CWiRobotSDK* m_MOTSDK_rob)
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated distanceTravelled and angleTravelled.
            // You can set and use variables like diffEncoder1, currentEncoderPulse1,
            // wheelDistanceL, wheelRadius, encoderResolution etc. These are defined
            // in the Robot.h file.

            diffEncoderPulseL = currentEncoderPulseL - lastEncoderPulseL;
            diffEncoderPulseR = -(currentEncoderPulseR - lastEncoderPulseR);
            // check for rollover
            if (Math.Abs(diffEncoderPulseL) > pulsesPerRotation)
            {
                diffEncoderPulseL = diffEncoderPulseL < 0 ? diffEncoderPulseL + encoderMax : encoderMax - diffEncoderPulseL;
            }
            if (Math.Abs(diffEncoderPulseR) > pulsesPerRotation)
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
        public void LocalizeRealWithOdometry()//CWiRobotSDK* m_MOTSDK_rob)
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
        public void LocalizeRealWithIMU()//CWiRobotSDK* m_MOTSDK_rob)
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
            x_est = x;
            y_est = y;
            t_est = t;

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x_est, y_est, t_est using a PF




            // ****************** Additional Student Code: End   ************

        }
        #endregion

    }
}

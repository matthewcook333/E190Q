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
        public double x_des, y_des, t_des;
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
        private double maxVelocity = 0.25;
        private double Kpho = 1;
        private double Kalpha = 2;//8
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

        public double rotRateL, rotRateR;
        public double K_p, K_i, K_d, maxErr;

        public double accCalib_x = 18;
        public double accCalib_y = 4;

        // PF Variables
        public Map map;
        public Particle[] particles;
        public Particle[] propagatedParticles;
        public int numParticles = 1000;
        public double K_wheelRandomness = 0.15;//0.25
        public Random random = new Random();
        public bool newLaserData = false;
        public double laserMaxRange = 4.0;
        public double laserMinRange = 0.2;
        public double[] laserAngles;
        private int laserCounter;
        private int laserStepSize = 3;
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

        // Motion Planner Variables
        const int numXCells = 20;
        const int numYCells = 20;
        const int maxNumNodes = 5000;
        const float minWorkspaceX = -10.0f;
        const float maxWorkspaceX = 10.0f;
        const float minWorkspaceY = -10.0f;
        const float maxWorkspaceY = 10.0f;

        // Motion Planner Variables 
        public double samplingCellSizeX, samplingCellSizeY;
        public int numOccupiedCells;
        public int[] occupiedCellsList;
        public int[] numNodesInCell;
        public Node[,] NodesInCells;
        public Node[] trajList, nodeList;
        public int trajSize, trajCurrentNode, numNodes;

        public class Node
        {
            public double x, y;
            public int lastNode;
            public int nodeIndex;

            public Node()
            {
                x = 0;
                y = 0;
                lastNode = 0;
                nodeIndex = 0;
            }

            public Node(double _x, double _y, int _nodeIndex, int _lastNode)
            {
                x = _x;
                y = _y;
                nodeIndex = _nodeIndex;
                lastNode = _lastNode;
            }
        }


        #endregion


        #region Navigation Setup
        
        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            map = new Map();

            // Create particles
            particles = new Particle[numParticles];
            propagatedParticles = new Particle[numParticles];
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
            // Initialize state estimates
            x = 0;//initialX;
            y = 0;//initialY;
            t = 0;//initialT;

            // Initialize state estimates
            x_est = 0;//initialX;
            y_est = 0;//initialY;
            t_est = 0;//initialT;

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

            // MP variable setup
            occupiedCellsList = new int[numXCells * numYCells];
            numNodesInCell = new int[numXCells * numYCells];
            NodesInCells = new Node[numXCells * numYCells, 500];
            trajList = new Node[maxNumNodes];
            nodeList = new Node[maxNumNodes];
            numNodes = 0;
            trajList[0] = new Node(0, 0, 0, 0);
            trajSize = 0;


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

                // Get most recenct encoder measurements
                currentEncoderPulseL = simulatedJaguar.GetEncoderPulse4();
                currentEncoderPulseR = simulatedJaguar.GetEncoderPulse5();

                // Get most recent laser scanner measurements
                laserCounter = laserCounter + deltaT;
                if (laserCounter >= 2000)
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

            double K_u = 163;// 140;
            double T_u = 29;// 8;
            double K_p = 0.6 /*0.725*/ * K_u;// 25;
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

            // Put code here to calculate desiredRotRateR and 
            // desiredRotRateL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!
            double deltaX = x_des - x_est;
            double deltaY = y_des - y_est;
            double pho = Math.Sqrt(Math.Pow(deltaX, 2) + Math.Pow(deltaY, 2));
            double alpha = AngleDiff(t_est, Math.Atan2(deltaY, deltaX));
            int isBackwards = 1;
            // Check if its optimal to travel backwards!
            if (Math.Abs(alpha) > Math.PI / 2)
            {
                alpha = AngleDiff(t_est, Math.Atan2(-deltaY, -deltaX));
                isBackwards = -1;
            }

            double beta = AngleDiff(t_est, -alpha);
            // adding desired T
            beta = AngleDiff(-t_des, beta);

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

            if (Math.Abs(pho) < .1)
            {
                withinEpsilon = true;
            }

            if (withinEpsilon)
            {
                double thetaError = AngleDiff(t_des, t_est);
                double epsilon = 0.175;
                short spinSpeed = (short)(60 + Math.Abs(thetaError) * 15 / Math.PI);

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



        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {
            double distToCurrentNode = Math.Sqrt(Math.Pow(x_est - trajList[trajCurrentNode].x, 2) + Math.Pow(y_est - trajList[trajCurrentNode].y, 2));
            if (distToCurrentNode < 0.1 && trajCurrentNode + 1 < trajSize)
            {
                trajCurrentNode++;
                x_des = trajList[trajCurrentNode].x;
                y_des = trajList[trajCurrentNode].y;
                t_des = 0;
                Console.WriteLine("trajCurrent node is " + trajCurrentNode + " x: " + x_des + " y: " + y_des);
            }

            FlyToSetPoint();
        }

        // This function houses the core motion planner. This function
        // will generate a new trajectory when called. Students must 
        // add their code here.

        private void PRMMotionPlanner()
        {
            // Initialize sampling grid cell variables for weighted
            // random selection of nodes to expand.
            samplingCellSizeX = (maxWorkspaceX - minWorkspaceX) / numXCells;
            samplingCellSizeY = (maxWorkspaceY - minWorkspaceY) / numYCells;
            numOccupiedCells = 0;
            for (int i = 0; i < numXCells * numYCells; i++)
                numNodesInCell[i] = 0;
            numNodes = 0;


            // ****************** Additional Student Code: Start ************

            // Put code here to expand the PRM until the goal node is reached,
            // or until a max number of iterations is reached.


            // Create and add the start Node
            Node startNode = new Node(x_est, y_est, 0, 0);
            AddNode(startNode);

            // Create the goal node
            Node goalNode = new Node(desiredX, desiredY, 0, 0);

            // Loop until path created
            bool pathFound = false;
            int maxIterations = maxNumNodes;
            int iterations = 0;
            Random randGenerator = new Random();

            double maxDistExpand = 1; // longest length the node can expand

            while (iterations < maxIterations && !pathFound)
            {
                int randCellNumber = randGenerator.Next(0, numOccupiedCells);
                int randNodeNumber = randGenerator.Next(0, numNodesInCell[occupiedCellsList[randCellNumber]]);
                Node randExpansionNode = NodesInCells[occupiedCellsList[randCellNumber], randNodeNumber];
                double distExpand = randGenerator.NextDouble() * maxDistExpand;
                double orientationExpand = random.Next(-314159, 314159) / 100000.0;

                double newX = randExpansionNode.x + distExpand * Math.Cos(orientationExpand);
                double newY = randExpansionNode.y + distExpand * Math.Sin(orientationExpand);
                Node newNode = new Node(newX, newY, numNodes, randExpansionNode.nodeIndex);

                if (!map.CollisionFound(randExpansionNode, newNode, robotRadius))
                {
                    AddNode(newNode);
                    if (!map.CollisionFound(newNode, goalNode, robotRadius))
                    {
                        goalNode.nodeIndex = numNodes;
                        goalNode.lastNode = newNode.nodeIndex;
                        AddNode(goalNode);
                        pathFound = true;
                    }
                }
                

                // Increment number of iterations
                iterations++;
            }


            // Create the trajectory to follow
            BuildTraj(goalNode);

            
            // ****************** Additional Student Code: End   ************




        }




        // This function is used to implement weighted sampling in 
        // when randomly selecting nodes to expand from in the PRM.
        // The work environment is divided into a grid of cells.
        // This function returns the cell number.
        int GetCellNumber(double x, double y)
        {
            int cell = (int)Math.Floor((x - minWorkspaceX) / samplingCellSizeX) + (int)(Math.Floor((y - minWorkspaceY) / samplingCellSizeY) * numXCells);
            return cell;
        }

        // This function is also used to implement weighted sampling in 
        // when randomly selecting nodes to expand from in the PRM.
        // When new nodes for the PRM are generated, they must be added
        // to a variety of memory locations.
        // First, the node is stored in a list of nodes specific to a grid
        // cell. If this is the first node in that grid cell, the list of 
        // occupied cells is updated. Then, the node is stored in a general
        // list that keeps track of all nodes for building the final
        // trajectory.

        void AddNode(Node n)
        {
            int cellNumber = GetCellNumber(n.x, n.y);
            if (numNodesInCell[cellNumber] < 1)
            {
                occupiedCellsList[numOccupiedCells] = cellNumber;
                numOccupiedCells++;
            }

            if (numNodesInCell[cellNumber] < 400)
            {
                NodesInCells[cellNumber, numNodesInCell[cellNumber]] = n;
                numNodesInCell[cellNumber]++;

                // Add to nodelist
                nodeList[numNodes] = n;
                numNodes++;
            }
            return;
        }


        // Given the goal node, this function will recursively add the
        // parent node to a trajectory until the start node is reached.
        // The result is a list of nodes that connect the start node to
        // the goal node with collision free edges.

        void BuildTraj(Node goalNode)
        {
            Node[] tempList = new Node[maxNumNodes];
            for (int j = 0; j < maxNumNodes; j++)
                trajList[j] = new Node(0, 0, 0, 0);

            tempList[0] = goalNode;
            int i = 1;

            // Make backwards traj by looking at parent of every child node
            while (tempList[i - 1].nodeIndex != 0)
            {
                tempList[i] = nodeList[tempList[i - 1].lastNode];
                i++;
            }

            // Reverse trajectory order
            for (int j = 0; j < i; j++)
            {
                trajList[j] = tempList[i - j - 1];
            }

            // Set size of trajectory and initialize node counter
            trajSize = i;
            trajCurrentNode = 0;
            /*
             * for (int pointnum = 0; pointnum < trajSize; ++pointnum) 
            {
                Console.WriteLine("point " + pointnum + " is at " + trajList[pointnum].x + " " + trajList[pointnum].y);

            }
             * */
            return;
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
        public void LocalizeRealWithOdometry()
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi
            // Update the actual
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

            // Put code here to calculate x_est, y_est, t_est using a PF
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
                //double DistREst = GaussianDist(wheelDistanceR, wheelDistanceR * 0.75);
                //double DistLEst = GaussianDist(wheelDistanceL, wheelDistanceL * 0.75);
                PFEncoderDiffL = (currentEncoderPulseL - PFLastEncoderL);
                PFEncoderDiffR = (currentEncoderPulseR - PFLastEncoderR);

                // check for rollover
                if (Math.Abs(PFEncoderDiffL) > pulsesPerRotation)
                {
                    PFEncoderDiffL = PFEncoderDiffL < 0 ? PFEncoderDiffL + encoderMax : encoderMax - PFEncoderDiffL;
                }
                if (Math.Abs(PFEncoderDiffR) > pulsesPerRotation)
                {
                    PFEncoderDiffR = PFEncoderDiffR < 0 ? PFEncoderDiffR + encoderMax : encoderMax - PFEncoderDiffR;
                }

                // calculate wheel distance and change in distance and angle travelled
                double PFDistanceL = (2 * Math.PI * wheelRadius * PFEncoderDiffL) / pulsesPerRotation;
                double PFDistanceR = -(2 * Math.PI * wheelRadius * PFEncoderDiffR) / pulsesPerRotation;
                PFDistanceL = GaussianDist(PFDistanceL, PFDistanceL * 0.2);
                PFDistanceR = GaussianDist(PFDistanceR, PFDistanceR * 0.2);

                double estAngleTravelled = (PFDistanceR - PFDistanceL) / (2 * robotRadius);
                double estDistanceTravelled = (PFDistanceR + PFDistanceL) / 2;
                //double estAngleTravelled = (DistREst - DistLEst) / (2 * robotRadius);
                //double estDistanceTravelled = (DistREst + DistLEst) / 2;

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

            // Put code here to calculate propagatedParticles[p].w. Feel free to use the
            // function map.GetClosestWallDistance from Map.cs.
            propagatedParticles[p].w = 0;
            double variance = 0.030; // meters
            for (int i = 0; i < LaserData.Length; i = i + laserStepSize)
            {
                double expectedRange = map.GetClosestWallDistance(
                    propagatedParticles[p].x,
                    propagatedParticles[p].y,
                    propagatedParticles[p].t - 1.57 + laserAngles[i]);
                double sampledRange = LaserData[i] / 1000.0;
                double weight = (1.0 / (Math.Sqrt(variance) * Math.Sqrt(2 * Math.PI))) *
                    Math.Pow(Math.E, (-Math.Pow(sampledRange - expectedRange, 2) / (2 * variance)));
                propagatedParticles[p].w += weight;
            }
            // ****************** Additional Student Code: End   ************

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

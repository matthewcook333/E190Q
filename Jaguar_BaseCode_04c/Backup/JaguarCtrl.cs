
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Runtime.InteropServices;
using EARTHLib;
using System.Drawing.Drawing2D;
using System.IO;
using Microsoft.DirectX.DirectInput;
using System.Threading;

namespace DrRobot.JaguarControl
{
    public partial class JaguarCtrl : Form
    {
        DrRobotRobotConnection drRobotConnect = null;
        RobotConfig robotCfg = null;
        RobotConfig.RobotConfigTableRow jaguarSetting = null;
        private const string configFile = "c:\\DrRobotAppFile\\OutDoorRobotConfig.xml";

        private const double TEMPERATURE_TH = 60.0;
                
        #region For Google Earth application
        [StructLayout(LayoutKind.Sequential)]
        public struct RECT
        {
            public int X;
            public int Y;
            public int Width;
            public int Height;
        }

        [DllImport("user32.dll", SetLastError = true)]
        static extern bool GetWindowRect(IntPtr hWnd, out RECT rect);

        public delegate int EnumWindowsProc(IntPtr hwnd, int lParam);

        [DllImport("user32", CharSet = CharSet.Auto)]
        public extern static IntPtr GetParent(IntPtr hWnd);

        [DllImport("user32", CharSet = CharSet.Auto)]
        public extern static bool MoveWindow(IntPtr hWnd, int X, int Y, int nWidth, int nHeight, bool bRepaint);

        [DllImport("user32", CharSet = CharSet.Auto)]
        public extern static IntPtr SetParent(IntPtr hWndChild, IntPtr hWndNewParent);

        [DllImport("user32.dll")]
        private static extern bool ShowWindowAsync(
        int hWnd,
        int nCmdShow);

        [DllImport("user32.dll", CharSet = CharSet.Auto)]
        public extern static bool SetWindowPos(int hWnd, IntPtr hWndInsertAfter, int X, int Y, int cx, int cy, uint uFlags);

        [DllImport("user32", CharSet = CharSet.Auto)]
        public extern static IntPtr PostMessage(int hWnd, int msg, int wParam, int IParam);


        readonly IntPtr HWND_BOTTOM = new IntPtr(1);
        readonly IntPtr HWND_NOTOPMOST = new IntPtr(-2);
        readonly IntPtr HWND_TOP = new IntPtr(0);
        readonly IntPtr HWND_TOPMOST = new IntPtr(-1);
        /*
        static readonly UInt32 SWP_NOSIZE = 1;
        static readonly UInt32 SWP_NOMOVE = 2;
        static readonly UInt32 SWP_NOZORDER = 4;
        static readonly UInt32 SWP_NOREDRAW = 8;
        static readonly UInt32 SWP_NOACTIVATE = 16;
        static readonly UInt32 SWP_FRAMECHANGED = 32;
        static readonly UInt32 SWP_SHOWWINDOW = 64;
        static readonly UInt32 SWP_HIDEWINDOW = 128;
        static readonly UInt32 SWP_NOCOPYBITS = 256;
        static readonly UInt32 SWP_NOOWNERZORDER = 512;
        static readonly UInt32 SWP_NOSENDCHANGING = 1024;
        static readonly Int32 WM_CLOSE = 0xF060;
         */
        static readonly Int32 WM_QUIT = 0x0012;


        private IntPtr GEHrender = (IntPtr)0;
        private IntPtr GEParentHrender = (IntPtr)0;

        public ApplicationGEClass googleEarth;
        public CameraInfoGE cam = null;


        private string cenerLatitude = "43.855159";
        private string centerLongitude = "-79.3615177";
        private int cameraRange = 200;
        private double preEstLatitude = 0;
        private double preEstLongitude = 0;

        private double preLatitude = 0;
        private double preLongtitude = 0;
        private double curLatitude = 0;
        private double curLongitude = 0;


        private string kmlFileStr1 = @"<?xml version=""1.0"" encoding=""utf-8""?><kml xmlns=""http://www.opengis.net/kml/2.2""><Document>";
        private string kmlFileStyle = @"<Style id=""MyLineStyle""><LineStyle><color>7f0000ff</color><width>4</width></LineStyle></Style>";
        //private string kmlFileLookAt = @"<LookAt><longitude>-79.3607666</longitude><latitude>43.855015</latitude><altitude>0</altitude><range>150</range><heading>0</heading></LookAt>";

        private string kmlFilePlacemark = @"<Placemark><name>unextruded</name><styleUrl>#MyLineStyle</styleUrl><LineString><extrude>1</extrude><tessellate>1</tessellate>";
        private string kmlFileCoordinate = @"<coordinates>-79.3607666,43.855015,0 -79.3608,43.855015,0</coordinates>";
        private string kmlFileEnd = @"</LineString></Placemark></Document></kml>";


        //private string kmlFileName = System.Environment.CurrentDirectory + "\\gpstest0.kml";
                
        //private static double startLat = 0;
        //private static double startLongitude = 0;


        private SetColor setColor = new SetColor ();

        public class SetColor
        {
            public string SetRed = "0x7f0000ff";
            public string SetGreen = "0x7f00ff00";
            public string SetBlue = "0x7fff0000";
        }

        #endregion

        #region motor variable define
        public class MotorData
        {
            public int pwmOutput = 0;
            public int encodeSpeed = 0;
            public int encoderPos = 0;
            public int encoderDir = 0;
        }
        //here is arm motor array, 0 for front arm, 1 for rear arm
        private MotorData[] armMotor = new MotorData[2];
        private int[] armEncoder = new int[2]{0,0};
        private int[] preArmEncoder = new int[2]{0,0};
        private bool firstEncoderData = true;
        private const int ARM_CIRCLE_CNT = 1140;         // 5*4*285 /5 = 1140
        private double[] armPosAngle = new double[2]{0,0};
        private double[] preArmPos = new double[2]{0,0};
        private int[] armResetPos = new int[2] { 0, 0 };
        private double[] armPosStart = new double[2] { 0, 0 };

        private MotorData leftFrontWheelMotor = new MotorData();
        private MotorData rightFrontWheelMotor = new MotorData();
        private MotorData leftRearWheelMotor = new MotorData();
        private MotorData rightRearWheelMotor = new MotorData();

        private int forwardPower = 0;
        private int turnPower = 0;

        
        //for temperature sensor
        private double[] resTable = new double[25]{114660,84510,62927,47077,35563,27119,20860,16204,12683,10000,
                        7942,6327,5074,4103,3336,2724,2237,1846,1530,1275,1068,899.3,760.7,645.2,549.4};
        private double[] tempTable = new double[25] { -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100 };
        private const double FULLAD = 4095;

        private double stuckVelTH = 50;
        private double stuckAcc = 0;
        private double stuckPWMTH = 12000;
        private double stuckAccTH = 50;
        #endregion

        #region variable for Joystick Control
        private bool forceStop = false;
        private bool protectMotorTemp = false;
        private bool protectMotorStuck = false;
        private const int NOCONTROL = -32768;
        int armCmd1 = 0;
        int armCmd2 = 0;
        int exeTime = 1000;
        private bool lightOn = false;
        private int expandIO = 0xff;
        private const short LEFTARMCHANNEL = 0;
        private const short RIGHTARMCHANNEL = 1;

        private const short LEFTWHEELCHANNEL = 3;           //3
        private const short RIGHTWHEELCHANNEL = 4;          //4
        public static JoystickState[] joyState = new JoystickState[2]; //maybe we need second Joystick in future
        private static int[] preSetButton = new int[2]{-1,-1};  //no any button pressed
        private static int[] joyButtonCnt = new int[2]{0,0};
        private const int JOYDELAY = 10;
        
        private Device[] applicationDevice = new Device[2];

        private bool blnJoyStick = true;
        private const int MAXPWM = 16384;
        private const int MINPWM = 4000;
        private const int INIPWM = 16384;
        private int MOTDIR = 1;
     

        private const int CMD_INT_TIME = 10;
        private bool armJoy2 = false;
        private bool armJoy1 = false;

        private const short armChannel1 = 0;
        private const short armChannel2 = 1;
        private bool lightCtrl = true;
        private const int ARM_DELAY_CNT = 40;
        private const int ARM_VEL1 = 200;
        private const int ARM_VEL2 = 1000;
        #endregion

        /////////////////////////////////////////////////////////////////
        
        #region form funtion
        public JaguarCtrl(DrRobotRobotConnection form, RobotConfig robotConfig)
        {
            drRobotConnect = form;
            robotCfg = robotConfig;
            jaguarSetting = (RobotConfig.RobotConfigTableRow)robotCfg.RobotConfigTable.Rows[0];
            InitializeComponent();
        }

        private void JaguarCtrl_Shown(object sender, EventArgs e)
        {
            drRobotConnect.Hide ();
            this.Focus();
        }

        private void JaguarCtrl_FormClosed(object sender, FormClosedEventArgs e)
        {
            myAMC.Stop();

            try
            {
                if (drRobotConnect != null)
                    drRobotConnect.Close();
            }
            catch
            {
            }
        }

        private void JaguarCtrl_Load(object sender, EventArgs e)
        {
            for (int i = 0; i < 2; i++)
            {
                armMotor[i] = new MotorData();
            }
            myJaguar.connectRobot (jaguarSetting.RobotID);
            
            //for google earth application
            cenerLatitude = jaguarSetting.GoogleEarthStartLat.ToString ();
            centerLongitude = jaguarSetting.GoogleEarthStartLong.ToString();
            txtStartLat.Text = cenerLatitude;
            txtStartLong.Text = centerLongitude;
            if (this.DesignMode == false)
            {
                googleEarth = new ApplicationGEClass();

                MessageBox.Show("Waiting for GoogleEarth to be loaded…. \n This may take a while.When the GoogleEarth is shown, click OK to continue.", "DrRobot Jaguar Control", MessageBoxButtons.OK, MessageBoxIcon.Asterisk);
                //ShowWindowAsync(googleEarth.GetMainHwnd(), 0);

                GEHrender = (IntPtr)googleEarth.GetRenderHwnd();
                GEParentHrender = GetParent(GEHrender);

                MoveWindow(GEHrender, 0, 0, this.panelGE.Width, this.panelGE.Height, true);
                SetParent(GEHrender, this.panelGE.Handle);



                // don'tdo it now , we keep google earth window to clear the KML data in temporary node
                //Hide the main GE window
                //SetWindowPos(googleEarth.GetMainHwnd(), HWND_BOTTOM, 0, 0, 0, 0, SWP_NOSIZE + SWP_HIDEWINDOW); 
            }
        
        

            //for Axis camera
            try
            {
                // Set properties, deciding what url completion to use by MediaType.
                myAMC.MediaUsername = jaguarSetting.CameraUser;
                myAMC.MediaPassword = jaguarSetting.CameraPWD;
                myAMC.MediaType = "mjpeg";
                myAMC.MediaURL = CompleteURL(jaguarSetting.CameraIP + ":" + jaguarSetting .CameraPort , "mjpeg");
                myAMC.AudioConfigURL = "http://" + jaguarSetting.CameraIP + ":" + jaguarSetting .CameraPort + "/axis-cgi/view/param.cgi?camera=1&action=list&group=Audio,AudioSource.A0";

                /*
                myAMC.PTZControlURL = "http://" + jaguarSetting.CameraIP + ":" + jaguarSetting.CameraPort + "/axis-cgi/com/ptz.cgi";
                myAMC.UIMode = "ptz-absolute";
                myAMC.ShowToolbar = true;
                myAMC.StretchToFit = true;
                myAMC.EnableContextMenu = true;
                myAMC.ToolbarConfiguration = "default,+ptz";
                */
                myAMC.ToolbarConfiguration = "default, + audiocontrol,+ rec";

                myAMC.AudioReceiveStop();
                // Start the streaming
                myAMC.Play();
                myAMC.AudioReceiveURL = "http://" + jaguarSetting.CameraIP + ":" + jaguarSetting .CameraPort + "/axis-cgi/audio/receive.cgi";
                myAMC.Volume = 100;
                //myAMC.AudioReceiveStart();
                myAMC.AudioTransmitURL = "http://" + jaguarSetting.CameraIP + ":" + jaguarSetting.CameraPort + "/axis-cgi/audio/transmit.cgi";
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, "Unable to play stream: " + ex.Message);
            }
            //for joystick control
            if (InitDirectInput())
                tmrJoyStickPoll.Enabled = true;


            //for GPS/IMU communication
            if ((jaguarSetting.IMUIP !="0.0.0.0") && (jaguarSetting .GPSIP != "0.0.0.0"))
            {
                startComm ();
            }

            // for laser scan
            if ((jaguarSetting.LaserRangeIP != "0.0.0.0") && (jaguarSetting.LaserRangePort != 0))
            {
                startCommLaser();
                btnLaserScan.Enabled = true;
            }
            else
            {
                btnLaserScan.Enabled = false;
            }
            drawLaserBackground();
            MOTDIR = jaguarSetting .MotorDir;
            if (jaguarSetting.RobotType.ToLower() == "jaguar")
            {
                armPosIndicator.SetArmPositionIndicatorParameters(0,0);
                pictureBoxRobot.SendToBack();
                armPosIndicator.BringToFront();
                lblMot1.Text = "Front Arm Motor:";
                lblMot2.Text = "Reserved:";
                lblMot3.Text = "Left Track Motor:";
                lblMot4.Text = "Right Track Motor:";
                btnDisArm.Visible = true;
            }
            else if (jaguarSetting.RobotType.ToLower() == "jaguar_4arms")
            {
                armPosIndicator.Set2ArmCtrl(true);
                armPosIndicator.SetArmPositionIndicatorParameters(0, 0);
                pictureBoxRobot.SendToBack();
                armPosIndicator.BringToFront();
                lblMot1.Text = "Front Arm Motor:";
                lblMot2.Text = "Rear Arm Motor:";
                lblMot3.Text = "Left Track Motor:";
                lblMot4.Text = "Right Track Motor:";
                btnDisArm.Visible = true;
            }
            else
            {
                armPosIndicator.SendToBack();
                pictureBoxRobot.BringToFront();
               
                btnDisArm.Visible = false;
                if (jaguarSetting.RobotType.ToLower() == "jaguar_lite")
                {
                    pictureBoxRobot.Image = imageList2.Images[0];
                    lblMot1.Text = "Left Track Motor:";
                    lblMot2.Text = "Right Track Motor:";
                    lblMot3.Visible = false;
                    lblMot4.Visible = false;
                    lblEncoderPos3.Visible = false;
                    lblEncoderPos4.Visible = false;
                    lblVel3.Visible = false;
                    lblVel4.Visible = false;
                    lblTemp3.Visible = false;
                    lblTemp4.Visible = false;
 
                }
                else if (jaguarSetting.RobotType.ToLower() == "jaguar_4x4track")
                {
                    pictureBoxRobot.Image = imageList2.Images[1];
                    lblMot1.Text = "Left Front Motor:";
                    lblMot2.Text = "Right Front Motor:";
                    lblMot3.Text = "Left Rear Motor:";
                    lblMot4.Text = "Right Rear Motor:";
                }
                else //if (jaguarSetting.RobotType.ToLower() == "jaguar_4x4wheel")
                {
                    
                    pictureBoxRobot.Image = imageList2.Images[2];
                    lblMot1.Text = "Left Front Wheel:";
                    lblMot2.Text = "Right Front Wheel:";
                    lblMot3.Text = "Left Rear Wheel:";
                    lblMot4.Text = "Right Rear Wheel:";
                }
                

            }
            pictureBoxStuckDetect.BackColor = Color.Green;
            btnScan.Enabled = false;
        }

        private void JaguarCtrl_FormClosing(object sender, FormClosingEventArgs e)
        {
            tmrDisplay.Enabled = false;
            tmrDrawing.Enabled = false;
            tmrJoyStickPoll.Enabled = false;
            stopCommunication();
            stopCommunicationLaser();
            if ((startRecord) || (SW != null))
                SW.Close();

            //we save google earth start point here
            double centerLat = double.Parse(cenerLatitude);
            double centerLong = double.Parse(centerLongitude);
            try
            {
                centerLat = double.Parse(txtStartLat.Text);
            }
            catch
            {
            }
            try
            {
                centerLong = double.Parse(txtStartLong.Text);
            }
            catch
            {
            }

            jaguarSetting.GoogleEarthStartLat = centerLat;
            jaguarSetting.GoogleEarthStartLong = centerLong;
            try
            {
                robotCfg.WriteXml(configFile);
            }
            catch
            {
            }
            try
            {
                PostMessage(googleEarth.GetMainHwnd(), WM_QUIT, 0, 0);
            }
            catch
            {
            }
            //wait for Gooogle Earth close
            Thread.Sleep(1000);

        }
        #endregion

        #region Axis Camera Control
        string CompleteURL(string theMediaURL, string theMediaType)
        {
            string anURL = theMediaURL;
            if (!anURL.EndsWith("/")) anURL += "/";

            if (theMediaType == "mjpeg")
            {
                anURL += "axis-cgi/mjpg/video.cgi";
            }
            else if (theMediaType == "mpeg4")
            {
                anURL += "mpeg4/media.amp";
            }
            else if (theMediaType == "mpeg2-unicast")
            {
                anURL += "axis-cgi/mpeg2/video.cgi";
            }
            else if (theMediaType == "mpeg2-multicast")
            {
                anURL += "axis-cgi/mpeg2/video.cgi";
            }

            anURL = CompleteProtocol(anURL, theMediaType);
            return anURL;
        }

        string CompleteProtocol(string theMediaURL, string theMediaType)
        {
            if (theMediaURL.IndexOf("://") >= 0) return theMediaURL;

            string anURL = theMediaURL;

            if (theMediaType == "mjpeg")
            {
                anURL = "http://" + anURL;
            }
            else if (theMediaType == "mpeg4")
            {
                anURL = "axrtsphttp://" + anURL;
            }
            else if (theMediaType == "mpeg2-unicast")
            {
                anURL = "http://" + anURL;
            }
            else if (theMediaType == "mpeg2-multicast")
            {
                anURL = "axsdp" + anURL;
            }

            return anURL;
        }

        #endregion

        #region GoogleEarth function
        //this is a "Look At" function
        private void btnSetStartPoint_Click(object sender, EventArgs e)
        {
            cam = new CameraInfoGE();
            double centerLat = double.Parse(cenerLatitude);
            double centerLong = double.Parse(centerLongitude);
            try
            {
                centerLat = double.Parse(txtStartLat.Text);
            }
            catch
            {
            }
            try
            {
                centerLong = double.Parse(txtStartLong.Text);
            }
            catch
            {
            }

            centerLat = (centerLat > 90 ? 90 : centerLat);
            centerLat = (centerLat < -90 ? -90 : centerLat);

            centerLong = (centerLong > 180 ? 180 : centerLong);
            centerLong = (centerLong < -180 ? -180 : centerLong);
            cam.FocusPointLatitude = centerLat;
            cam.FocusPointLongitude = centerLong;
            cam.FocusPointAltitude = 0;
            cam.Range = cameraRange;
            cam.Tilt = 0;


            googleEarth.SetCamera(cam, 0.6);
        }

        //this is a zoom function
        private void trackBarZoom_Scroll(object sender, EventArgs e)
        {
            if (cam != null)
            {
                cameraRange = trackBarZoom.Value;
                cam.Range = cameraRange;
                googleEarth.SetCamera(cam, 0.6);
            }
        }

        // this function will set map center as default map start point
        private void btnSetMapCenter_Click(object sender, EventArgs e)
        {
            PointOnTerrainGE pointGe = new PointOnTerrainGE();
            pointGe = googleEarth.GetPointOnTerrainFromScreenCoords(0, 0);


            double lat = pointGe.Latitude;
            double longitude = pointGe.Longitude;
            txtStartLat.Text = lat.ToString();
            txtStartLong.Text = longitude.ToString();

            btnSetStartPoint_Click(null, null);
        }

        #endregion

        #region myJaguar ActiveX event
        private void myJaguar_StandardSensorEvent(object sender, EventArgs e)
        {
            double vol = (double)myJaguar.GetSensorBatteryAD2() / 4095 * 34.498;
            //to protect Li-Po Battery, you'd better shut down when battery is lower than 22.2V (3.7V*6)
            if (vol < 22.2)
            {
                lblBatVol.ForeColor = Color.Red;
            }
            else
            {
                lblBatVol.ForeColor = Color.Black;
            }

            lblBatVol.Text = vol.ToString("0.00");
            //here is the board voltage 5V, not display on the UI
            vol = (double)myJaguar.GetSensorBatteryAD1() / 4095 * 9;
            
        }

        private void myJaguar_MotorSensorEvent(object sender, EventArgs e)
        {
            //here read back exactly output PWM value for all motors
            armMotor[0].pwmOutput = myJaguar.GetMotorPWMValue1();
            armMotor[1].pwmOutput = myJaguar.GetMotorPWMValue2();         //right motor still use PWM channel 0,only for Jaguar_4arm
            //here is front left arm encoder reading
            armMotor[0].encoderDir = myJaguar.GetEncoderDir1();
            armMotor[0].encoderPos = myJaguar.GetEncoderPulse1();
            armMotor[0].encodeSpeed = myJaguar.GetEncoderSpeed1();
            //here is front right arm encoder reading
            armMotor[1].encoderDir = myJaguar.GetEncoderDir2();
            armMotor[1].encoderPos = myJaguar.GetEncoderPulse2();
            armMotor[1].encodeSpeed = myJaguar.GetEncoderSpeed2();


            forwardPower  = myJaguar.GetMotorPWMValue4();   //actually is forward PWM power
            turnPower = myJaguar.GetMotorPWMValue5();  //actually is tunning PWM power

            if ((jaguarSetting.RobotType.ToLower() == "jaguar") || (jaguarSetting.RobotType.ToLower() == "jaguar_4arm") )
            {
                // display here
                lblEncoderPos1.Text = armMotor[0].encoderPos.ToString ();
                lblEncoderPos2.Text = armMotor[1].encoderPos.ToString();
                

                lblVel1.Text = (armMotor[0].encodeSpeed * armMotor[0].encoderDir).ToString();
                lblVel2.Text = (-armMotor[1].encodeSpeed * armMotor[1].encoderDir).ToString();
                armEncoder[0] = armMotor[0].encoderPos;
                armEncoder[1] = armMotor[1].encoderPos;
                if (firstEncoderData)
                {
                    preArmEncoder[0] = armEncoder[0];
                    preArmEncoder[1] = armEncoder[1];
                    firstEncoderData = false;
                }
                else
                {
                    //switch to angle to display
                    for (int i = 0; i < 2; i++)
                    {

                        int temp1 = -(armEncoder[i] - preArmEncoder[i]);
                        if (Math.Abs(temp1) > ARM_CIRCLE_CNT)
                        {
                            if (temp1 > 0)
                            {
                                temp1 = -(32767 - preArmEncoder[i] + armEncoder[i]);
                            }
                            else
                            {
                                temp1 = ((32767 - armEncoder[i]) + preArmEncoder[i]);
                                
                            }
                        }

                        temp1 = temp1 % ARM_CIRCLE_CNT;
                        double temp2 = (double)temp1 / ARM_CIRCLE_CNT * 360;
                        armPosAngle[i] = preArmPos[i] + temp2;
                        if (armPosAngle[i] > 360)
                        {
                            armPosAngle[i] = armPosAngle[i] - 360;
                        }
                        else if (armPosAngle[i] < 0)
                        {
                            armPosAngle[i] = 360 - Math.Abs(armPosAngle[i]);
                        }
                        preArmPos[i] = armPosAngle[i];
                        preArmEncoder[i] = armEncoder[i];
                        
                        
                    }
                    armPosIndicator.SetArmPositionIndicatorParameters(armPosAngle[0],armPosAngle[1]);
                }

                

            }
            else
            {
                if (jaguarSetting.RobotType.ToLower() == "jaguar_lite")
                {
                    
                    
                }
                else if( (jaguarSetting .RobotType .ToLower () == "jaguar_4x4track") || (jaguarSetting.RobotType.ToLower() == "jaguar_4x4wheel") )
                {
                    leftRearWheelMotor.encoderDir = myJaguar.GetEncoderDir1();
                    leftRearWheelMotor.encoderPos = myJaguar.GetEncoderPulse1();
                    leftRearWheelMotor.encodeSpeed = myJaguar.GetEncoderSpeed1();
                    
                    rightRearWheelMotor.encoderDir = myJaguar.GetEncoderDir2();
                    rightRearWheelMotor.encoderPos = myJaguar.GetEncoderPulse2();
                    rightRearWheelMotor.encodeSpeed = myJaguar.GetEncoderSpeed2();
                    // display here
                    lblEncoderPos3.Text = leftRearWheelMotor.encoderPos.ToString();
                    lblEncoderPos4.Text = rightRearWheelMotor.encoderPos.ToString();

                    lblVel3.Text = (MOTDIR * leftRearWheelMotor.encodeSpeed * leftRearWheelMotor.encoderDir).ToString();
                    lblVel4.Text = (-MOTDIR * rightRearWheelMotor.encodeSpeed * rightRearWheelMotor.encoderDir).ToString();

                }
                
            }

        }

        private void myJaguar_CustomSensorEvent(object sender, EventArgs e)
        {
            //here is temperature detect
            // For a motor, if output PWM is very big but encoder speed is very low or zero, it usually means the motor 
            // works in a stuck state, the current will be very big, the motor will be damaged in a moment, so please be 
            // very carefull at these information, especialy if temperature of a motor is getting much higher(above 60 degree), stop the motor
            // for a while to protect the motor

            // now the temperature is connect to Extended AD port

            double tempM1 = Trans2Temperature((double)myJaguar.GetCustomAD5());
            tempM1 = double.Parse(tempM1.ToString("0.00"));


            double tempM2 = Trans2Temperature((double)myJaguar.GetCustomAD6());
            tempM2 = double.Parse(tempM2.ToString("0.00"));

            double tempM3 = Trans2Temperature((double)myJaguar.GetCustomAD7());
            tempM3 = double.Parse(tempM3.ToString("0.00"));

            double tempM4 = Trans2Temperature((double)myJaguar.GetCustomAD8());
            tempM4 = double.Parse(tempM4.ToString("0.00"));

            
            leftFrontWheelMotor.encoderPos = myJaguar.GetEncoderPulse4();
            leftFrontWheelMotor.encodeSpeed = myJaguar.GetEncoderSpeed4();
            leftFrontWheelMotor.encoderDir = myJaguar.GetEncoderDir4();

            rightFrontWheelMotor.encoderPos = myJaguar.GetEncoderPulse5();
            rightFrontWheelMotor.encodeSpeed = myJaguar.GetEncoderSpeed5();
            rightFrontWheelMotor.encoderDir = myJaguar.GetEncoderDir5();

            //here can read back left wheel motor and right wheel motor encoder information
            if ((jaguarSetting.RobotType.ToLower() == "jaguar") )
            {
                if (checkBoxMotorProtect.Checked)
                {
                    if ((tempM1 > TEMPERATURE_TH) || (tempM2 > TEMPERATURE_TH) || (tempM3 > TEMPERATURE_TH))
                    {
                        protectMotorTemp = true;
                    }
                    else
                    {
                        protectMotorTemp = false;
                    }
                }
                else
                {
                    protectMotorTemp = false;
                }


                if (tempM2 > TEMPERATURE_TH)
                    lblTemp1.ForeColor = Color.Red;
                else
                    lblTemp1.ForeColor = Color.Black;
                
                if (tempM1 > TEMPERATURE_TH)
                    lblTemp3.ForeColor = Color.Red;
                else
                    lblTemp3.ForeColor = Color.Black;

                if (tempM3 > TEMPERATURE_TH)
                    lblTemp4.ForeColor = Color.Red;
                else
                    lblTemp4.ForeColor = Color.Black;




                lblVel3.Text = (MOTDIR * leftFrontWheelMotor.encodeSpeed * leftFrontWheelMotor.encoderDir).ToString();
                lblVel4.Text = (-MOTDIR * rightFrontWheelMotor.encodeSpeed * rightFrontWheelMotor.encoderDir).ToString();
                lblTemp1.Text = tempM2.ToString("0.00");
                lblTemp2.Text = tempM4.ToString("0.00");
                lblTemp3.Text = tempM1.ToString("0.00");
                lblTemp4.Text = tempM3.ToString("0.00");
                lblEncoderPos3.Text = leftFrontWheelMotor.encoderPos.ToString();
                lblEncoderPos4.Text = rightFrontWheelMotor.encoderPos.ToString();
            }
            else if ((jaguarSetting.RobotType.ToLower() == "jaguar_4arms"))
            {
                if (checkBoxMotorProtect.Checked)
                {
                    if ((tempM1 > TEMPERATURE_TH) || (tempM2 > TEMPERATURE_TH) || (tempM3 > TEMPERATURE_TH) || (tempM4 > TEMPERATURE_TH))
                    {
                        protectMotorTemp = true;
                    }
                    else
                    {
                        protectMotorTemp = false;
                    }
                }
                else
                {
                    protectMotorTemp = false;
                }
                if (tempM3 > TEMPERATURE_TH)
                    lblTemp1.ForeColor = Color.Red;
                else
                    lblTemp1.ForeColor = Color.Black;
                
                if (tempM4 > TEMPERATURE_TH)
                    lblTemp2.ForeColor = Color.Red;
                else
                    lblTemp2.ForeColor = Color.Black;

                if (tempM1 > TEMPERATURE_TH)
                    lblTemp3.ForeColor = Color.Red;
                else
                    lblTemp3.ForeColor = Color.Black;

                if (tempM2 > TEMPERATURE_TH)
                    lblTemp4.ForeColor = Color.Red;
                else
                    lblTemp4.ForeColor = Color.Black;




                lblVel3.Text = (MOTDIR * leftFrontWheelMotor.encodeSpeed * leftFrontWheelMotor.encoderDir).ToString();
                lblVel4.Text = (-MOTDIR * rightFrontWheelMotor.encodeSpeed * rightFrontWheelMotor.encoderDir).ToString();
                lblTemp1.Text = tempM3.ToString("0.00");
                lblTemp2.Text = tempM4.ToString("0.00");
                lblTemp3.Text = tempM1.ToString("0.00");
                lblTemp4.Text = tempM2.ToString("0.00");
                lblEncoderPos3.Text = leftFrontWheelMotor.encoderPos.ToString();
                lblEncoderPos4.Text = rightFrontWheelMotor.encoderPos.ToString();
            }
            else if ((jaguarSetting.RobotType.ToLower() == "jaguar_4x4wheel") || (jaguarSetting.RobotType.ToLower() == "jaguar_4x4track"))
            {
                if (checkBoxMotorProtect.Checked)
                {
                    if ((tempM1 > TEMPERATURE_TH) || (tempM2 > TEMPERATURE_TH) || (tempM3 > TEMPERATURE_TH) || (tempM4 > TEMPERATURE_TH))
                    {
                        protectMotorTemp = true;
                    }
                    else
                    {
                        protectMotorTemp = false;
                    }
                }
                else
                {
                    protectMotorTemp = false;
                }

                if (tempM1 > TEMPERATURE_TH)
                    lblTemp1.ForeColor = Color.Red;
                else
                    lblTemp1.ForeColor = Color.Black;
                if (tempM2 > TEMPERATURE_TH)
                    lblTemp2.ForeColor = Color.Red;
                else
                    lblTemp2.ForeColor = Color.Black;
                if (tempM3 > TEMPERATURE_TH)
                    lblTemp3.ForeColor = Color.Red;
                else
                    lblTemp3.ForeColor = Color.Black;

                if (tempM4 > TEMPERATURE_TH)
                    lblTemp4.ForeColor = Color.Red;
                else
                    lblTemp4.ForeColor = Color.Black;


                lblVel1.Text = (MOTDIR * leftFrontWheelMotor.encodeSpeed * leftFrontWheelMotor.encoderDir).ToString();
                lblVel2.Text = (-MOTDIR * rightFrontWheelMotor.encodeSpeed * rightFrontWheelMotor.encoderDir).ToString();
                lblEncoderPos1.Text = leftFrontWheelMotor.encoderPos.ToString();
                lblEncoderPos2.Text = rightFrontWheelMotor.encoderPos.ToString();
                lblTemp1.Text = tempM1.ToString("0.00");
                lblTemp2.Text = tempM2.ToString("0.00");
                lblTemp3.Text = tempM3.ToString("0.00");
                lblTemp4.Text = tempM4.ToString("0.00");

            }
            else //jaguar_lite 
            {
                if (checkBoxMotorProtect.Checked)
                {
                    if ((tempM1 > TEMPERATURE_TH) || (tempM2 > TEMPERATURE_TH) )
                    {
                        protectMotorTemp = true;
                    }
                    else
                    {
                        protectMotorTemp = false;
                    }
                }
                else
                {
                    protectMotorTemp = false;
                }

                if (tempM1 > TEMPERATURE_TH)
                    lblTemp1.ForeColor = Color.Red;
                else
                    lblTemp1.ForeColor = Color.Black;
                if (tempM2 > TEMPERATURE_TH)
                    lblTemp2.ForeColor = Color.Red;
                else
                    lblTemp2.ForeColor = Color.Black;


                lblEncoderPos1.Text = leftFrontWheelMotor.encoderPos.ToString();
                lblEncoderPos2.Text = rightFrontWheelMotor.encoderPos.ToString();
                lblVel1.Text = (MOTDIR * leftFrontWheelMotor.encodeSpeed * leftFrontWheelMotor.encoderDir).ToString();
                lblVel2.Text = (-MOTDIR * rightFrontWheelMotor.encodeSpeed * rightFrontWheelMotor.encoderDir).ToString();
                lblTemp1.Text = tempM1.ToString("0.00");
                lblTemp2.Text = tempM2.ToString("0.00");
            }
           
            //stuck detect here
            if ((Math.Abs(forwardPower - 16384) > stuckPWMTH) || (Math.Abs(turnPower - 16384) > stuckPWMTH))
            {
                if (((Math.Abs(leftFrontWheelMotor.encodeSpeed) < stuckVelTH))
                || ((Math.Abs(rightFrontWheelMotor.encodeSpeed) < stuckVelTH)))
                {
                    stuckAcc++;
                }
                else
                {
                    stuckAcc--;
                    if (stuckAcc < 0) stuckAcc = 0;
                }
            }
            else
            {
                stuckAcc--;
                if (stuckAcc < 0) stuckAcc = 0;
            }
            if (stuckAcc >= stuckAccTH)
            {
                stuckAcc = stuckAccTH;
                pictureBoxStuckDetect.BackColor = Color.Red;
                if (checkBoxMotorProtect.Checked)
                {
                    
                    protectMotorStuck = true;
                }
            }
            else if (stuckAcc <= (stuckAccTH - 50))
            {
                pictureBoxStuckDetect.BackColor = Color.Green;
                if (checkBoxMotorProtect.Checked)
                {
                    protectMotorStuck = false ;
                }
            }




            //here we record encoder with GPS/IMU data

            string recTemp = "#M,";
            if (startRecord)
            {
                recTemp += leftFrontWheelMotor.encoderPos.ToString() + "," + rightFrontWheelMotor.encoderPos.ToString();
                SW.WriteLine(recTemp);
                recordCnt++;
                if (recordCnt > MAXFILELEN)
                {
                    recordCnt = 0;
                    SW.Close();
                    //open next file

                    SW = File.CreateText(fileNme + fileCnt.ToString() + ".txt");
                    fileCnt++;

                }
            }

        }

        private double Trans2Temperature(double adValue)
        {
            //for new temperature sensor
            double tempM = 0;
            double k = (adValue / FULLAD);
            double resValue = 0;
            if (k != 1)
            {
                resValue = 10000 * k / (1 - k);      //AD value to resistor
            }
            else
            {
                resValue = resTable[0];
            }


            int index = -1;
            if (resValue >= resTable[0])       //too lower
            {
                tempM = -20;
            }
            else if (resValue <= resTable[24])
            {
                tempM = 100;
            }
            else
            {
                for (int i = 0; i < 24; i++)
                {
                    if ((resValue <= resTable[i]) && (resValue >= resTable[i + 1]))
                    {
                        index = i;
                        break;
                    }
                }
                if (index >= 0)
                {
                    tempM = tempTable[index] + (resValue - resTable[index]) / (resTable[index + 1] - resTable[index]) * (tempTable[index + 1] - tempTable[index]);
                }
                else
                {
                    tempM = 0;
                }

            }

            return tempM;
        }

        #endregion

        #region Joystick control
        private bool InitDirectInput()
        {
            // Enumerate joysticks in the system.
            int i = 0;
            foreach (DeviceInstance instance in Manager.GetDevices(DeviceClass.GameControl, EnumDevicesFlags.AttachedOnly))
            {
                // Create the device.  Just pick the first one
                applicationDevice[i] = new Device(instance.InstanceGuid);
                i++;
            }

            if ( (null == applicationDevice[0]))
            {
                // MessageBox.Show("Unable to create a joystick device.", "No joystick found");
                return false;
            }

            i = 0;
            // Set the data format to the c_dfDIJoystick pre-defined format.
            applicationDevice[i].SetDataFormat(DeviceDataFormat.Joystick);
            // Set the cooperative level for the device.
            applicationDevice[i].SetCooperativeLevel(this, CooperativeLevelFlags.Exclusive | CooperativeLevelFlags.Foreground);
            // Enumerate all the objects on the device.
            foreach (DeviceObjectInstance d in applicationDevice[i].Objects)
            {
                // For axes that are returned, set the DIPROP_RANGE property for the
                // enumerated axis in order to scale min/max values.

                if ((0 != (d.ObjectId & (int)DeviceObjectTypeFlags.Axis)))
                {
                    // Set the range for the axis.
                    applicationDevice[i].Properties.SetRange(ParameterHow.ById, d.ObjectId, new InputRange(0, +10000));
                }
            }
            
            
            return true;
        }

        private void GetData()
        {
            //first device for Arm control, second for wheel control
            // Make sure there is a valid device.
            
                if (null == applicationDevice[0])
                    return;
                try
                {
                    // Poll the device for info.
                    applicationDevice[0].Poll();
                }
                catch (InputException inputex)
                {
                    if ((inputex is NotAcquiredException) || (inputex is InputLostException))
                    {
                        // Check to see if either the app
                        // needs to acquire the device, or
                        // if the app lost the device to another
                        // process.
                        try
                        {
                            // Acquire the device.
                            applicationDevice[0].Acquire();
                        }
                        catch (InputException)
                        {
                            // Failed to acquire the device.
                            // This could be because the app
                            // doesn't have focus.
                            myJaguar.DcMotorPwmNonTimeCtrAll(INIPWM, INIPWM, NOCONTROL, INIPWM , INIPWM , NOCONTROL);
                            return;
                        }
                    }

                } //catch(InputException inputex)

                // Get the state of the device.
                try { joyState[0] = applicationDevice[0].CurrentJoystickState; }
                // Catch any exceptions. None will be handled here, 
                // any device re-aquisition will be handled above.  
                catch (InputException)
                {
                    myJaguar.DcMotorPwmNonTimeCtrAll(INIPWM, INIPWM, NOCONTROL, INIPWM , INIPWM , NOCONTROL);
                    return;
                }
                if (blnJoyStick)
                {
                    
                    int[] pov = joyState[0].GetPointOfView();
                    if (pov[0] == 9000)
                    {
                        // show axis camera in big panel
                        SetCameraSize(true);
                    }
                    else if (pov[0] == 27000)
                    {
                        //axis camera original size
                        SetCameraSize(false);
                    }
                    else if (pov[0] == 18000)
                    {
                        if (lightCtrl)
                        {
                            lightCtrl = false;
                            SetFrontLight();
                        }
                    }
                    else if (pov[0] == 0)
                    {
                        SetIniPos();
                    }
                    else
                    {
                        lightCtrl = true;
                        
                    }



                    if ((!protectMotorTemp) && (!protectMotorStuck ))
                    {
                        UpdateWheelControl();
                    }
                    else
                    {
                        //stop first
                        myJaguar.DcMotorPwmNonTimeCtrAll(INIPWM, INIPWM, NOCONTROL, INIPWM, INIPWM, NOCONTROL);
                    }
                   
                    
                    if (!forceStop) UpdateArmControl();
                }



            
        }

        private void UpdateWheelControl()
        {
            
            int x = (joyState[0].Y - 5000);
            int y = -(joyState[0].Z- 5000);

            double forwardP = 0;
            double turnP = 0;
            if  (( Math.Abs(x) > 1000) || (Math.Abs(y) > 1000))
            {
                forwardP = (double)x / 5000 * 100;
                if ((x > 0) && (Math.Abs(x) > 300))
                {
                    y = -y;
                }
            
                
                turnP = (double)y / 5000 * 100;
                trackBarForwardPower.Value = (int)(forwardP);
                trackBarTurnPower.Value = (int)(turnP);


            }
            else
            {
                trackBarForwardPower.Value = 0;
                trackBarTurnPower.Value = 0;
                myJaguar.DcMotorPwmNonTimeCtrAll(NOCONTROL, NOCONTROL, NOCONTROL, INIPWM, INIPWM, NOCONTROL);
            }

            
        }
        
        private void UpdateArmControl()
        {
            //for arm control
            armCmd1 = 0;
            armCmd2 = 0;
            bool armCtrl2 = false;
            bool armCtrl1 = false;
            
            byte[] buttons = joyState[0].GetButtons();
            if (buttons != null)
            {
                if ((buttons[3] & 0x80) != 0)
                {
                    //reset arm
                    move2Arm(0, 2000);
                    preSetButton[0] = 3;
                    preSetButton[1] = 3;
                }
                else if ((buttons[2] & 0x80) != 0)
                {
                    //up 30;
                    move2Arm(30, 2000);
                    preSetButton[0] = 2;
                    preSetButton[1] = 2;
                }
                else if ((buttons[1] & 0x80) != 0)
                {
                    //flip over
                    move2Arm(300, 4000);
                    preSetButton[0] = 1;
                    preSetButton[1] = 1;

                }
                else if ((buttons[0] & 0x80) != 0)
                {
                    //up 300. flip over
                    move2Arm(30, 4000);
                    preSetButton[0] = 0;
                    preSetButton[1] = 0;
                }
                else if( (buttons[7] != 0) || (buttons [5] != 0) || (buttons [4] != 0) || (buttons[6] != 0) )
                {
                    if ((buttons[5] & 0x80) != 0)
                    {
                        //move front arm up
                        if (preSetButton[0] != 5)
                        {
                            armCmd2 = -ARM_VEL1;
                            armCtrl2 = true;
                            joyButtonCnt[0] = 0;
                            armJoy2 = true;
                        }
                        else
                        {
                            joyButtonCnt[0]++;
                            if (joyButtonCnt[0] == JOYDELAY)
                            {
                                armCtrl2 = true;
                                armCmd2 = -ARM_VEL2;
                                armJoy2 = true;
                            }
                        }
                        preSetButton[0] = 5;
                    }
                    else if ((buttons[7] & 0x80) != 0)
                    {
                        //move front arm down
                        if (preSetButton[0] != 7)
                        {
                            armCmd2 = ARM_VEL1;
                            armCtrl2 = true;
                            joyButtonCnt[0] = 0;
                            armJoy2 = true;
                        }
                        else
                        {
                            joyButtonCnt[0]++;
                            if (joyButtonCnt[0] == JOYDELAY)
                            {
                                armCtrl2 = true;
                                armCmd2 = ARM_VEL2;
                                armJoy2 = true;
                            }
                        }
                        preSetButton[0] = 7;
                    }
                    else if( ((buttons [7]& 0x80) == 0) && ((buttons [5]& 0x80) == 0) )
                    {
                        //releaase button[7] or button[5]
                        if (armJoy2)
                        {
                            armCtrl2 = false;
                            armJoy2 = false;
                            armCmd2 = armMotor[1].encoderPos;
                            if (preSetButton[0] == 5)
                            {
                                armCmd2 = armCmd2 - ARM_DELAY_CNT;
                            }
                            else if (preSetButton[0] == 7)
                            {
                                armCmd2 = armCmd2 + ARM_DELAY_CNT;
                            }
                            if (armCmd2 > 32767) armCmd2 = armCmd2 - 32767;
                            if (armCmd2 < 0) armCmd2 = 32767 + armCmd2;
                            myJaguar.DcMotorPositionNonTimeCtr(armChannel2, (short)armCmd2);
                            
                        }
                        preSetButton[0] = -1;
                    }

                    if (((buttons[4] & 0x80) != 0))
                    {

                        //move rear arm down
                        if (preSetButton[1] != 4)
                        {

                            armCmd1 = -ARM_VEL1;
                            armCtrl1 = true;
                            joyButtonCnt[1] = 0;
                            armJoy1 = true;
                        }
                        else
                        {
                            joyButtonCnt[1]++;
                            if (joyButtonCnt[1] == JOYDELAY)
                            {
                                armCtrl1 = true;
                                armCmd1 = -ARM_VEL2;
                                armJoy1 = true;
                            }
                        }
                        preSetButton[1] = 4;
                    }
                    else if ((buttons[6] & 0x80) != 0)
                    {
                        //move arm down
                        if (preSetButton[1] != 6)
                        {
                            armCmd1 = ARM_VEL1;
                            armCtrl1 = true;
                            joyButtonCnt[1] = 0;
                            armJoy1 = true;
                        }
                        else
                        {
                            joyButtonCnt[1]++;
                            if (joyButtonCnt[1] == JOYDELAY)
                            {
                                armCtrl1 = true;
                                armCmd1 = ARM_VEL2;
                                armJoy1 = true;
                            }
                        }
                        preSetButton[1] = 6;
                    }
                    else if (((buttons[6] & 0x80) == 0) && ((buttons[4] & 0x80) == 0))
                    {
                       
                        if (armJoy1)
                        {
                            armCtrl1 = false;
                            armJoy1 = false;
                            armCmd1 = armMotor[0].encoderPos;
                            if (preSetButton[1] == 4)
                            {
                                armCmd1 = armCmd1 - ARM_DELAY_CNT;
                            }
                            else if (preSetButton[1] == 6)
                            {
                                armCmd1 = armCmd1 + ARM_DELAY_CNT;
                            }
                            if (armCmd1 > 32767) armCmd1 = armCmd1 - 32767;
                            if (armCmd1 < 0) armCmd1 = 32767 + armCmd1;
                            myJaguar.DcMotorPositionNonTimeCtr(armChannel1, (short)armCmd1);
                            
                            
                        }
                        preSetButton[1] = -1;
                    }
                }
                else
                {
                    
                    //stop arm, and hold current position
                    if (armJoy2)
                    {
                        armCtrl2 = false;
                        armJoy2 = false;

                        armCmd2 = armMotor[1].encoderPos;
                        if (preSetButton[0] == 5)
                        {
                            armCmd2 = armCmd2 - ARM_DELAY_CNT;
                        }
                        else if (preSetButton[0] == 7)
                        {
                            armCmd2 = armCmd2 + ARM_DELAY_CNT;
                        }
                        if (armCmd2 > 32767) armCmd2 = armCmd2 - 32767;
                        if (armCmd2 < 0) armCmd2 = 32767 + armCmd2;

                        myJaguar.DcMotorPositionNonTimeCtr(armChannel2, (short)armCmd2);

                    }

                    if (armJoy1)
                    {

                        armCtrl1 = false;
                        armJoy1 = false;
                        armCmd1 = armMotor[0].encoderPos;
                        if (preSetButton[1] == 4)
                        {
                            armCmd1 = armCmd1 - ARM_DELAY_CNT;
                        }
                        else if (preSetButton[1] == 6)
                        {
                            armCmd1 = armCmd1 + ARM_DELAY_CNT;
                        }
                        if (armCmd1 > 32767) armCmd1 = armCmd1 - 32767;
                        if (armCmd1 < 0) armCmd1 = 32767 + armCmd1;
                        myJaguar.DcMotorPositionNonTimeCtr(armChannel1, (short)armCmd1);
                        
                    }
                    preSetButton[0] = -1;      //no button
                    preSetButton[1] = -1;      //no button
                    joyButtonCnt[0] = 0;
                    joyButtonCnt[1] = 0;
                }
            }
          
            if (armCtrl2 || armCtrl1)
            {
                if ((armCtrl2) && (!armCtrl1 ))
                {
                    myJaguar.DcMotorVelocityNonTimeCtr(armChannel2, (short)armCmd2);
                    armCmd1 = -2;
                }
                else if ((!armCtrl2) && (armCtrl1))
                {
                    myJaguar.DcMotorVelocityNonTimeCtr(armChannel1, (short)armCmd1);
                    armCmd2 = -2;
                }
                else
                {
                    //myJaguar.DcMotorVelocityTimeCtrAll((short)armCmd1, (short)armCmd2, NOCONTROL, NOCONTROL, NOCONTROL, NOCONTROL, (short)exeTime);
                    myJaguar.DcMotorVelocityNonTimeCtrAll((short)armCmd1, (short)armCmd2, NOCONTROL, NOCONTROL, NOCONTROL, NOCONTROL);
                    
                }
                
                armCtrl2 = false;
                armCtrl1 = false;
            }
        }
        
        private void tmrJoyStickPoll_Tick(object sender, EventArgs e)
        {
            GetData();
        }

        private void trackBarForwardPower_ValueChanged(object sender, EventArgs e)
        {
            int forwardPWM = 0;
            if ((!protectMotorTemp) &&(!protectMotorStuck))
            {

                if (trackBarForwardPower.Value != 0)
                {
                    forwardPWM = MOTDIR * Math.Sign(trackBarForwardPower.Value) * ((int)((double)Math.Abs(trackBarForwardPower.Value) / 100 * (MAXPWM - MINPWM) + MINPWM));
                    forwardPWM = forwardPWM + INIPWM;
                }
                else
                    forwardPWM = INIPWM;

                if (forwardPWM > 32767) forwardPWM = 32767;
                if (forwardPWM < 0) forwardPWM = 0;
            }
            else
            {
                forwardPWM = INIPWM;
            }
            myJaguar.DcMotorPwmNonTimeCtr(LEFTWHEELCHANNEL, (short)forwardPWM);
        }

        private void trackBarTurnPower_ValueChanged(object sender, EventArgs e)
        {
            int turnPWM = 0;
            double  turnPower = 0;
            if ((!protectMotorTemp) && (!protectMotorStuck ))
            {
                if (trackBarTurnPower.Value != 0)
                {
                    if (checkBoxTurnK.Checked)
                        turnPower = trackBarTurnPower.Value * jaguarSetting.TurnK;
                    else
                        turnPower = trackBarTurnPower.Value;
                    turnPWM = -MOTDIR * Math.Sign(trackBarTurnPower.Value) * ((int)((double)Math.Abs(turnPower) / 100 * (MAXPWM - MINPWM) + MINPWM));
                    turnPWM = turnPWM + INIPWM;
                }
                else
                {
                    turnPWM = INIPWM;
                }

                if (turnPWM > 32767) turnPWM = 32767;
                if (turnPWM < 0) turnPWM = 0;
            }
            else
            {
                turnPWM = INIPWM;
            }
            myJaguar.DcMotorPwmNonTimeCtr(RIGHTWHEELCHANNEL, (short)turnPWM);
        }


        private void move2Arm(double angle, int time)
        {
          
            double temp = -angle / 360 * ARM_CIRCLE_CNT;
            armCmd1 = (int)(armResetPos[0] + temp);
            armCmd1 = (armCmd1 > 32767 ? armCmd1 - 32767 : armCmd1);
            armCmd1 = (armCmd1 < 0 ? 32767 + armCmd1 : armCmd1);
            armCmd2 = (int)(armResetPos[1] + temp);
            armCmd2 = (armCmd2 > 32767 ? armCmd2 - 32767 : armCmd2);
            armCmd2 = (armCmd2 < 0 ? 32767 + armCmd2 : armCmd2);
            armJoy2 = false;
            armJoy1 = false;
            myJaguar.DcMotorPositionTimeCtrAll((short)armCmd1 ,(short)armCmd2, NOCONTROL, NOCONTROL, NOCONTROL, NOCONTROL, (short)time);
        }


        private void SetFrontLight()
        {
            if (lightOn)
            {
                expandIO = expandIO & 0x7f;     //bit 7 turn off lights
                lightOn = false;
                myJaguar.SetCustomDOUT((short)expandIO);
            }
            else
            {
                expandIO = expandIO | 0x80;     //bit 7 turn on lights
                lightOn = true;
                myJaguar.SetCustomDOUT((short)expandIO);
            }
        }

        private void SetIniPos()
        {
            preArmPos[0] = 0;
            preArmPos[1] = 0;
            preArmEncoder[0] = armEncoder[0];
            armResetPos[0] = armEncoder[0];
            armResetPos[1] = armEncoder[1];
            firstEncoderData = true;
            armPosIndicator.SetArmPositionIndicatorIni(0,0);
        }

        private void SetCameraSize(bool setting)
        {
            if (setting)
            {
                //big size
                panel1.Top = 3;
                panel1.Left = 4;
                panel1.Width = 1022;
                panel1.Height = 730;
                panel1.BringToFront();
                myAMC.Width = 1022;
                myAMC.Height = 730;
                myAMC.BringToFront();
            }
            else
            {
                //original size
                panel1.Top = 3;
                panel1.Left = 545;
                panel1.Width = 462;
                panel1.Height = 378;
                panel1.BringToFront();
                myAMC.Width = 460;
                myAMC.Height = 372;
                myAMC.BringToFront();
            }
        }

        #endregion

        private void btnLaserScan_Click(object sender, EventArgs e)
        {
            if (btnLaserScan.Text == "LaserScan")
            {
                btnLaserScan.Text = "Camera";
                myAMC.SendToBack();
                pictureBoxSensor.BringToFront();
                pictureBoxLaser.BringToFront();
                btnScan.BringToFront();
                btnTurnOn.BringToFront();
              
            }
            else
            {
                btnLaserScan.Text = "LaserScan";
                myAMC.BringToFront();
                pictureBoxSensor.SendToBack();
                pictureBoxLaser.SendToBack();
                btnScan.SendToBack();
                btnTurnOn.SendToBack();
            }
        }

        private void tmrDisplay_Tick(object sender, EventArgs e)
        {
            lblCOG.Text = gpsRecord.cog.ToString();
            lblVOG.Text = gpsRecord.vog .ToString("0.00");
            lblLat .Text = gpsRecord .latitude.ToString ();
            lblLong.Text = gpsRecord .longitude.ToString ();

            if (gpsRecord.qi == 0)
            {
                lblGPSQI.Text = "Fix not avaiable";
            }
            else if (gpsRecord.qi == 1)
            {
                lblGPSQI.Text = "Non DGPS fix avaiable";
            }
            else if (gpsRecord.qi == 2)
            {
                lblGPSQI.Text = "DGPS fix avaiable";
            }
            else if (gpsRecord.qi == 6)
            {
                lblGPSQI.Text = "Estimate";
            }
            else
            {
                lblGPSQI.Text = "Fix not avaiable";
            }


            int dataLen = 0;
            if (drawEndPoint != drawStartPoint)
            {
                if (drawEndPoint > drawStartPoint)
                    dataLen = drawEndPoint - drawStartPoint;
                else
                    dataLen = DrawDataLen;

                DrawDataPic(pictureBoxAccelX, "Accel_X", Color.Crimson, draw_AccelX, drawStartPoint, drawEndPoint, dataLen, 512);
                DrawDataPic(pictureBoxAccelY, "Accel_Y", Color.Fuchsia, draw_AccelY, drawStartPoint, drawEndPoint, dataLen, 512);
                DrawDataPic(pictureBoxAccelZ, "Accel_Z", Color.DeepSkyBlue, draw_AccelZ, drawStartPoint, drawEndPoint, dataLen, 512);

                DrawDataPic(pictureBoxGyroX, "Pitch", Color.Lime, draw_GyroY, drawStartPoint, drawEndPoint, dataLen, 16384);    //actual the data is 2^16 = 65536
                DrawDataPic(pictureBoxGyroY, "Roll", Color.DeepPink, draw_GyroX, drawStartPoint, drawEndPoint, dataLen, 16384);
                DrawDataPic(pictureBoxGyroZ, "Yaw", Color.Blue, draw_GyroZ, drawStartPoint, drawEndPoint, dataLen, 16384);
            }
        }

        private void DrawDataPic(PictureBox picCtrl, string dataName, Color penColor, double[] data, int startPos, int endPos, int length, int maxData)
        {
            Bitmap bmp = new Bitmap(picCtrl.Width, picCtrl.Height);
            Graphics g = Graphics.FromImage(bmp);

            int width = bmp.Width - 4;
            int height = bmp.Height - 4;
            Pen drawAxisPen = new Pen(Color.Black);
            Pen drawDataPen = new Pen(penColor);

            g.Clear(System.Drawing.Color.White);
            g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;
            GraphicsPath path = new GraphicsPath();

            g.DrawLine(drawAxisPen, 0, height / 2, width, height / 2);
            int x = 0;
            int y = 0;
            int lastX = 0;
            int lastY = height / 2;
            double scaleY = ((double)height / 2) / maxData;
            double scaleX = (double)width / data.Length;
            int dataPos = 0;

            for (int i = 0; i < length; i++)
            {
                x = (int)(i * scaleX);

                dataPos = i + startPos;
                if (dataPos >= length)
                {
                    dataPos = dataPos - length;
                }
                y = (int)(data[dataPos] * scaleY) + height / 2;
                g.DrawLine(drawDataPen, lastX, lastY, x, y);
                lastX = x;
                lastY = y;
            }

            String drawString = dataName;
            // Create font and brush.
            Font drawFont = new Font("Arial", 8, FontStyle.Bold);
            SolidBrush drawBrush = new SolidBrush(Color.Red);
            // Create point for upper-left corner of drawing.
            float fontPosX = 3.0F;
            float fontPosY = 3.0F;
            // Set format of string.
            StringFormat drawFormat = new StringFormat();
            //drawFormat.FormatFlags = StringFormatFlags.DirectionVertical;

            // Draw string to screen.
            g.DrawString(drawString, drawFont, drawBrush, fontPosX, fontPosY, drawFormat);


            picCtrl.Image = bmp;
        }

        private void btnRecord_Click(object sender, EventArgs e)
        {
            if (btnRecord.Text == "Record")
            {
                btnRecord.Text = "StopRec";
                //fileCnt = 0; //don't reset the fileCnt
                recordCnt = 0;
                startRecord = true;
                SW = File.CreateText(fileNme + fileCnt.ToString() + ".txt");
                fileCnt++;
            }
            else
            {
                btnRecord.Text = "Record";
                startRecord = false;
                if (SW != null)
                    SW.Close();
            }
        }

        private void btnScan_Click(object sender, EventArgs e)
        {
            if (clientSocketLaser != null)
            {
                sendCommandLaser(SCANCOMMAND );
                
            }
            firstData = true;
        }

        private void btnDisArm_Click(object sender, EventArgs e)
        {
            if (btnDisArm.Text == "Dis Arm")
            {
                btnDisArm.BackColor = Color.Green;
                myJaguar.DcMotorPwmNonTimeCtrAll((short)INIPWM, (short)INIPWM, (short)INIPWM, (short)INIPWM, (short)INIPWM, (short)INIPWM);
                btnDisArm.Text = "En Arm";
                forceStop = true;
            }
            else
            {
                btnDisArm.BackColor = Color.Red;
                btnDisArm.Text = "Dis Arm";
                forceStop = false;

            }


        }

        private void checkBoxMotorProtect_CheckedChanged(object sender, EventArgs e)
        {
            protectMotorStuck = false;
            protectMotorTemp = false;
        }

        private void btnTurnOn_Click(object sender, EventArgs e)
        {
            if (clientSocketLaser != null)
            {
                sendCommandLaser("BM");
                btnScan.Enabled = true;
            }
            myJaguar.DcMotorPwmNonTimeCtr(0, 30000);
        }

    }
}

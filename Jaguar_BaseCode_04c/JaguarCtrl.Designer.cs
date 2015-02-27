namespace DrRobot.JaguarControl
{
    partial class JaguarCtrl
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(JaguarCtrl));
            this.groupBoxGPSIMU = new System.Windows.Forms.GroupBox();
            this.lblBatVol = new System.Windows.Forms.Label();
            this.realJaguar = new AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel();
            this.pictureBoxIMUGPS = new System.Windows.Forms.PictureBox();
            this.label2 = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.textBoxGPSRCV = new System.Windows.Forms.TextBox();
            this.textBoxRCV = new System.Windows.Forms.TextBox();
            this.lblGPSQI = new System.Windows.Forms.Label();
            this.label46 = new System.Windows.Forms.Label();
            this.lblLat = new System.Windows.Forms.Label();
            this.label15 = new System.Windows.Forms.Label();
            this.label17 = new System.Windows.Forms.Label();
            this.lblLong = new System.Windows.Forms.Label();
            this.label33 = new System.Windows.Forms.Label();
            this.lblVOG = new System.Windows.Forms.Label();
            this.label18 = new System.Windows.Forms.Label();
            this.lblCOG = new System.Windows.Forms.Label();
            this.pictureBoxGyroZ = new System.Windows.Forms.PictureBox();
            this.pictureBoxGyroY = new System.Windows.Forms.PictureBox();
            this.pictureBoxGyroX = new System.Windows.Forms.PictureBox();
            this.label3 = new System.Windows.Forms.Label();
            this.pictureBoxAccelZ = new System.Windows.Forms.PictureBox();
            this.pictureBoxAccelY = new System.Windows.Forms.PictureBox();
            this.pictureBoxAccelX = new System.Windows.Forms.PictureBox();
            this.btnRecord = new System.Windows.Forms.Button();
            this.panel1 = new System.Windows.Forms.Panel();
            this.myAMC = new AxAXISMEDIACONTROLLib.AxAxisMediaControl();
            this.btnScan = new System.Windows.Forms.Button();
            this.pictureBoxLaser = new System.Windows.Forms.PictureBox();
            this.pictureBoxSensor = new System.Windows.Forms.PictureBox();
            this.btnTurnOn = new System.Windows.Forms.Button();
            this.lblMot3 = new System.Windows.Forms.Label();
            this.lblMot2 = new System.Windows.Forms.Label();
            this.lblMot1 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.txtStartLong = new System.Windows.Forms.TextBox();
            this.txtStartLat = new System.Windows.Forms.TextBox();
            this.label11 = new System.Windows.Forms.Label();
            this.label12 = new System.Windows.Forms.Label();
            this.btnSetStartPoint = new System.Windows.Forms.Button();
            this.trackBarZoom = new System.Windows.Forms.TrackBar();
            this.lblEncoderPos1 = new System.Windows.Forms.Label();
            this.lblEncoderPos2 = new System.Windows.Forms.Label();
            this.lblEncoderPos4 = new System.Windows.Forms.Label();
            this.lblVel4 = new System.Windows.Forms.Label();
            this.lblVel2 = new System.Windows.Forms.Label();
            this.lblVel1 = new System.Windows.Forms.Label();
            this.lblTemp4 = new System.Windows.Forms.Label();
            this.lblTemp2 = new System.Windows.Forms.Label();
            this.lblTemp1 = new System.Windows.Forms.Label();
            this.tmrJoyStickPoll = new System.Windows.Forms.Timer(this.components);
            this.trackBarTurnPower = new System.Windows.Forms.TrackBar();
            this.trackBarForwardPower = new System.Windows.Forms.TrackBar();
            this.btnLaserScan = new System.Windows.Forms.Button();
            this.imageList1 = new System.Windows.Forms.ImageList(this.components);
            this.tmrDisplay = new System.Windows.Forms.Timer(this.components);
            this.tmrDrawing = new System.Windows.Forms.Timer(this.components);
            this.imageList2 = new System.Windows.Forms.ImageList(this.components);
            this.label4 = new System.Windows.Forms.Label();
            this.btnDisArm = new System.Windows.Forms.Button();
            this.checkBoxMotorProtect = new System.Windows.Forms.CheckBox();
            this.label5 = new System.Windows.Forms.Label();
            this.pictureBoxStuckDetect = new System.Windows.Forms.PictureBox();
            this.checkBoxHardware = new System.Windows.Forms.CheckBox();
            this.buttonReset = new System.Windows.Forms.Button();
            this.pictureBox2 = new System.Windows.Forms.PictureBox();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.buttonStop = new System.Windows.Forms.Button();
            this.panelGE = new System.Windows.Forms.Panel();
            this.label8 = new System.Windows.Forms.Label();
            this.label13 = new System.Windows.Forms.Label();
            this.label14 = new System.Windows.Forms.Label();
            this.label16 = new System.Windows.Forms.Label();
            this.desiredT = new System.Windows.Forms.Label();
            this.txtStartTheta = new System.Windows.Forms.TextBox();
            this.txtNumParticles = new System.Windows.Forms.TextBox();
            this.label19 = new System.Windows.Forms.Label();
            this.checkBoxKnownStart = new System.Windows.Forms.CheckBox();
            this.groupBoxGPSIMU.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.realJaguar)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxIMUGPS)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxGyroZ)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxGyroY)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxGyroX)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxAccelZ)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxAccelY)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxAccelX)).BeginInit();
            this.panel1.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.myAMC)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxLaser)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxSensor)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBarZoom)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBarTurnPower)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBarForwardPower)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxStuckDetect)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).BeginInit();
            this.SuspendLayout();
            // 
            // groupBoxGPSIMU
            // 
            this.groupBoxGPSIMU.Controls.Add(this.lblBatVol);
            this.groupBoxGPSIMU.Controls.Add(this.realJaguar);
            this.groupBoxGPSIMU.Controls.Add(this.pictureBoxIMUGPS);
            this.groupBoxGPSIMU.Controls.Add(this.label2);
            this.groupBoxGPSIMU.Controls.Add(this.label1);
            this.groupBoxGPSIMU.Controls.Add(this.textBoxGPSRCV);
            this.groupBoxGPSIMU.Controls.Add(this.textBoxRCV);
            this.groupBoxGPSIMU.Controls.Add(this.lblGPSQI);
            this.groupBoxGPSIMU.Controls.Add(this.label46);
            this.groupBoxGPSIMU.Controls.Add(this.lblLat);
            this.groupBoxGPSIMU.Controls.Add(this.label15);
            this.groupBoxGPSIMU.Controls.Add(this.label17);
            this.groupBoxGPSIMU.Controls.Add(this.lblLong);
            this.groupBoxGPSIMU.Controls.Add(this.label33);
            this.groupBoxGPSIMU.Controls.Add(this.lblVOG);
            this.groupBoxGPSIMU.Controls.Add(this.label18);
            this.groupBoxGPSIMU.Controls.Add(this.lblCOG);
            this.groupBoxGPSIMU.Controls.Add(this.pictureBoxGyroZ);
            this.groupBoxGPSIMU.Controls.Add(this.pictureBoxGyroY);
            this.groupBoxGPSIMU.Controls.Add(this.pictureBoxGyroX);
            this.groupBoxGPSIMU.Controls.Add(this.label3);
            this.groupBoxGPSIMU.Controls.Add(this.pictureBoxAccelZ);
            this.groupBoxGPSIMU.Controls.Add(this.pictureBoxAccelY);
            this.groupBoxGPSIMU.Controls.Add(this.pictureBoxAccelX);
            this.groupBoxGPSIMU.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.groupBoxGPSIMU.Location = new System.Drawing.Point(3, 498);
            this.groupBoxGPSIMU.Name = "groupBoxGPSIMU";
            this.groupBoxGPSIMU.Size = new System.Drawing.Size(1008, 192);
            this.groupBoxGPSIMU.TabIndex = 0;
            this.groupBoxGPSIMU.TabStop = false;
            this.groupBoxGPSIMU.Text = "GPS && IMU";
            // 
            // lblBatVol
            // 
            this.lblBatVol.BackColor = System.Drawing.Color.White;
            this.lblBatVol.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.lblBatVol.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblBatVol.Location = new System.Drawing.Point(231, 76);
            this.lblBatVol.Name = "lblBatVol";
            this.lblBatVol.Size = new System.Drawing.Size(33, 17);
            this.lblBatVol.TabIndex = 43;
            this.lblBatVol.Text = "0";
            this.lblBatVol.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // realJaguar
            // 
            this.realJaguar.Enabled = true;
            this.realJaguar.Location = new System.Drawing.Point(173, 21);
            this.realJaguar.Name = "realJaguar";
            this.realJaguar.OcxState = ((System.Windows.Forms.AxHost.State)(resources.GetObject("realJaguar.OcxState")));
            this.realJaguar.Size = new System.Drawing.Size(18, 30);
            this.realJaguar.TabIndex = 85;
            this.realJaguar.Visible = false;
            this.realJaguar.StandardSensorEvent += new System.EventHandler(this.realJaguar_StandardSensorEvent);
            this.realJaguar.MotorSensorEvent += new System.EventHandler(this.realJaguar_MotorSensorEvent);
            this.realJaguar.CustomSensorEvent += new System.EventHandler(this.realJaguar_CustomSensorEvent);
            // 
            // pictureBoxIMUGPS
            // 
            this.pictureBoxIMUGPS.Location = new System.Drawing.Point(211, 21);
            this.pictureBoxIMUGPS.Name = "pictureBoxIMUGPS";
            this.pictureBoxIMUGPS.Size = new System.Drawing.Size(38, 32);
            this.pictureBoxIMUGPS.TabIndex = 86;
            this.pictureBoxIMUGPS.TabStop = false;
            // 
            // label2
            // 
            this.label2.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label2.ForeColor = System.Drawing.Color.Black;
            this.label2.Location = new System.Drawing.Point(5, 150);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(124, 15);
            this.label2.TabIndex = 84;
            this.label2.Text = "GPS Raw Data:";
            this.label2.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // label1
            // 
            this.label1.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.ForeColor = System.Drawing.Color.Black;
            this.label1.Location = new System.Drawing.Point(6, 114);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(123, 14);
            this.label1.TabIndex = 83;
            this.label1.Text = "IMU Raw Data:";
            this.label1.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // textBoxGPSRCV
            // 
            this.textBoxGPSRCV.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxGPSRCV.Location = new System.Drawing.Point(9, 166);
            this.textBoxGPSRCV.Name = "textBoxGPSRCV";
            this.textBoxGPSRCV.Size = new System.Drawing.Size(259, 20);
            this.textBoxGPSRCV.TabIndex = 82;
            // 
            // textBoxRCV
            // 
            this.textBoxRCV.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.textBoxRCV.Location = new System.Drawing.Point(9, 129);
            this.textBoxRCV.Name = "textBoxRCV";
            this.textBoxRCV.Size = new System.Drawing.Size(259, 20);
            this.textBoxRCV.TabIndex = 81;
            // 
            // lblGPSQI
            // 
            this.lblGPSQI.BackColor = System.Drawing.Color.White;
            this.lblGPSQI.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.lblGPSQI.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblGPSQI.Location = new System.Drawing.Point(89, 96);
            this.lblGPSQI.Name = "lblGPSQI";
            this.lblGPSQI.Size = new System.Drawing.Size(176, 17);
            this.lblGPSQI.TabIndex = 80;
            this.lblGPSQI.Text = "No fix avaiable";
            this.lblGPSQI.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label46
            // 
            this.label46.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label46.ForeColor = System.Drawing.Color.Black;
            this.label46.Location = new System.Drawing.Point(4, 95);
            this.label46.Name = "label46";
            this.label46.Size = new System.Drawing.Size(82, 19);
            this.label46.TabIndex = 78;
            this.label46.Text = "GPS Quality:";
            this.label46.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // lblLat
            // 
            this.lblLat.BackColor = System.Drawing.Color.White;
            this.lblLat.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.lblLat.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblLat.Location = new System.Drawing.Point(89, 16);
            this.lblLat.Name = "lblLat";
            this.lblLat.Size = new System.Drawing.Size(78, 17);
            this.lblLat.TabIndex = 71;
            this.lblLat.Text = "0";
            this.lblLat.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label15
            // 
            this.label15.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label15.ForeColor = System.Drawing.Color.Black;
            this.label15.Location = new System.Drawing.Point(31, 15);
            this.label15.Name = "label15";
            this.label15.Size = new System.Drawing.Size(55, 18);
            this.label15.TabIndex = 70;
            this.label15.Text = "Latitude:";
            this.label15.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // label17
            // 
            this.label17.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label17.ForeColor = System.Drawing.Color.Black;
            this.label17.Location = new System.Drawing.Point(17, 35);
            this.label17.Name = "label17";
            this.label17.Size = new System.Drawing.Size(69, 18);
            this.label17.TabIndex = 72;
            this.label17.Text = "Longtitude:";
            this.label17.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // lblLong
            // 
            this.lblLong.BackColor = System.Drawing.Color.White;
            this.lblLong.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.lblLong.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblLong.Location = new System.Drawing.Point(89, 36);
            this.lblLong.Name = "lblLong";
            this.lblLong.Size = new System.Drawing.Size(78, 17);
            this.lblLong.TabIndex = 73;
            this.lblLong.Text = "0";
            this.lblLong.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label33
            // 
            this.label33.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label33.ForeColor = System.Drawing.Color.Black;
            this.label33.Location = new System.Drawing.Point(23, 75);
            this.label33.Name = "label33";
            this.label33.Size = new System.Drawing.Size(63, 18);
            this.label33.TabIndex = 77;
            this.label33.Text = "GPS VOG:";
            this.label33.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // lblVOG
            // 
            this.lblVOG.BackColor = System.Drawing.Color.White;
            this.lblVOG.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.lblVOG.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblVOG.Location = new System.Drawing.Point(89, 76);
            this.lblVOG.Name = "lblVOG";
            this.lblVOG.Size = new System.Drawing.Size(78, 17);
            this.lblVOG.TabIndex = 76;
            this.lblVOG.Text = "0";
            this.lblVOG.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label18
            // 
            this.label18.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label18.ForeColor = System.Drawing.Color.Black;
            this.label18.Location = new System.Drawing.Point(20, 55);
            this.label18.Name = "label18";
            this.label18.Size = new System.Drawing.Size(66, 18);
            this.label18.TabIndex = 74;
            this.label18.Text = "GPS COG:";
            this.label18.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // lblCOG
            // 
            this.lblCOG.BackColor = System.Drawing.Color.White;
            this.lblCOG.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.lblCOG.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblCOG.Location = new System.Drawing.Point(89, 56);
            this.lblCOG.Name = "lblCOG";
            this.lblCOG.Size = new System.Drawing.Size(78, 17);
            this.lblCOG.TabIndex = 75;
            this.lblCOG.Text = "0";
            this.lblCOG.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // pictureBoxGyroZ
            // 
            this.pictureBoxGyroZ.BackColor = System.Drawing.Color.White;
            this.pictureBoxGyroZ.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.pictureBoxGyroZ.Location = new System.Drawing.Point(766, 105);
            this.pictureBoxGyroZ.Name = "pictureBoxGyroZ";
            this.pictureBoxGyroZ.Size = new System.Drawing.Size(235, 80);
            this.pictureBoxGyroZ.TabIndex = 31;
            this.pictureBoxGyroZ.TabStop = false;
            // 
            // pictureBoxGyroY
            // 
            this.pictureBoxGyroY.BackColor = System.Drawing.Color.White;
            this.pictureBoxGyroY.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.pictureBoxGyroY.Location = new System.Drawing.Point(526, 105);
            this.pictureBoxGyroY.Name = "pictureBoxGyroY";
            this.pictureBoxGyroY.Size = new System.Drawing.Size(235, 80);
            this.pictureBoxGyroY.TabIndex = 30;
            this.pictureBoxGyroY.TabStop = false;
            // 
            // pictureBoxGyroX
            // 
            this.pictureBoxGyroX.BackColor = System.Drawing.Color.White;
            this.pictureBoxGyroX.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.pictureBoxGyroX.Location = new System.Drawing.Point(285, 105);
            this.pictureBoxGyroX.Name = "pictureBoxGyroX";
            this.pictureBoxGyroX.Size = new System.Drawing.Size(235, 80);
            this.pictureBoxGyroX.TabIndex = 29;
            this.pictureBoxGyroX.TabStop = false;
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label3.ForeColor = System.Drawing.Color.Black;
            this.label3.Location = new System.Drawing.Point(173, 77);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(61, 14);
            this.label3.TabIndex = 6;
            this.label3.Text = "Battery(V):";
            this.label3.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // pictureBoxAccelZ
            // 
            this.pictureBoxAccelZ.BackColor = System.Drawing.Color.White;
            this.pictureBoxAccelZ.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.pictureBoxAccelZ.Location = new System.Drawing.Point(766, 19);
            this.pictureBoxAccelZ.Name = "pictureBoxAccelZ";
            this.pictureBoxAccelZ.Size = new System.Drawing.Size(235, 80);
            this.pictureBoxAccelZ.TabIndex = 28;
            this.pictureBoxAccelZ.TabStop = false;
            // 
            // pictureBoxAccelY
            // 
            this.pictureBoxAccelY.BackColor = System.Drawing.Color.White;
            this.pictureBoxAccelY.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.pictureBoxAccelY.Location = new System.Drawing.Point(526, 19);
            this.pictureBoxAccelY.Name = "pictureBoxAccelY";
            this.pictureBoxAccelY.Size = new System.Drawing.Size(235, 80);
            this.pictureBoxAccelY.TabIndex = 27;
            this.pictureBoxAccelY.TabStop = false;
            // 
            // pictureBoxAccelX
            // 
            this.pictureBoxAccelX.BackColor = System.Drawing.Color.White;
            this.pictureBoxAccelX.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.pictureBoxAccelX.Location = new System.Drawing.Point(285, 19);
            this.pictureBoxAccelX.Name = "pictureBoxAccelX";
            this.pictureBoxAccelX.Size = new System.Drawing.Size(235, 80);
            this.pictureBoxAccelX.TabIndex = 26;
            this.pictureBoxAccelX.TabStop = false;
            // 
            // btnRecord
            // 
            this.btnRecord.BackColor = System.Drawing.Color.Silver;
            this.btnRecord.Location = new System.Drawing.Point(546, 477);
            this.btnRecord.Name = "btnRecord";
            this.btnRecord.Size = new System.Drawing.Size(86, 23);
            this.btnRecord.TabIndex = 30;
            this.btnRecord.Text = "Record";
            this.btnRecord.UseVisualStyleBackColor = true;
            this.btnRecord.Click += new System.EventHandler(this.btnRecord_Click);
            // 
            // panel1
            // 
            this.panel1.Controls.Add(this.myAMC);
            this.panel1.Controls.Add(this.btnScan);
            this.panel1.Controls.Add(this.pictureBoxLaser);
            this.panel1.Controls.Add(this.pictureBoxSensor);
            this.panel1.Controls.Add(this.btnTurnOn);
            this.panel1.Location = new System.Drawing.Point(545, 41);
            this.panel1.Name = "panel1";
            this.panel1.Size = new System.Drawing.Size(462, 378);
            this.panel1.TabIndex = 2;
            // 
            // myAMC
            // 
            this.myAMC.Enabled = true;
            this.myAMC.Location = new System.Drawing.Point(1, 0);
            this.myAMC.Name = "myAMC";
            this.myAMC.OcxState = ((System.Windows.Forms.AxHost.State)(resources.GetObject("myAMC.OcxState")));
            this.myAMC.Size = new System.Drawing.Size(460, 372);
            this.myAMC.TabIndex = 1;
            // 
            // btnScan
            // 
            this.btnScan.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnScan.Location = new System.Drawing.Point(242, 342);
            this.btnScan.Name = "btnScan";
            this.btnScan.Size = new System.Drawing.Size(60, 22);
            this.btnScan.TabIndex = 45;
            this.btnScan.Text = "Scan";
            this.btnScan.UseVisualStyleBackColor = true;
            this.btnScan.Click += new System.EventHandler(this.btnScan_Click);
            // 
            // pictureBoxLaser
            // 
            this.pictureBoxLaser.BackColor = System.Drawing.Color.Black;
            this.pictureBoxLaser.Location = new System.Drawing.Point(204, 316);
            this.pictureBoxLaser.Name = "pictureBoxLaser";
            this.pictureBoxLaser.Size = new System.Drawing.Size(36, 33);
            this.pictureBoxLaser.TabIndex = 3;
            this.pictureBoxLaser.TabStop = false;
            // 
            // pictureBoxSensor
            // 
            this.pictureBoxSensor.Location = new System.Drawing.Point(1, 3);
            this.pictureBoxSensor.Name = "pictureBoxSensor";
            this.pictureBoxSensor.Size = new System.Drawing.Size(460, 372);
            this.pictureBoxSensor.TabIndex = 2;
            this.pictureBoxSensor.TabStop = false;
            // 
            // btnTurnOn
            // 
            this.btnTurnOn.Location = new System.Drawing.Point(243, 317);
            this.btnTurnOn.Name = "btnTurnOn";
            this.btnTurnOn.Size = new System.Drawing.Size(60, 22);
            this.btnTurnOn.TabIndex = 46;
            this.btnTurnOn.Text = "Turn On";
            this.btnTurnOn.UseVisualStyleBackColor = true;
            this.btnTurnOn.Click += new System.EventHandler(this.btnTurnOn_Click);
            // 
            // lblMot3
            // 
            this.lblMot3.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblMot3.ForeColor = System.Drawing.Color.Black;
            this.lblMot3.Location = new System.Drawing.Point(746, 478);
            this.lblMot3.Name = "lblMot3";
            this.lblMot3.Size = new System.Drawing.Size(36, 19);
            this.lblMot3.TabIndex = 12;
            this.lblMot3.Text = "X (m):";
            this.lblMot3.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // lblMot2
            // 
            this.lblMot2.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblMot2.ForeColor = System.Drawing.Color.Black;
            this.lblMot2.Location = new System.Drawing.Point(737, 452);
            this.lblMot2.Margin = new System.Windows.Forms.Padding(0);
            this.lblMot2.Name = "lblMot2";
            this.lblMot2.Size = new System.Drawing.Size(45, 20);
            this.lblMot2.TabIndex = 10;
            this.lblMot2.Text = "R Enc:";
            this.lblMot2.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // lblMot1
            // 
            this.lblMot1.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblMot1.ForeColor = System.Drawing.Color.Black;
            this.lblMot1.Location = new System.Drawing.Point(737, 428);
            this.lblMot1.Margin = new System.Windows.Forms.Padding(0);
            this.lblMot1.Name = "lblMot1";
            this.lblMot1.Size = new System.Drawing.Size(45, 19);
            this.lblMot1.TabIndex = 8;
            this.lblMot1.Text = "L Enc:";
            this.lblMot1.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label9.ForeColor = System.Drawing.Color.Black;
            this.label9.Location = new System.Drawing.Point(838, 429);
            this.label9.Margin = new System.Windows.Forms.Padding(0);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(34, 14);
            this.label9.TabIndex = 23;
            this.label9.Text = "L Vel:";
            this.label9.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label10.ForeColor = System.Drawing.Color.Black;
            this.label10.Location = new System.Drawing.Point(922, 430);
            this.label10.Margin = new System.Windows.Forms.Padding(0);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(40, 14);
            this.label10.TabIndex = 24;
            this.label10.Text = "LT(°C):";
            this.label10.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // txtStartLong
            // 
            this.txtStartLong.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txtStartLong.Location = new System.Drawing.Point(134, 426);
            this.txtStartLong.Name = "txtStartLong";
            this.txtStartLong.Size = new System.Drawing.Size(47, 20);
            this.txtStartLong.TabIndex = 29;
            this.txtStartLong.Text = "0";
            // 
            // txtStartLat
            // 
            this.txtStartLat.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txtStartLat.Location = new System.Drawing.Point(44, 426);
            this.txtStartLat.Name = "txtStartLat";
            this.txtStartLat.Size = new System.Drawing.Size(45, 20);
            this.txtStartLat.TabIndex = 28;
            this.txtStartLat.Text = "0";
            // 
            // label11
            // 
            this.label11.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label11.ForeColor = System.Drawing.Color.Black;
            this.label11.Location = new System.Drawing.Point(97, 427);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(59, 18);
            this.label11.TabIndex = 27;
            this.label11.Text = "Des Y:";
            this.label11.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // label12
            // 
            this.label12.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label12.ForeColor = System.Drawing.Color.Black;
            this.label12.Location = new System.Drawing.Point(7, 427);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(61, 18);
            this.label12.TabIndex = 26;
            this.label12.Text = "Des X:";
            this.label12.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // btnSetStartPoint
            // 
            this.btnSetStartPoint.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnSetStartPoint.Location = new System.Drawing.Point(394, 426);
            this.btnSetStartPoint.Name = "btnSetStartPoint";
            this.btnSetStartPoint.Size = new System.Drawing.Size(90, 25);
            this.btnSetStartPoint.TabIndex = 25;
            this.btnSetStartPoint.Text = "FlyToSetPoint";
            this.btnSetStartPoint.UseVisualStyleBackColor = true;
            this.btnSetStartPoint.Click += new System.EventHandler(this.btnSetStartPoint_Click);
            // 
            // trackBarZoom
            // 
            this.trackBarZoom.Location = new System.Drawing.Point(497, 33);
            this.trackBarZoom.Maximum = 100;
            this.trackBarZoom.Minimum = 1;
            this.trackBarZoom.Name = "trackBarZoom";
            this.trackBarZoom.Orientation = System.Windows.Forms.Orientation.Vertical;
            this.trackBarZoom.Size = new System.Drawing.Size(45, 393);
            this.trackBarZoom.TabIndex = 30;
            this.trackBarZoom.Value = 50;
            this.trackBarZoom.Scroll += new System.EventHandler(this.trackBarZoom_Scroll);
            // 
            // lblEncoderPos1
            // 
            this.lblEncoderPos1.BackColor = System.Drawing.Color.White;
            this.lblEncoderPos1.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.lblEncoderPos1.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblEncoderPos1.Location = new System.Drawing.Point(781, 427);
            this.lblEncoderPos1.Name = "lblEncoderPos1";
            this.lblEncoderPos1.Size = new System.Drawing.Size(50, 17);
            this.lblEncoderPos1.TabIndex = 31;
            this.lblEncoderPos1.Text = "0";
            this.lblEncoderPos1.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lblEncoderPos2
            // 
            this.lblEncoderPos2.BackColor = System.Drawing.Color.White;
            this.lblEncoderPos2.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.lblEncoderPos2.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblEncoderPos2.Location = new System.Drawing.Point(781, 453);
            this.lblEncoderPos2.Name = "lblEncoderPos2";
            this.lblEncoderPos2.Size = new System.Drawing.Size(50, 17);
            this.lblEncoderPos2.TabIndex = 32;
            this.lblEncoderPos2.Text = "0";
            this.lblEncoderPos2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lblEncoderPos4
            // 
            this.lblEncoderPos4.BackColor = System.Drawing.Color.White;
            this.lblEncoderPos4.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.lblEncoderPos4.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblEncoderPos4.Location = new System.Drawing.Point(781, 479);
            this.lblEncoderPos4.Name = "lblEncoderPos4";
            this.lblEncoderPos4.Size = new System.Drawing.Size(50, 17);
            this.lblEncoderPos4.TabIndex = 34;
            this.lblEncoderPos4.Text = "0";
            this.lblEncoderPos4.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lblVel4
            // 
            this.lblVel4.BackColor = System.Drawing.Color.White;
            this.lblVel4.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.lblVel4.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblVel4.Location = new System.Drawing.Point(871, 480);
            this.lblVel4.Name = "lblVel4";
            this.lblVel4.Size = new System.Drawing.Size(50, 17);
            this.lblVel4.TabIndex = 38;
            this.lblVel4.Text = "0";
            this.lblVel4.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lblVel2
            // 
            this.lblVel2.BackColor = System.Drawing.Color.White;
            this.lblVel2.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.lblVel2.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblVel2.Location = new System.Drawing.Point(871, 453);
            this.lblVel2.Name = "lblVel2";
            this.lblVel2.Size = new System.Drawing.Size(50, 17);
            this.lblVel2.TabIndex = 36;
            this.lblVel2.Text = "0";
            this.lblVel2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lblVel1
            // 
            this.lblVel1.BackColor = System.Drawing.Color.White;
            this.lblVel1.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.lblVel1.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblVel1.Location = new System.Drawing.Point(871, 427);
            this.lblVel1.Name = "lblVel1";
            this.lblVel1.Size = new System.Drawing.Size(50, 17);
            this.lblVel1.TabIndex = 35;
            this.lblVel1.Text = "0";
            this.lblVel1.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lblTemp4
            // 
            this.lblTemp4.BackColor = System.Drawing.Color.White;
            this.lblTemp4.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.lblTemp4.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblTemp4.Location = new System.Drawing.Point(958, 481);
            this.lblTemp4.Name = "lblTemp4";
            this.lblTemp4.Size = new System.Drawing.Size(50, 17);
            this.lblTemp4.TabIndex = 42;
            this.lblTemp4.Text = "0";
            this.lblTemp4.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lblTemp2
            // 
            this.lblTemp2.BackColor = System.Drawing.Color.White;
            this.lblTemp2.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.lblTemp2.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblTemp2.Location = new System.Drawing.Point(958, 453);
            this.lblTemp2.Name = "lblTemp2";
            this.lblTemp2.Size = new System.Drawing.Size(50, 17);
            this.lblTemp2.TabIndex = 40;
            this.lblTemp2.Text = "0";
            this.lblTemp2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lblTemp1
            // 
            this.lblTemp1.BackColor = System.Drawing.Color.White;
            this.lblTemp1.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.lblTemp1.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblTemp1.Location = new System.Drawing.Point(958, 427);
            this.lblTemp1.Name = "lblTemp1";
            this.lblTemp1.Size = new System.Drawing.Size(50, 17);
            this.lblTemp1.TabIndex = 39;
            this.lblTemp1.Text = "0";
            this.lblTemp1.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tmrJoyStickPoll
            // 
            this.tmrJoyStickPoll.Interval = 200;
            this.tmrJoyStickPoll.Tick += new System.EventHandler(this.tmrJoyStickPoll_Tick);
            // 
            // trackBarTurnPower
            // 
            this.trackBarTurnPower.BackColor = System.Drawing.Color.DimGray;
            this.trackBarTurnPower.Location = new System.Drawing.Point(284, 454);
            this.trackBarTurnPower.Maximum = 100;
            this.trackBarTurnPower.Minimum = -100;
            this.trackBarTurnPower.Name = "trackBarTurnPower";
            this.trackBarTurnPower.Size = new System.Drawing.Size(200, 45);
            this.trackBarTurnPower.TabIndex = 18;
            this.trackBarTurnPower.TickStyle = System.Windows.Forms.TickStyle.Both;
            this.trackBarTurnPower.ValueChanged += new System.EventHandler(this.trackBarTurnPower_ValueChanged);
            // 
            // trackBarForwardPower
            // 
            this.trackBarForwardPower.BackColor = System.Drawing.Color.DimGray;
            this.trackBarForwardPower.Location = new System.Drawing.Point(9, 454);
            this.trackBarForwardPower.Maximum = 100;
            this.trackBarForwardPower.Minimum = -100;
            this.trackBarForwardPower.Name = "trackBarForwardPower";
            this.trackBarForwardPower.Size = new System.Drawing.Size(200, 45);
            this.trackBarForwardPower.TabIndex = 17;
            this.trackBarForwardPower.TickStyle = System.Windows.Forms.TickStyle.Both;
            this.trackBarForwardPower.ValueChanged += new System.EventHandler(this.trackBarForwardPower_ValueChanged);
            // 
            // btnLaserScan
            // 
            this.btnLaserScan.BackColor = System.Drawing.Color.Gold;
            this.btnLaserScan.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnLaserScan.Location = new System.Drawing.Point(546, 424);
            this.btnLaserScan.Name = "btnLaserScan";
            this.btnLaserScan.Size = new System.Drawing.Size(86, 25);
            this.btnLaserScan.TabIndex = 45;
            this.btnLaserScan.Text = "LaserScan";
            this.btnLaserScan.UseVisualStyleBackColor = true;
            this.btnLaserScan.Click += new System.EventHandler(this.btnLaserScan_Click);
            // 
            // imageList1
            // 
            this.imageList1.ImageStream = ((System.Windows.Forms.ImageListStreamer)(resources.GetObject("imageList1.ImageStream")));
            this.imageList1.TransparentColor = System.Drawing.Color.Transparent;
            this.imageList1.Images.SetKeyName(0, "tank_gps_connected.ico");
            this.imageList1.Images.SetKeyName(1, "tank_gps_disconnect.ico");
            this.imageList1.Images.SetKeyName(2, "tank_laser_connected.ico");
            this.imageList1.Images.SetKeyName(3, "tank_laser_disconnect.ico");
            // 
            // tmrDisplay
            // 
            this.tmrDisplay.Enabled = true;
            this.tmrDisplay.Interval = 200;
            this.tmrDisplay.Tick += new System.EventHandler(this.tmrDisplay_Tick);
            // 
            // tmrDrawing
            // 
            this.tmrDrawing.Interval = 1000;
            this.tmrDrawing.Tick += new System.EventHandler(this.tmrDrawing_Tick);
            // 
            // imageList2
            // 
            this.imageList2.ImageStream = ((System.Windows.Forms.ImageListStreamer)(resources.GetObject("imageList2.ImageStream")));
            this.imageList2.TransparentColor = System.Drawing.Color.Transparent;
            this.imageList2.Images.SetKeyName(0, "tank_Jaguar-Lite_blod_473x373.bmp");
            this.imageList2.Images.SetKeyName(1, "tank_Jaguar-4Track_blod_473x373.bmp");
            this.imageList2.Images.SetKeyName(2, "tank_Jaguar-4Wheel_blod_474x373.bmp");
            // 
            // label4
            // 
            this.label4.Location = new System.Drawing.Point(498, 420);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(41, 38);
            this.label4.TabIndex = 47;
            this.label4.Text = "Map Zoom";
            this.label4.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // btnDisArm
            // 
            this.btnDisArm.BackColor = System.Drawing.Color.Red;
            this.btnDisArm.Location = new System.Drawing.Point(497, 424);
            this.btnDisArm.Name = "btnDisArm";
            this.btnDisArm.Size = new System.Drawing.Size(37, 76);
            this.btnDisArm.TabIndex = 54;
            this.btnDisArm.Text = "Dis Arm";
            this.btnDisArm.UseVisualStyleBackColor = false;
            this.btnDisArm.Click += new System.EventHandler(this.btnDisArm_Click);
            // 
            // checkBoxMotorProtect
            // 
            this.checkBoxMotorProtect.AutoSize = true;
            this.checkBoxMotorProtect.Checked = true;
            this.checkBoxMotorProtect.CheckState = System.Windows.Forms.CheckState.Checked;
            this.checkBoxMotorProtect.ForeColor = System.Drawing.Color.Black;
            this.checkBoxMotorProtect.Location = new System.Drawing.Point(638, 447);
            this.checkBoxMotorProtect.Name = "checkBoxMotorProtect";
            this.checkBoxMotorProtect.Size = new System.Drawing.Size(90, 17);
            this.checkBoxMotorProtect.TabIndex = 55;
            this.checkBoxMotorProtect.Text = "Motor Protect";
            this.checkBoxMotorProtect.UseVisualStyleBackColor = true;
            this.checkBoxMotorProtect.CheckedChanged += new System.EventHandler(this.checkBoxMotorProtect_CheckedChanged);
            // 
            // label5
            // 
            this.label5.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label5.ForeColor = System.Drawing.Color.Black;
            this.label5.Location = new System.Drawing.Point(650, 479);
            this.label5.Margin = new System.Windows.Forms.Padding(0);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(76, 24);
            this.label5.TabIndex = 56;
            this.label5.Text = "Stuck Detect";
            this.label5.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // pictureBoxStuckDetect
            // 
            this.pictureBoxStuckDetect.Location = new System.Drawing.Point(638, 486);
            this.pictureBoxStuckDetect.Name = "pictureBoxStuckDetect";
            this.pictureBoxStuckDetect.Size = new System.Drawing.Size(15, 10);
            this.pictureBoxStuckDetect.TabIndex = 57;
            this.pictureBoxStuckDetect.TabStop = false;
            // 
            // checkBoxHardware
            // 
            this.checkBoxHardware.AutoSize = true;
            this.checkBoxHardware.Checked = true;
            this.checkBoxHardware.CheckState = System.Windows.Forms.CheckState.Checked;
            this.checkBoxHardware.ForeColor = System.Drawing.Color.Black;
            this.checkBoxHardware.Location = new System.Drawing.Point(638, 429);
            this.checkBoxHardware.Name = "checkBoxHardware";
            this.checkBoxHardware.Size = new System.Drawing.Size(102, 17);
            this.checkBoxHardware.TabIndex = 58;
            this.checkBoxHardware.Text = "Hardware Mode";
            this.checkBoxHardware.UseVisualStyleBackColor = true;
            this.checkBoxHardware.CheckedChanged += new System.EventHandler(this.checkBoxHardware_CheckedChanged);
            // 
            // buttonReset
            // 
            this.buttonReset.BackColor = System.Drawing.Color.Gold;
            this.buttonReset.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.buttonReset.Location = new System.Drawing.Point(546, 451);
            this.buttonReset.Name = "buttonReset";
            this.buttonReset.Size = new System.Drawing.Size(86, 24);
            this.buttonReset.TabIndex = 59;
            this.buttonReset.Text = "Reset";
            this.buttonReset.UseVisualStyleBackColor = true;
            this.buttonReset.Click += new System.EventHandler(this.btnReset_Click);
            // 
            // pictureBox2
            // 
            this.pictureBox2.BackColor = System.Drawing.Color.Transparent;
            this.pictureBox2.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("pictureBox2.BackgroundImage")));
            this.pictureBox2.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.pictureBox2.Image = ((System.Drawing.Image)(resources.GetObject("pictureBox2.Image")));
            this.pictureBox2.InitialImage = ((System.Drawing.Image)(resources.GetObject("pictureBox2.InitialImage")));
            this.pictureBox2.Location = new System.Drawing.Point(15, 3);
            this.pictureBox2.Name = "pictureBox2";
            this.pictureBox2.Size = new System.Drawing.Size(64, 34);
            this.pictureBox2.SizeMode = System.Windows.Forms.PictureBoxSizeMode.StretchImage;
            this.pictureBox2.TabIndex = 61;
            this.pictureBox2.TabStop = false;
            // 
            // label6
            // 
            this.label6.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.label6.Font = new System.Drawing.Font("Arial", 24F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label6.ForeColor = System.Drawing.Color.Black;
            this.label6.ImageAlign = System.Drawing.ContentAlignment.TopLeft;
            this.label6.Location = new System.Drawing.Point(80, 3);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(169, 36);
            this.label6.TabIndex = 62;
            this.label6.Text = "E190Q";
            this.label6.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // label7
            // 
            this.label7.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.label7.Font = new System.Drawing.Font("Arial", 14F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label7.ForeColor = System.Drawing.Color.Black;
            this.label7.Location = new System.Drawing.Point(188, 13);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(290, 24);
            this.label7.TabIndex = 63;
            this.label7.Text = "Autonomous Robot Navigation";
            this.label7.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // buttonStop
            // 
            this.buttonStop.BackColor = System.Drawing.Color.Silver;
            this.buttonStop.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.buttonStop.Location = new System.Drawing.Point(217, 454);
            this.buttonStop.Name = "buttonStop";
            this.buttonStop.Size = new System.Drawing.Size(59, 45);
            this.buttonStop.TabIndex = 64;
            this.buttonStop.Text = "Stop";
            this.buttonStop.UseVisualStyleBackColor = true;
            this.buttonStop.Click += new System.EventHandler(this.btnStop_Click);
            // 
            // panelGE
            // 
            this.panelGE.BackColor = System.Drawing.Color.White;
            this.panelGE.Location = new System.Drawing.Point(11, 67);
            this.panelGE.Name = "panelGE";
            this.panelGE.Size = new System.Drawing.Size(10, 34);
            this.panelGE.TabIndex = 3;
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label8.ForeColor = System.Drawing.Color.Black;
            this.label8.Location = new System.Drawing.Point(838, 455);
            this.label8.Margin = new System.Windows.Forms.Padding(0);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(35, 14);
            this.label8.TabIndex = 65;
            this.label8.Text = "R Vel:";
            this.label8.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label13
            // 
            this.label13.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label13.ForeColor = System.Drawing.Color.Black;
            this.label13.Location = new System.Drawing.Point(921, 481);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(40, 15);
            this.label13.TabIndex = 66;
            this.label13.Text = "T(rad):";
            this.label13.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // label14
            // 
            this.label14.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label14.ForeColor = System.Drawing.Color.Black;
            this.label14.Location = new System.Drawing.Point(838, 479);
            this.label14.Name = "label14";
            this.label14.Size = new System.Drawing.Size(34, 17);
            this.label14.TabIndex = 67;
            this.label14.Text = "Y(m):";
            this.label14.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // label16
            // 
            this.label16.AutoSize = true;
            this.label16.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label16.ForeColor = System.Drawing.Color.Black;
            this.label16.Location = new System.Drawing.Point(921, 454);
            this.label16.Margin = new System.Windows.Forms.Padding(0);
            this.label16.Name = "label16";
            this.label16.Size = new System.Drawing.Size(42, 14);
            this.label16.TabIndex = 68;
            this.label16.Text = "RT(°C):";
            this.label16.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // desiredT
            // 
            this.desiredT.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.desiredT.ForeColor = System.Drawing.Color.Black;
            this.desiredT.Location = new System.Drawing.Point(189, 427);
            this.desiredT.Name = "desiredT";
            this.desiredT.Size = new System.Drawing.Size(59, 18);
            this.desiredT.TabIndex = 69;
            this.desiredT.Text = "Des T:";
            this.desiredT.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // txtStartTheta
            // 
            this.txtStartTheta.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txtStartTheta.Location = new System.Drawing.Point(224, 426);
            this.txtStartTheta.Name = "txtStartTheta";
            this.txtStartTheta.Size = new System.Drawing.Size(47, 20);
            this.txtStartTheta.TabIndex = 70;
            this.txtStartTheta.Text = "0";
            // 
            // txtNumParticles
            // 
            this.txtNumParticles.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txtNumParticles.Location = new System.Drawing.Point(341, 427);
            this.txtNumParticles.Name = "txtNumParticles";
            this.txtNumParticles.Size = new System.Drawing.Size(47, 20);
            this.txtNumParticles.TabIndex = 71;
            this.txtNumParticles.Text = "1000";
            // 
            // label19
            // 
            this.label19.Font = new System.Drawing.Font("Arial", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label19.ForeColor = System.Drawing.Color.Black;
            this.label19.Location = new System.Drawing.Point(303, 428);
            this.label19.Name = "label19";
            this.label19.Size = new System.Drawing.Size(59, 18);
            this.label19.TabIndex = 72;
            this.label19.Text = "Num P:";
            this.label19.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // checkBoxKnownStart
            // 
            this.checkBoxKnownStart.AutoSize = true;
            this.checkBoxKnownStart.Checked = true;
            this.checkBoxKnownStart.CheckState = System.Windows.Forms.CheckState.Checked;
            this.checkBoxKnownStart.ForeColor = System.Drawing.Color.Black;
            this.checkBoxKnownStart.Location = new System.Drawing.Point(638, 465);
            this.checkBoxKnownStart.Name = "checkBoxKnownStart";
            this.checkBoxKnownStart.Size = new System.Drawing.Size(105, 17);
            this.checkBoxKnownStart.TabIndex = 73;
            this.checkBoxKnownStart.Text = "Known Start Loc";
            this.checkBoxKnownStart.UseVisualStyleBackColor = true;
            this.checkBoxKnownStart.CheckStateChanged += new System.EventHandler(this.checkBoxKnownStart_CheckedChanged);
            // 
            // JaguarCtrl
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.SystemColors.ControlLight;
            this.ClientSize = new System.Drawing.Size(1016, 698);
            this.Controls.Add(this.txtNumParticles);
            this.Controls.Add(this.label19);
            this.Controls.Add(this.txtStartTheta);
            this.Controls.Add(this.desiredT);
            this.Controls.Add(this.lblTemp4);
            this.Controls.Add(this.lblTemp2);
            this.Controls.Add(this.label16);
            this.Controls.Add(this.label14);
            this.Controls.Add(this.label13);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.buttonStop);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.btnRecord);
            this.Controls.Add(this.pictureBox2);
            this.Controls.Add(this.buttonReset);
            this.Controls.Add(this.checkBoxHardware);
            this.Controls.Add(this.checkBoxKnownStart);
            this.Controls.Add(this.pictureBoxStuckDetect);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.checkBoxMotorProtect);
            this.Controls.Add(this.btnDisArm);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.btnLaserScan);
            this.Controls.Add(this.trackBarTurnPower);
            this.Controls.Add(this.trackBarForwardPower);
            this.Controls.Add(this.lblTemp1);
            this.Controls.Add(this.lblMot3);
            this.Controls.Add(this.lblVel4);
            this.Controls.Add(this.lblVel2);
            this.Controls.Add(this.lblVel1);
            this.Controls.Add(this.lblEncoderPos4);
            this.Controls.Add(this.lblEncoderPos2);
            this.Controls.Add(this.lblEncoderPos1);
            this.Controls.Add(this.trackBarZoom);
            this.Controls.Add(this.txtStartLong);
            this.Controls.Add(this.txtStartLat);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.btnSetStartPoint);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.lblMot2);
            this.Controls.Add(this.lblMot1);
            this.Controls.Add(this.panelGE);
            this.Controls.Add(this.panel1);
            this.Controls.Add(this.groupBoxGPSIMU);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedDialog;
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.MaximizeBox = false;
            this.Name = "JaguarCtrl";
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.Text = "E190Q - Jaguar Control";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.JaguarCtrl_FormClosing);
            this.FormClosed += new System.Windows.Forms.FormClosedEventHandler(this.JaguarCtrl_FormClosed);
            this.Load += new System.EventHandler(this.JaguarCtrl_Load);
            this.Shown += new System.EventHandler(this.JaguarCtrl_Shown);
            this.groupBoxGPSIMU.ResumeLayout(false);
            this.groupBoxGPSIMU.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.realJaguar)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxIMUGPS)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxGyroZ)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxGyroY)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxGyroX)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxAccelZ)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxAccelY)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxAccelX)).EndInit();
            this.panel1.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.myAMC)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxLaser)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxSensor)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBarZoom)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBarTurnPower)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBarForwardPower)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxStuckDetect)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.GroupBox groupBoxGPSIMU;
        private System.Windows.Forms.PictureBox pictureBoxGyroZ;
        private System.Windows.Forms.PictureBox pictureBoxGyroY;
        private System.Windows.Forms.PictureBox pictureBoxGyroX;
        private System.Windows.Forms.PictureBox pictureBoxAccelZ;
        private System.Windows.Forms.PictureBox pictureBoxAccelY;
        private System.Windows.Forms.PictureBox pictureBoxAccelX;
        private System.Windows.Forms.Label label46;
        private System.Windows.Forms.Label lblLat;
        private System.Windows.Forms.Label label15;
        private System.Windows.Forms.Label label17;
        private System.Windows.Forms.Label lblLong;
        private System.Windows.Forms.Label label33;
        private System.Windows.Forms.Label lblVOG;
        private System.Windows.Forms.Label label18;
        private System.Windows.Forms.Label lblCOG;
        private System.Windows.Forms.Label lblGPSQI;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.TextBox textBoxGPSRCV;
        private System.Windows.Forms.TextBox textBoxRCV;
        private System.Windows.Forms.Label label2;
        private AxAXISMEDIACONTROLLib.AxAxisMediaControl myAMC;
        //public AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel realJaguar; 
        public AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel realJaguar;             //-wf
        public AxDDrRobotSentinel_Simulator simulatedJaguar;
        private System.Windows.Forms.Panel panel1;
        private System.Windows.Forms.PictureBox pictureBoxSensor;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label lblMot3;
        private System.Windows.Forms.Label lblMot2;
        private System.Windows.Forms.Label lblMot1;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        public System.Windows.Forms.Button btnRecord;
        public System.Windows.Forms.TextBox txtStartLong;
        public System.Windows.Forms.TextBox txtStartLat;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Button btnSetStartPoint;
        public System.Windows.Forms.TrackBar trackBarZoom;
        private System.Windows.Forms.Label lblEncoderPos1;
        private System.Windows.Forms.Label lblEncoderPos2;
        private System.Windows.Forms.Label lblEncoderPos4;
        private System.Windows.Forms.Label lblVel4;
        private System.Windows.Forms.Label lblVel2;
        private System.Windows.Forms.Label lblVel1;
        private System.Windows.Forms.Label lblTemp4;
        private System.Windows.Forms.Label lblTemp2;
        private System.Windows.Forms.Label lblTemp1;
        private System.Windows.Forms.Label lblBatVol;
        private System.Windows.Forms.Timer tmrJoyStickPoll;
        private System.Windows.Forms.TrackBar trackBarTurnPower;
        private System.Windows.Forms.TrackBar trackBarForwardPower;
        private System.Windows.Forms.Button btnLaserScan;
        private System.Windows.Forms.PictureBox pictureBoxIMUGPS;
        private System.Windows.Forms.ImageList imageList1;
        private System.Windows.Forms.Timer tmrDisplay;
        private System.Windows.Forms.PictureBox pictureBoxLaser;
        private System.Windows.Forms.Timer tmrDrawing;
        private System.Windows.Forms.Button btnScan;
        private System.Windows.Forms.ImageList imageList2;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Button btnDisArm;
        private System.Windows.Forms.CheckBox checkBoxMotorProtect;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.PictureBox pictureBoxStuckDetect;
        private System.Windows.Forms.Button btnTurnOn;
        private System.Windows.Forms.CheckBox checkBoxHardware;
        private System.Windows.Forms.Button buttonReset;
        private System.Windows.Forms.PictureBox pictureBox2;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Button buttonStop;
        public System.Windows.Forms.Panel panelGE;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.Label label14;
        private System.Windows.Forms.Label label16;
        private System.Windows.Forms.Label desiredT;
        public System.Windows.Forms.TextBox txtStartTheta;
        public System.Windows.Forms.TextBox txtNumParticles;
        private System.Windows.Forms.Label label19;
        private System.Windows.Forms.CheckBox checkBoxKnownStart;
    }
}


using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Diagnostics;
using System.Xml;
using System.Threading;
using System.Text.RegularExpressions;

namespace DrRobot.JaguarControl
{
    
    public partial class DrRobotRobotConnection : Form
    {
        public RobotConfig robotConfig = new RobotConfig();
        private RobotConfig.RobotConfigTableRow row = null;
        public static Process p1 = null;

        private const string configFile = "c:\\DrRobotAppFile\\OutDoorRobotConfig.xml";

        //public static JaguarCtrl controlForm = null;
        public JaguarCtrl controlForm = null;
        public DrRobotRobotConnection(JaguarCtrl jc)
        {
            InitializeComponent();
            controlForm = jc;
            DrRobotRobotConnection_Load();
        }

        private void DrRobotRobotConnection_Load()
        {

            robotConfig.Clear();
            try
            {
                robotConfig.ReadXml(configFile);
            }
            catch
            {
                MessageBox.Show("Unable to open robot configure file!", "DrRobot Jaguar Control", MessageBoxButtons.OK, MessageBoxIcon.Error);
                Close();

            }
            row = (RobotConfig.RobotConfigTableRow)robotConfig.RobotConfigTable.Rows[0];
            txtRobotID.Text = row.RobotID;
            txtRobotIP.Text = row.RobotIP;
            txtGPSIP.Text = row.GPSIP;
            txtCameraIP.Text = row.CameraIP;
            txtCameraPWD.Text = row.CameraPWD;
            txtCameraID.Text = row.CameraUser;

        }
        
        public void connectRobot()
        {
            //save any change to xml file
            row.LaserRangeIP = txtRobotIP.Text;
            
            row.RobotID = txtRobotID.Text;
            row.RobotIP = txtRobotIP.Text;
            row.GPSIP = txtGPSIP.Text;
            row.IMUIP = txtGPSIP.Text;
            row.CameraIP = txtCameraIP.Text;
            //row.CameraPWD = txtCameraPWD.Text;
            row.CameraUser = txtCameraID.Text;
            
            try
            {
                robotConfig.WriteXml(configFile);
            }
            catch
            {
            }
            //write gateway config file 
            XmlTextWriter textWriter = new XmlTextWriter("C:\\DrRobotAppFile\\gatewayConfig.xml", null);
            textWriter.WriteStartDocument();
            
            // Write comments
            textWriter.WriteComment("For Gateway Communication");
            
            // Write first element
            textWriter.WriteStartElement("GateWaySetting", "");
            
            // Write next element
            textWriter.WriteStartElement("RobotID", "");
            textWriter.WriteString(row.RobotID);
            textWriter.WriteEndElement();
            textWriter.WriteStartElement("RobotMethod", "");
            textWriter.WriteString("WiFi");
            
            //textWriter.WriteString("Serial");
            textWriter.WriteEndElement();
            textWriter.WriteStartElement("RobotCom", "");
            textWriter.WriteString("COM1");
            textWriter.WriteEndElement();
            textWriter.WriteStartElement("RobotIP", "");
            textWriter.WriteString(row.RobotIP);
            textWriter.WriteEndElement();
            textWriter.WriteStartElement("RobotPort", "");
            textWriter.WriteString("10001");
            textWriter.WriteEndElement();

            // Ends the document.
            textWriter.WriteEndElement();
            textWriter.WriteEndDocument();

            // close writer
            textWriter.Close();
            Thread.Sleep(1000);

            //start one gateway         
            p1 = new Process();
            p1.StartInfo.FileName = "C:\\DrRobotAppFile\\WirobotGateway.exe";
            p1.StartInfo.WindowStyle = ProcessWindowStyle.Minimized;
            p1.Start();

            Thread.Sleep(1000);
            controlForm.robotCfg = robotConfig;
            this.Close();
        }

        private void btnConnect_Click(object sender, EventArgs e)
        {
            connectRobot();
        }

        private void DrRobotRobotConnection_FormClosing(object sender, FormClosingEventArgs e)
        {
            DrRobotRobotConnection_Kill();
        }

        public void DrRobotRobotConnection_Kill()
        {
            try
            {
                if (p1 != null)
                    p1.Kill();
            }
            catch
            {
            }
        }

        /// <summary>
        /// method to validate an IP address
        /// using regular expressions. The pattern
        /// being used will validate an ip address
        /// with the range of 1.0.0.0 to 255.255.255.255
        /// </summary>
        /// <param name="addr">Address to validate</param>
        /// <returns></returns>
        public bool IsValidIP(string addr)
        {
            //create our match pattern
            string pattern = @"^([1-9]|[1-9][0-9]|1[0-9][0-9]|2[0-4][0-9]|25[0-5])(\.([0-9]|[1-9][0-9]|1[0-9][0-9]|2[0-4][0-9]|25[0-5])){3}$";
            //create our Regular Expression object
            Regex check = new Regex(pattern);
            //boolean variable to hold the status
            bool valid = false;
            //check to make sure an ip address was provided
            if (addr == "")
            {
                //no address provided so return false
                valid = false;
            }
            else
            {
                //address provided so use the IsMatch Method
                //of the Regular Expression object
                valid = check.IsMatch(addr, 0);
            }
            //return the results
            return valid;
        }
        
        private void txtRobotIP_Leave(object sender, EventArgs e)
        {
            string addre = txtRobotIP.Text;
            if (!IsValidIP(addre))
            {
                MessageBox.Show("This is an invalid address!", "DrRobot Jaguar Control", MessageBoxButtons.OK, MessageBoxIcon.Error);
                txtRobotIP.Focus();
            }
        }

        private void txtGPSIP_Leave(object sender, EventArgs e)
        {
            string addre = txtGPSIP.Text;
            if (!IsValidIP(addre))
            {
                MessageBox.Show("This is an invalid address!", "DrRobot Jaguar Control", MessageBoxButtons.OK, MessageBoxIcon.Error);
                txtGPSIP.Focus();
            }
        }

        private void txtCameraIP_Leave(object sender, EventArgs e)
        {
            string addre = txtCameraIP.Text;
            if (!IsValidIP(addre))
            {
                MessageBox.Show("This is an invalid address!", "DrRobot Motion/Power Control", MessageBoxButtons.OK, MessageBoxIcon.Error);
                txtCameraIP.Focus();
            }
        }

      
    }
}

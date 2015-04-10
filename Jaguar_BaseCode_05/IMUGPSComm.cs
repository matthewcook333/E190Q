using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO;
using System.Net.Sockets;
using System.Threading;
using System.Globalization;

namespace DrRobot.JaguarControl
{
    public partial class JaguarCtrl : Form
    {
        private TcpClient clientSocketIMU = null;
        private Thread threadClientIMU = null;
        private NetworkStream cmdStreamIMU = null;

        
        private StreamReader readerIMU = null;
        private StreamWriter writerIMU = null;
        private delegate void updateSensorDataInfo(string data);

        private TcpClient clientSocketGPS = null;
        private Thread threadClientGPS = null;
        private NetworkStream cmdStreamGPS = null;
        private StreamReader readerGPS = null;


        //private bool firstDataIMU = true;
        //private bool firstDataGPS = true;

        private bool receivingGPS = false;
        private bool receivingIMU = false;


        private bool firstSetupComm = true;

        //private int DATA_LEN = 14;      

        private int seqNo = 0;
        private double accel_x = 0;
        private double accel_y = 0;
        private double accel_z = 0;
        private double accel_x_offset = 0;
        private double accel_y_offset = 0;
        private double accel_z_offset = 0;

        private double gyro_x = 0;
        private double gyro_y = 0;
        private double gyro_z = 0;

        private double gyro_x_offset = 0;   //377
        private double gyro_y_offset = 0;
        private double gyro_z_offset = 0;


        public struct IMURecord
        {
            public int index;
            public double accel_x;
            public double accel_y;
            public double accel_z;
            public double gyro_x;
            public double gyro_y;
            public double gyro_z;
            public double magn_x;
            public double magn_y;
            public double magn_z;
            public double eRoll;
            public double ePitch;
            public double eYaw;
        }

        public IMURecord imuRecord = new IMURecord();
        public int recordCnt = 0;
        public bool startRecord = false;
        public StreamWriter SW;
        public string fileNme = "c:\\DrRobotAppFile\\DrRobotGPSIMURec";
        public int fileCnt = 0;
        public const int MAXFILELEN = 32767;
        public struct GPSRecord
        {
            public double latitude;
            public double longitude;
            public double altitude;
            public double seaHeight;
            public double geoidalHeight;
            public double vog;
            public double cog;
            public int satNum;
            public double timeStamp;
            public string latHemi;
            public string lngHemi;
            public double h_pre;
            public double v_pre;
            public double p_pre;
            public string status;
            public int qi;
            public string recRaw;
        }

        public GPSRecord gpsRecord = new GPSRecord();
        public const double KNNOT2MS = 0.514444444;
        private delegate void updateGPSSensorDataInfo(string data);

        private const int DrawDataLen = 200;        //around 4 second
        private double[] draw_AccelX = new double[DrawDataLen];
        private double[] draw_AccelY = new double[DrawDataLen];
        private double[] draw_AccelZ = new double[DrawDataLen];
        private double[] draw_GyroX = new double[DrawDataLen];
        private double[] draw_GyroY = new double[DrawDataLen];
        private double[] draw_GyroZ = new double[DrawDataLen];
        private int drawStartPoint = 0;
        private int drawEndPoint = 0;

        public double getAccel_x()
        {
            return imuRecord.accel_x;
        }
        
        public double getAccel_y()
        {
            return imuRecord.accel_y;
        }

        public double getAccel_z()
        {
            return imuRecord.accel_z;
        }

        private void startComm()
        {
            int remotePortGPS = jaguarSetting.GPSPort;
            String IPAddrGPS = jaguarSetting.GPSIP;
            try
            {
                clientSocketGPS = new TcpClient();
                IAsyncResult result = clientSocketGPS.BeginConnect(IPAddrGPS, remotePortGPS, null, null);
                bool success = result.AsyncWaitHandle.WaitOne(500, true);
                if (!success)
                {
                    receivingGPS = false;
                }
                else
                {
                    //firstDataGPS = true;
                    receivingGPS = true;
                    threadClientGPS = new Thread(new ThreadStart(HandleClientGPS));
                    threadClientGPS.CurrentCulture = new CultureInfo("en-US");
                    threadClientGPS.Start();
                    
                }
            }
            catch
            {
                receivingGPS = false;
                
            }


            //for IMU
            int remotePort = jaguarSetting.IMUPort;
            String IPAddr = jaguarSetting.IMUIP;
            firstSetupComm = true;
            try
            {
                clientSocketIMU = new TcpClient();
                IAsyncResult result = clientSocketIMU.BeginConnect(IPAddr, remotePort, null, null);
                bool success = result.AsyncWaitHandle.WaitOne(500, true);
                if (!success)
                {
                    receivingIMU = false;
                }
                else
                {
                    //firstDataIMU = true;
                    receivingIMU = true;
                    threadClientIMU = new Thread(new ThreadStart(HandleClientIMU));
                    threadClientIMU.CurrentCulture = new CultureInfo("en-US");
                    threadClientIMU.Start();
                }
            }
            catch
            {
                receivingIMU = false;
            }




            if (receivingGPS && receivingIMU)
            {
                pictureBoxIMUGPS.Image = imageList1.Images[0];
            }
            else
            {
                pictureBoxIMUGPS.Image = imageList1.Images[1];
            }

        }

        private void stopCommunication()
        {

            if (clientSocketIMU != null)
                clientSocketIMU.Close();

            pictureBoxIMUGPS.Image = imageList1.Images[1];
            
            receivingIMU = false;
            if (writerIMU != null)
            {
                try
                {
                    writerIMU.Close();
                }
                catch
                {
                }
            }
            if (readerIMU != null)
                readerIMU.Close();
            if (cmdStreamIMU != null)
                cmdStreamIMU.Close();
            if (threadClientIMU != null)
                threadClientIMU.Abort();


            if (clientSocketGPS != null)
                clientSocketGPS.Close();

            receivingGPS = false;

            if (readerGPS != null)
                readerGPS.Close();
            if (cmdStreamGPS != null)
                cmdStreamGPS.Close();
            if (threadClientGPS != null)
                threadClientGPS.Abort();
        }

        //for ASCII output
        private void HandleClientIMU()
        {
            try
            {
                //here process all receive data from client
                cmdStreamIMU = clientSocketIMU.GetStream();
                readerIMU = new StreamReader(cmdStreamIMU);
                writerIMU = new StreamWriter(cmdStreamIMU);


                string strRec = "";

                while (receivingIMU)
                {
                    try
                    {

                        if (!cmdStreamIMU.DataAvailable)      //there is no data
                        {
                            //need to sleep for other thread
                            Thread.Sleep(5);        //50 Hz
                        }
                        else
                        {
                            // Reads NetworkStream into a byte buffer.
                            strRec = readerIMU.ReadLine();
                            if (strRec != null)
                                processDataIMU(strRec);



                        }
                    }
                    catch (Exception e)
                    {
                        string temp = e.ToString();

                    }
                    finally
                    {
                    }


                }
                receivingIMU = false;

                readerIMU.Close();
                cmdStreamIMU.Close();
                clientSocketIMU.Close();
            }
            catch
            {
                receivingIMU = false;
            }


        }

        private void processDataIMU(string msg)
        {
            string[] data = msg.Split(',');
            
            if (data.Length == 10) //the whole package here
            {
                Invoke(new updateSensorDataInfo(updateSensor), msg);
                
                data[0] = data[0].Remove(0,1);
                seqNo = int.Parse(data[0]);
                accel_x = double.Parse(data[1] );
                accel_y = double.Parse(data[2]);
                accel_z = double.Parse(data[3]);
                gyro_x = double.Parse(data[4]);
                gyro_y = double.Parse(data[5]);
                gyro_z = double.Parse(data[6]);
                imuRecord.accel_x = accel_x;
                imuRecord.accel_y = accel_y;
                imuRecord.accel_z = accel_z;
                imuRecord.gyro_x = gyro_x;
                imuRecord.gyro_y = gyro_y;
                imuRecord.gyro_z = gyro_z;
                imuRecord.index = seqNo;
                imuRecord.magn_x = double.Parse(data[7]);
                imuRecord.magn_y = double.Parse(data[8]);
                data[9] = data[9].Remove(data[9].Length - 1, 1);
                imuRecord.magn_z = double.Parse(data[9]);
                
                //for drawing
                draw_AccelX[drawEndPoint] = accel_x - accel_x_offset;
                draw_AccelY[drawEndPoint] = accel_y - accel_y_offset;
                draw_AccelZ[drawEndPoint] = accel_z - accel_z_offset;
                draw_GyroX[drawEndPoint] = gyro_x - gyro_x_offset;
                draw_GyroY[drawEndPoint] = gyro_y - gyro_y_offset;
                draw_GyroZ[drawEndPoint] = gyro_z - gyro_z_offset;
                drawEndPoint++;
                if (drawEndPoint >= DrawDataLen)
                {
                    drawEndPoint = 0;

                }

                if (drawEndPoint == drawStartPoint)
                {
                    drawStartPoint++;
                    if (drawStartPoint >= DrawDataLen)
                        drawStartPoint = 0;
                }

                if (startRecord)
                {
                    String recTemp = imuRecord.index.ToString() + "," +
                                    imuRecord.accel_x.ToString() + "," +
                                    imuRecord.accel_y.ToString() + "," +
                                    imuRecord.accel_z.ToString() + "," +
                                    imuRecord.gyro_x.ToString() + "," +
                                    imuRecord.gyro_y.ToString() + "," +
                                    imuRecord.gyro_z.ToString() + "," +
                                    imuRecord.magn_x.ToString() + "," +
                                    imuRecord.magn_y.ToString() + "," +
                                    imuRecord.magn_z.ToString();
                                    

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

            
        }

        private void updateSensor(string msg)
        {
            textBoxRCV.Text = msg;
        }
   
        private void HandleClientGPS()
        {
            //here process all receive data from client
            try
            {
                cmdStreamGPS = clientSocketGPS.GetStream();
                readerGPS = new StreamReader(cmdStreamGPS);

                string strRec = "";

                while (receivingGPS)
                {
                    try
                    {

                        if (!cmdStreamGPS.DataAvailable)      //there is no data
                        {
                            //need to sleep for other thread
                            Thread.Sleep(40);        //5 Hz
                        }
                        else
                        {
                            // Reads NetworkStream into a byte buffer.
                            strRec = readerGPS.ReadLine();
                            if (strRec != null)
                                processDataGPS(strRec);
                        }
                    }
                    catch (Exception e)
                    {
                        string temp = e.ToString();

                    }
                    finally
                    {
                    }


                }
                receivingGPS = false;
                readerGPS.Close();
                cmdStreamGPS.Close();
                clientSocketGPS.Close();
            }
            catch
            {
                receivingGPS = false;
            }
            
        }
        private void updateSensorGPS(string rcv)
        {
            textBoxGPSRCV.Text = rcv;
        }

        private void processDataGPS(string input)
        {
            if ((input.Substring(0, 3) == "$GP") || (input.Substring(0, 3) == "$PG"))
            {
                gpsRecord.recRaw = input;
                Invoke(new updateGPSSensorDataInfo(updateSensorGPS), input);
                if (startRecord)
                {
                    SW.WriteLine(gpsRecord.recRaw);
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

            String[] strData = input.Split(',');
            double longitude = 0;
            double lat = 0;
            if ((strData[0] == "$GPRMC") && (strData[3] != "") && (strData[5] != ""))
            {
                gpsRecord.timeStamp = double.Parse(strData[1]);     //time
                gpsRecord.status = strData[2];                        //A- valid, V = invalid
                gpsRecord.latitude = double.Parse(strData[3]);
                gpsRecord.latHemi = strData[4];
                gpsRecord.longitude = double.Parse(strData[5]);   //dddmm.mmmm
                gpsRecord.lngHemi = strData[6];
                gpsRecord.vog = double.Parse(strData[7]) * KNNOT2MS;
                gpsRecord.cog = double.Parse(strData[8]);
                

                gEarth.curLatitude = ToDegree(gpsRecord.latitude);
                gEarth.curLongitude = -ToDegree(gpsRecord.longitude);


                if ((gEarth.preLatitude == 0) && (gEarth.preLongtitude == 0))
                {

                }
                else
                {
                    //display real time GPS information here
                    if ((gEarth.preLatitude != 0) && (gEarth.preLongtitude != 0))
                    {
                        kmlLineDataMakeLoad(gEarth.setColor.SetBlue, 4, gEarth.preLatitude, gEarth.preLongtitude, gEarth.curLatitude, gEarth.curLongitude);
                    }
                 
                }
                gEarth.preEstLatitude = lat;
                gEarth.preEstLongitude = longitude;

                gEarth.preLatitude = gEarth.curLatitude;
                gEarth.preLongtitude = gEarth.curLongitude;

            }
            else if (strData[0] == "$GPGGA")
            {
                gpsRecord.timeStamp = double.Parse(strData[1]);     //time
                gpsRecord.latitude = double.Parse(strData[2]);
                gpsRecord.latHemi = strData[3];
                gpsRecord.longitude = double.Parse(strData[4]);   //dddmm.mmmm
                gpsRecord.lngHemi = strData[5];
                gpsRecord.qi = int.Parse(strData[6]);
                gpsRecord.satNum = int.Parse(strData[7]);
                gpsRecord.h_pre = double.Parse(strData[8]);
                gpsRecord.seaHeight = double.Parse(strData[9]);
                gpsRecord.geoidalHeight = double.Parse(strData[10]);
                gEarth.curLatitude = ToDegree(gpsRecord.latitude);
                gEarth.curLongitude = -ToDegree(gpsRecord.longitude);

            }
            else if (strData[0] == "$GPGSA")
            {

                gpsRecord.p_pre = double.Parse(strData[4]);
                gpsRecord.h_pre = double.Parse(strData[5]);
                gpsRecord.v_pre = double.Parse(strData[6]);

            }
            else if (strData[0] == "$GPGSV")
            {
                int len = 0;
                len = strData.Length;
                gpsRecord.satNum = int.Parse(strData[3]);
             }
        }

        private double ToDegree(double angle)
        {
            //to 
            double deg = 0;

            int degree = (int)(angle / 100);
            double min = ((angle - degree * 100));

            deg = degree + ((double)min) / 60;

            return deg;
        }

        private void kmlLineDataMakeLoad(string settingColor, int setWidth, double preLatitude, double preLongtitude, double lat, double longitude)
        {
            //make data
            gEarth.kmlFileCoordinate = @"<coordinates>" + preLongtitude.ToString() + "," + preLatitude.ToString() + "," + "0" +
            " " + longitude.ToString() + "," + lat.ToString() + "," + "0</coordinates>";
            gEarth.kmlFileStyle = @"<Style id=""MyLineStyle""><LineStyle><color>" + settingColor + @"</color><width>" + setWidth.ToString() + @"</width></LineStyle></Style>";
            string data = gEarth.kmlFileStr1 + gEarth.kmlFileStyle + gEarth.kmlFilePlacemark + gEarth.kmlFileCoordinate + gEarth.kmlFileEnd;
            gEarth.googleEarth.LoadKmlData(ref data);

        }


    }
}
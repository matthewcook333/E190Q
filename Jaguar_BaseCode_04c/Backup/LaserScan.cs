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
using System.Drawing.Drawing2D;
using System.Globalization;

namespace DrRobot.JaguarControl
{
    public partial class JaguarCtrl : Form
    {

        private TcpClient clientSocketLaser = null;
        private Thread threadClientLaser = null;
        private NetworkStream cmdStreamLaser = null;
        private BinaryReader readerLaser = null;
        private BinaryWriter writerLaser = null;
        private bool receivingLaser = false;
        /*
        private int monitorCntLaser = 0;
        private int errorChkCntLaser = 0;
        private bool firstSetupCommLaser = true;
        */
        //private bool connectedLaser = false;
        private delegate void updateLaserSensorDataInfo(byte[] data);

        private const string SCANCOMMAND = "GD0045072503";
        //below variables are based on above scan command, 
        private const int DISDATALEN = 227; // based on scan command, it will be 680 distance data
        private const int DATALEN = 681;
        private const int DATALINE = 11;    // 11 data lines(each line = 64 byte data + sum + LF, 

        private long[] disData = new long[DISDATALEN];
        private byte[] dataArray = new Byte[DATALEN];
        private long[] disDataLast = new long[DISDATALEN];
        //this program will get data from -120 to +120. (G04572401) 724-44 = 680;
        private const double startAng = -31.5 / 180 * Math.PI; //27.0 / 180 * Math.PI;
        private const double stepAng = 0.3529 / 180 * Math.PI * 3;//2.1 / 180 * Math.PI;    cluster = 03
        private const double ratio = 25;
        private const int START_X = 0;
        private const int START_Y = 120;
        private const int WIDTH_SUB = 40;

        private int drawCnt = 3;
        private bool firstData = true;


        private void startCommLaser()
        {
            int remotePort = 10002;

            String IPAddr = jaguarSetting.LaserRangeIP;
            firstSetupComm = true;
            try
            {
                //clientSocket = new TcpClient(IPAddr, remotePort);
                clientSocketLaser = new TcpClient();
                IAsyncResult results = clientSocketLaser.BeginConnect(IPAddr, remotePort, null, null);
                bool success = results.AsyncWaitHandle.WaitOne(500, true);
                
                if (!success)
                {
                    clientSocketLaser.Close();
                    clientSocketLaser = null;
                    receivingLaser = false;
                    pictureBoxLaser.Image = imageList1.Images[3];
                }
                else
                {
                    receivingLaser = true;
                    threadClientLaser = new Thread(new ThreadStart(HandleClientLaser));
                    threadClientLaser.CurrentCulture = new CultureInfo("en-US");
                    threadClientLaser.Start();
                    pictureBoxLaser.Image = imageList1.Images[2];
                }
            }
            catch
            {
                pictureBoxLaser.Image = imageList1.Images[3];

            }
        }

        private void stopCommunicationLaser()
        {
            if (clientSocketLaser != null)
                clientSocketLaser.Close();
            
            pictureBoxLaser.Image = imageList1.Images[3];


            receivingLaser = false;
            if (writerLaser != null)
                writerLaser.Close();
            if (readerLaser != null)
                readerLaser.Close();
            if (cmdStreamLaser != null)
                cmdStreamLaser.Close();
            if (threadClientLaser != null)
                threadClientLaser.Abort();

        }

        private void HandleClientLaser()
        {
            //here process all receive data from client
            try
            {
                cmdStreamLaser = clientSocketLaser.GetStream();
                readerLaser = new BinaryReader(cmdStreamLaser);
                writerLaser = new BinaryWriter(cmdStreamLaser);

                bool keepSearch = true;
                byte[] decodeBuff = new byte[2048];  //maxlength
                int lastPos = 0;
                int processLen = 0;
                int endPos = 0;

                while (receivingLaser)
                {
                    try
                    {

                        if (!cmdStreamLaser.DataAvailable)      //there is no data
                        {
                            //need to sleep for other thread
                            Thread.Sleep(30);
                        }
                        else
                        {
                            // Reads NetworkStream into a byte buffer.

                            //errorChkCntLaser = 0;
                            byte[] bytes = new byte[2048];
                            // Read can return anything from 0 to numBytesToRead. 
                            // This method blocks until at least one byte is read.
                            processLen = readerLaser.Read(bytes, 0, bytes.Length);
                            Array.Copy(bytes, 0, decodeBuff, lastPos, processLen);
                            processLen += lastPos;
                            keepSearch = true;
                            endPos = 0;
                            lastPos = processLen;

                            while (keepSearch)
                            {
                                endPos = 0;
                                if (processLen >= 2)
                                {
                                    for (int i = 0; i < processLen - 1; i++)
                                    {
                                        if ((decodeBuff[i] == 10) && (decodeBuff[i + 1] == 10))
                                        {
                                            endPos = i;
                                            break;
                                        }
                                    }
                                }


                                if (endPos > 0)
                                {
                                    // already find the end, cut the string to decode command

                                    //string recCommand = Encoding.Unicode.GetString(decodeBuff, 0, endPos);
                                    byte[] recCommand = new byte[endPos + 2];
                                    Array.Copy(decodeBuff, 0, recCommand, 0, endPos + 2);
                                    //process receive data here
                                    processDataLaser(recCommand);

                                    if ((endPos + 2) >= processLen)
                                    {
                                        //stop search
                                        keepSearch = false;
                                        lastPos = 0;
                                    }
                                    else
                                    {
                                        // trim this package from decode buff
                                        int len = processLen - endPos - 2;
                                        byte[] temp = new byte[len];

                                        Array.Copy(decodeBuff, endPos + 2, temp, 0, len);
                                        Array.Copy(temp, 0, decodeBuff, 0, len);
                                        lastPos = len;
                                        processLen = len;
                                    }
                                }
                                else
                                {
                                    keepSearch = false;
                                    //lastPos = 0;
                                    processLen = 0;
                                }
                            }
                        }
                    }
                    catch 
                    {

                    }
                    finally
                    {
                    }


                }
                readerLaser.Close();
                cmdStreamLaser.Close();
            }
            catch
            {
            }
            receivingLaser = false;

           
            clientSocketLaser.Close();


        }

        private void processDataLaser(byte[] data)
        {
            Invoke(new updateLaserSensorDataInfo(updateSensor), data);
        }

        private void updateSensor(byte[] msg)
        {
            if (firstSetupComm)
            {
                firstSetupComm = false;
                pictureBoxLaser.Image = imageList1.Images[0];
            }

            string rcvData = Encoding.ASCII.GetString(msg);
            

            StringReader dataRead = new StringReader(rcvData);
            string ackData = "";
            try
            {
                ackData = dataRead.ReadLine();
            }
            catch
            {
            }
            string status = "";
            try
            {
                status = dataRead.ReadLine().Remove(2, 1);
            }
            catch
            {
            }
            string timeStamp = "";
            try
            {
                timeStamp = dataRead.ReadLine().Remove(4, 1);
            }
            catch
            {
            }
            long timeData = 0;
            byte[] tempData = null;
            if (timeStamp.Length == 4)
            {
                tempData = System.Text.Encoding.ASCII.GetBytes(timeStamp);

                timeData = (((long)(tempData[0] - 0x30)) << 18) + (((long)(tempData[1] - 0x30)) << 12)
                + (((long)(tempData[2] - 0x30)) << 6) + ((long)(tempData[3] - 0x30));
            }
       
            string temp = "";
            int lineCnt = 0;
            if ((status == "00") && (msg.Length == 727))
            {
                //suppose all the data is right
                while (lineCnt < DATALINE - 1)
                {
                    temp = dataRead.ReadLine();       //remove sum and LF, won't check sum here
                    temp = temp.Remove(64, 1);
                    tempData = System.Text.Encoding.ASCII.GetBytes(temp);
                    Array.Copy(tempData, 0, dataArray, lineCnt * 64, tempData.Length);
                    lineCnt++;
                }
                if (lineCnt == DATALINE - 1)
                {
                    temp = dataRead.ReadLine();
                    temp = temp.Remove(41, 1);       //the last line is 41 byte data + sum + LF + LF
                    tempData = System.Text.Encoding.ASCII.GetBytes(temp);
                    Array.Copy(tempData, 0, dataArray, lineCnt * 64, tempData.Length);
                }

                //transfer data to distance sta
                for (int i = 0; i < DISDATALEN; i++)
                {
                    disData[i] = ((long)(dataArray[3 * i] - 0x30) << 12) + (((long)(dataArray[3 * i + 1] - 0x30)) << 6) + (long)(dataArray[3 * i + 2] - 0x30);
                }


                drawSensor(DISDATALEN);
                if (tmrDrawing.Enabled == false)
                {
                    drawCnt = 0;
                    tmrDrawing.Interval = 500;
                    tmrDrawing.Enabled = true;
                    btnTurnOn.Enabled = false;
                }

            }
        }

        private void sendCommandLaser(string s)
        {
            try
            {
                s = s + new string((char)13, 1);

                ASCIIEncoding ascCode = new ASCIIEncoding();
                writerLaser.Flush();
                byte[] bytes = ascCode.GetBytes(s);
                writerLaser.Write(bytes, 0, bytes.Length);

            }
            catch
            {
            }
        }

        private void drawLaserBackground()
        {
            Bitmap bmp = new Bitmap(pictureBoxSensor.Width, pictureBoxSensor.Height);
            Graphics g = Graphics.FromImage(bmp);

            int width = bmp.Width - 50;
            int height = bmp.Height;


            g.Clear(Color.Black);
            g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;

            int centerX = bmp.Width / 2;
            int centerY = bmp.Height - 120;
            //g.DrawLine(Pens.Blue, 0, bmp.Height- 120, bmp .Width , bmp.Height -120 );
            //g.DrawLine(Pens.Blue, bmp.Width /2, 0, bmp.Width/2 , bmp.Height- 120);



            for (int i = 0; i < 8; ++i)
            {
                g.DrawArc(Pens.Green, centerX - 25 * (i + 1), centerY - 25 * (i + 1), 25 * (i + 1) * 2, 25 * (i + 1) * 2, 150, 240);
            }

            Font drawFont = new Font("Arial", 8);
            SolidBrush drawBrush = new SolidBrush(Color.Green);


            for (int i = 0; i < 36; ++i)
            {
                float cosValue = (float)Math.Cos(10.0 * i / 180 * Math.PI);
                float sinValue = (float)Math.Sin(10.0 * i / 180 * Math.PI);
                float x = (float)(centerX + width / 2 * cosValue);
                float y = (float)(centerY - width / 2 * sinValue);

                if ((i < 22) || (i > 32))
                {
                    g.DrawLine(Pens.Green, centerX, centerY, x, y);
                }

            }

            RectangleF drawRect = new RectangleF(centerX - 25, centerY + 10, 50, 50);
            g.DrawString("1.0", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX - 45, centerY + 25, 50, 50);
            g.DrawString("2.0", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX - 65, centerY + 35, 50, 50);
            g.DrawString("3.0", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX - 85, centerY + 48, 50, 50);
            g.DrawString("4.0", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX - 110, centerY + 60, 50, 50);
            g.DrawString("5.0", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX - 130, centerY + 73, 50, 50);
            g.DrawString("6.0", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX - 150, centerY + 83, 50, 50);
            g.DrawString("7.0", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX - 170, centerY + 95, 50, 50);
            g.DrawString("8.0", drawFont, drawBrush, drawRect);

            drawRect = new RectangleF(centerX + 5, centerY + 10, 50, 50);
            g.DrawString("1.0", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX + 25, centerY + 25, 50, 50);
            g.DrawString("2.0", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX + 45, centerY + 35, 50, 50);
            g.DrawString("3.0", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX + 70, centerY + 48, 50, 50);
            g.DrawString("4.0", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX + 90, centerY + 60, 50, 50);
            g.DrawString("5.0", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX + 115, centerY + 73, 50, 50);
            g.DrawString("6.0", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX + 135, centerY + 83, 50, 50);
            g.DrawString("7.0", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX + 155, centerY + 95, 50, 50);
            g.DrawString("8.0", drawFont, drawBrush, drawRect);

            drawRect = new RectangleF(centerX - 3, centerY - 213, 50, 50);
            g.DrawString("0", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX + 27, centerY - 210, 50, 50);
            g.DrawString("-10", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX + 60, centerY - 202, 50, 50);
            g.DrawString("-20", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX + 94, centerY - 187, 50, 50);
            g.DrawString("-30", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX + 125, centerY - 165, 50, 50);
            g.DrawString("-40", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX + 153, centerY - 140, 50, 50);
            g.DrawString("-50", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX + 173, centerY - 110, 50, 50);
            g.DrawString("-60", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX + 190, centerY - 78, 50, 50);
            g.DrawString("-70", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX + 200, centerY - 43, 50, 50);
            g.DrawString("-80", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX + 203, centerY - 8, 50, 50);
            g.DrawString("-90", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX + 198, centerY + 28, 50, 50);
            g.DrawString("-100", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX + 188, centerY + 63, 50, 50);
            g.DrawString("-110", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX + 178, centerY + 93, 50, 50);
            g.DrawString("-120", drawFont, drawBrush, drawRect);

            drawRect = new RectangleF(centerX - 42, centerY - 210, 50, 50);
            g.DrawString("10", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX - 75, centerY - 202, 50, 50);
            g.DrawString("20", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX - 109, centerY - 187, 50, 50);
            g.DrawString("30", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX - 140, centerY - 165, 50, 50);
            g.DrawString("40", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX - 165, centerY - 140, 50, 50);
            g.DrawString("50", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX - 188, centerY - 110, 50, 50);
            g.DrawString("60", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX - 203, centerY - 78, 50, 50);
            g.DrawString("70", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX - 213, centerY - 43, 50, 50);
            g.DrawString("80", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX - 215, centerY - 8, 50, 50);
            g.DrawString("90", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX - 218, centerY + 28, 50, 50);
            g.DrawString("100", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX - 210, centerY + 63, 50, 50);
            g.DrawString("110", drawFont, drawBrush, drawRect);
            drawRect = new RectangleF(centerX - 195, centerY + 93, 50, 50);
            g.DrawString("120", drawFont, drawBrush, drawRect);

            pictureBoxSensor.BackgroundImage = bmp;
        }

        private void drawSensor(int len)
        {

            Bitmap sensorImge = new Bitmap(pictureBoxSensor.Width, pictureBoxSensor.Height);


            Graphics g = Graphics.FromImage(sensorImge);
            int width = sensorImge.Width - WIDTH_SUB;
            int height = sensorImge.Height;

            int centerX = sensorImge.Width / 2 - START_X, centerY = sensorImge.Height - START_Y;

            g.Clear(Color.Transparent);

            g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;

            SolidBrush dataBrush = null;
            PointF[] curvePoints = new PointF[len + 1];

            // Define fill mode.
            FillMode newFillMode = FillMode.Winding;

            curvePoints[len] = new PointF((float)centerX, (float)centerY);

            if (firstData)
            {
                firstData = false;

            }
            else
            {
                //draw old image first
                dataBrush = new SolidBrush(Color.Yellow);
                PointF[] curvePointsLast = new PointF[DISDATALEN + 1];

                for (int i = 0; i < DISDATALEN; ++i)
                {
                    float Y = (float)(centerY - ratio * (double)disDataLast[i] / 1000 * Math.Sin(startAng + stepAng * i));
                    float X = (float)(centerX + ratio * (double)disDataLast[i] / 1000 * Math.Cos(startAng + stepAng * i));
                    curvePointsLast[i] = new PointF(X, Y);
                }

                curvePointsLast[DISDATALEN] = new PointF((float)centerX, (float)centerY);

                g.FillPolygon(dataBrush, curvePointsLast, newFillMode);
            }

            dataBrush = new SolidBrush(Color.Lime);
            for (int i = 0; i < DISDATALEN; ++i)
            {
                float Y = (float)(centerY - ratio * (double)disData[i] / 1000 * Math.Sin(startAng + stepAng * i));
                float X = (float)(centerX + ratio * (double)disData[i] / 1000 * Math.Cos(startAng + stepAng * i));
                curvePoints[i] = new PointF(X, Y);
            }
            // Fill polygon to screen.

            g.FillPolygon(dataBrush, curvePoints, newFillMode);

            pictureBoxSensor.Image = sensorImge;

            Array.Copy(disData, disDataLast, DISDATALEN);

        }

        private void tmrDrawing_Tick(object sender, EventArgs e)
        {
            drawCnt ++;
            
            if (drawCnt > 0)
            {
                drawCnt = 0;
                sendCommandLaser(SCANCOMMAND ); //read data again
            }

        }


    }
}
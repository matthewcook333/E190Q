using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Runtime.InteropServices;
using EARTHLib;


namespace DrRobot.JaguarControl
{
    class GoogleEarth
    {

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

        static readonly Int32 WM_QUIT = 0x0012;


        private IntPtr GEHrender = (IntPtr)0;
        private IntPtr GEParentHrender = (IntPtr)0;

        public ApplicationGEClass googleEarth;
        public CameraInfoGE cam = null;


        public string centerLatitude = "43.855159";
        public string centerLongitude = "-79.3615177";
        public int cameraRange = 200;
        public double preEstLatitude = 0;
        public double preEstLongitude = 0;

        public double preLatitude = 0;
        public double preLongtitude = 0;
        public double curLatitude = 0;
        public double curLongitude = 0;


        public string kmlFileStr1 = @"<?xml version=""1.0"" encoding=""utf-8""?><kml xmlns=""http://www.opengis.net/kml/2.2""><Document>";
        public string kmlFileStyle = @"<Style id=""MyLineStyle""><LineStyle><color>7f0000ff</color><width>4</width></LineStyle></Style>";
        //private string kmlFileLookAt = @"<LookAt><longitude>-79.3607666</longitude><latitude>43.855015</latitude><altitude>0</altitude><range>150</range><heading>0</heading></LookAt>";

        public string kmlFilePlacemark = @"<Placemark><name>unextruded</name><styleUrl>#MyLineStyle</styleUrl><LineString><extrude>1</extrude><tessellate>1</tessellate>";
        public string kmlFileCoordinate = @"<coordinates>-79.3607666,43.855015,0 -79.3608,43.855015,0</coordinates>";
        public string kmlFileEnd = @"</LineString></Placemark></Document></kml>";


        //private string kmlFileName = System.Environment.CurrentDirectory + "\\gpstest0.kml";

        //private static double startLat = 0;
        //private static double startLongitude = 0;


        public SetColor setColor = new SetColor();

        public class SetColor
        {
            public string SetRed = "0x7f0000ff";
            public string SetGreen = "0x7f00ff00";
            public string SetBlue = "0x7fff0000";
        }

        private void btnSetStartPoint_Click(JaguarCtrl jc, object sender, EventArgs e)
        {
            cam = new CameraInfoGE();
            double centerLat = double.Parse(centerLatitude);
            double centerLong = double.Parse(centerLongitude);
            try
            {
                centerLat = double.Parse(jc.txtStartLat.Text);
            }
            catch
            {
            }
            try
            {
                centerLong = double.Parse(jc.txtStartLong.Text);
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
        private void trackBarZoom_Scroll(JaguarCtrl jc, object sender, EventArgs e)
        {
            if (cam != null)
            {
                cameraRange = jc.trackBarZoom.Value;
                cam.Range = cameraRange;
                googleEarth.SetCamera(cam, 0.6);
            }
        }

        // this function will set map center as default map start point
        private void btnSetMapCenter_Click(JaguarCtrl jc, object sender, EventArgs e)
        {
            PointOnTerrainGE pointGe = new PointOnTerrainGE();
            pointGe = googleEarth.GetPointOnTerrainFromScreenCoords(0, 0);


            double lat = pointGe.Latitude;
            double longitude = pointGe.Longitude;
            jc.txtStartLat.Text = lat.ToString();
            jc.txtStartLong.Text = longitude.ToString();

            btnSetStartPoint_Click(jc, null, null);
        }

        public void Initialize(JaguarCtrl jc)
        {

            centerLatitude = jc.jaguarSetting.GoogleEarthStartLat.ToString();
            centerLongitude = jc.jaguarSetting.GoogleEarthStartLong.ToString();
            jc.txtStartLat.Text = centerLatitude;
            jc.txtStartLong.Text = centerLongitude;
            if (true)//(jc.DesignMode == false)
            {
                googleEarth = new ApplicationGEClass();

                MessageBox.Show("Waiting for GoogleEarth to be loaded…. \n This may take a while.When the GoogleEarth is shown, click OK to continue.", "DrRobot Jaguar Control", MessageBoxButtons.OK, MessageBoxIcon.Asterisk);
                //ShowWindowAsync(googleEarth.GetMainHwnd(), 0);

                GEHrender = (IntPtr)googleEarth.GetRenderHwnd();
                GEParentHrender = GetParent(GEHrender);

                MoveWindow(GEHrender, 0, 0, jc.panelGE.Width, jc.panelGE.Height, true);
                SetParent(GEHrender, jc.panelGE.Handle);



                // don'tdo it now , we keep google earth window to clear the KML data in temporary node
                //Hide the main GE window
                //SetWindowPos(googleEarth.GetMainHwnd(), HWND_BOTTOM, 0, 0, 0, 0, SWP_NOSIZE + SWP_HIDEWINDOW); 
            }

        }

        public void Close(JaguarCtrl jc, RobotConfig robotCfg, string configFile)
        {
            double centerLat = double.Parse(centerLatitude);
            double centerLong = double.Parse(centerLongitude);
            try
            {
                centerLat = double.Parse(jc.txtStartLat.Text);
            }
            catch
            {
            }
            try
            {
                centerLong = double.Parse(jc.txtStartLong.Text);
            }
            catch
            {
            }

            jc.jaguarSetting.GoogleEarthStartLat = centerLat;
            jc.jaguarSetting.GoogleEarthStartLong = centerLong;
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

        }

    }
}

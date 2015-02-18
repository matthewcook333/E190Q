using System;
using System.ComponentModel;
using System.Windows.Forms;
using System.Collections;
using System.Drawing;
using System.Text;
using System.Data;

namespace DrRobot.JaguarControl
{
    class ArmPositionIndicatorControl : InstrumentControl
    {
        #region Fields
        //depend on the image 
        const int bgCenterX0 = 141;//230;
        const int bgCenterY0 = 185;
        const int bgCenterX1 = 312; //for rear arm
        const int bgCenterY1 = 185;

        const int armCenterX0 = 128;
        const int armCenterY0 = 32;

        const int armCenterX1 = 32;
        const int armCenterY1 = 32;


        bool set2Arm = false;
        // Parameters
        float anglePos0 = 0;
        float anglePos1 = 0;
        float angleIni0 = 0;
        float angleIni1 = 0;


        // Images
        Bitmap bmpArmPos = new Bitmap(DrRobot.JaguarControl.JaguarCtrlResource.jaguar_Arm);
        Bitmap bmpArmM0 = new Bitmap(DrRobot.JaguarControl.JaguarCtrlResource.jaguar_arm_M);
        Bitmap bmpArmM1 = new Bitmap(DrRobot.JaguarControl.JaguarCtrlResource.jaguar_arm_M1);
        #endregion

        #region Contructor

        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.Container components = null;

        public ArmPositionIndicatorControl()
        {
            // Double bufferisation
            SetStyle(ControlStyles.DoubleBuffer | ControlStyles.UserPaint |
                ControlStyles.AllPaintingInWmPaint, true);
        }

        #endregion

        #region Component Designer generated code
        /// <summary>
        /// Required method for Designer support - do not modify 
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            components = new System.ComponentModel.Container();
        }
        #endregion

        #region Paint

        protected override void OnPaint(PaintEventArgs pe)
        {
            // Calling the base class OnPaint
            base.OnPaint(pe);

            // Pre Display computings
            Point ptRotation0 = new Point(bgCenterX0, bgCenterY0);     //  background center for rotate
            Point ptimgArm0 = new Point(bgCenterX0 - armCenterX0, bgCenterY0 - armCenterY0);

            Point ptRotation1 = new Point(bgCenterX1, bgCenterY1);     //  background center for rotate
            Point ptimgArm1 = new Point(bgCenterX1 - armCenterX1, bgCenterY1 - armCenterY1);


            bmpArmPos.MakeTransparent(Color.Yellow);
            bmpArmM0.MakeTransparent(Color.Yellow);
            bmpArmM1.MakeTransparent(Color.Yellow);

            double alphaArm0 = InterpolPhyToAngle(anglePos0, 0, 360, 0, 360);
            double alphaArm1 = InterpolPhyToAngle(anglePos1, 0, 360, 0, 360);

            float scale = (float)this.Width / bmpArmPos.Width;

            // display mask
            Pen maskPen = new Pen(this.BackColor, 30 * scale);
            pe.Graphics.DrawRectangle(maskPen, 0, 0, bmpArmPos.Width * scale, bmpArmPos.Height * scale);

            // display cadran
            pe.Graphics.DrawImage(bmpArmPos, 0, 0, (float)(bmpArmPos.Width * scale), (float)(bmpArmPos.Height * scale));

            // display arm
            RotateImage(pe, bmpArmM0, alphaArm0, ptimgArm0, ptRotation0, scale);

            if (set2Arm)
            {
                RotateImage(pe, bmpArmM1, alphaArm1, ptimgArm1, ptRotation1, scale);
            }


        }

        #endregion

        #region Methods


        /// <summary>
        /// Define the physical value to be displayed on the indicator
        /// </summary>
        /// <param name="aircraftheat">The aircraft air speed in kts</param>
        public void SetArmPositionIndicatorParameters(double angleValue0, double angleValue1 )
        {
            angleValue0 = 360 + angleValue0;
            angleValue0 = angleValue0 + angleIni0;
            angleValue0 = (angleValue0 >= 360 ? angleValue0 - 360 : angleValue0);
            angleValue0 = (angleValue0 <= 0 ? 0 : angleValue0);
            anglePos0 = (float)angleValue0;     //0 - 360

            angleValue1 = -angleValue1;
            angleValue1 = 360 + angleValue1;
            angleValue1 = angleValue1 + angleIni1;
            angleValue1 = (angleValue1 >= 360 ? angleValue1 - 360 : angleValue1);
            angleValue1 = (angleValue1 <= 0 ? 0 : angleValue1);
            anglePos1 = (float)angleValue1;     //0 - 360

            this.Refresh();
        }

        public void SetArmPositionIndicatorIni(double angle0, double angle1)
        {
            if (angle0 >= 360) angle0 = 360;
            if (angle0 < 0) angle0 = 0;
            angleIni0 = (float)angle0;
            angleIni1 = (float)angle1;
        }

        public void Set2ArmCtrl(bool val)
        {
            set2Arm = val;
        }
        #endregion

        # region IDE




        # endregion

    }
}

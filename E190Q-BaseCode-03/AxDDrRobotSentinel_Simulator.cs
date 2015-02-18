using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace DrRobot.JaguarControl
{
    public class AxDDrRobotSentinel_Simulator //: AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel 
    {
        //public Navigation navigation;
        private double encoderPulseL, encoderPulseR;
        private double encoderSpeedL, encoderSpeedR;
        private double encoderDirL, encoderDirR;
        private short actuatorL, actuatorR;
        private short zeroOutput = 16383;
        private short maxPosOutput = 32767;


        public AxDDrRobotSentinel_Simulator()
        {
            //navigation = nav;
            Reset();
        }

        public void Reset()
        {
            encoderPulseL = 0;
            encoderPulseR = 0;
            encoderSpeedL = 0;
            encoderSpeedR = 0;
            actuatorL = 0;
            actuatorR = 0;
        }

        public void UpdateSensors(int deltaT)
        {
            encoderDirL = Math.Sign(actuatorL);
            encoderDirR = Math.Sign(actuatorR);
            encoderSpeedL = Math.Abs(actuatorL);
            encoderSpeedR = Math.Abs(actuatorR);
            encoderPulseL = limitEncoder(encoderPulseL + encoderSpeedL * encoderDirL * deltaT/1000.0);
            encoderPulseR = limitEncoder(encoderPulseR + encoderSpeedR * encoderDirR * deltaT/1000.0);
        }

        private double limitEncoder(double value)
        {
            if (value > 32767)
                value -= 32767;
            else if (value < 0)
                value += 32767;
            return value;
        }

        public int GetEncoderPulse4()
        {
            return (int)encoderPulseL;
        }

        public int GetEncoderSpeed4()
        {
            return (int) encoderSpeedL;
        }

        public int GetEncoderDir4()
        {
            return (int) encoderDirL;
        }


        public int GetEncoderPulse5()
        {
            return (int)encoderPulseR;
        }

        public int GetEncoderSpeed5()
        {
            return (int) encoderSpeedR;
        }

        public int GetEncoderDir5()
        {
            return (int) encoderDirR;
        }

        public void DcMotorVelocityNonTimeCtrAll(short channel1, short channel2, short channel3, short channel4, short channel5, short channel6)
        {
            actuatorL = channel4;
            actuatorR = channel5;
        }

        public void DcMotorPwmNonTimeCtrAll(short channel1, short channel2, short channel3, short channel4, short channel5, short channel6)
        {
            actuatorL = (short)(350 * (channel4 - zeroOutput) / (maxPosOutput - zeroOutput));
            actuatorR = (short)(350 * (channel5 - zeroOutput) / (maxPosOutput - zeroOutput));
        }

        public void DcMotorPwmNonTimeCtr(short channelNum, short forwardPWM)
        {
            if (channelNum == 4)
                actuatorL = (short)(350*(forwardPWM - zeroOutput) / (maxPosOutput-zeroOutput));
            else if (channelNum == 5)
                actuatorR = (short)(350 * (forwardPWM - zeroOutput) / (maxPosOutput - zeroOutput));
        }
    }
}

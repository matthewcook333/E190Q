using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace DrRobot.JaguarControl
{
    class Robot
    {
        float x;
        float y;
        float t;

        public Robot()
        {
            x = initialX;
            y = initialY;
            t = initialT;
            Initialize();
        }
        public Robot(float _x, float _y, float _t)
        {
            x = _x;
            y = _y;
            t = _t;
            Initialize();
        }
        public void Initialize()
        {
            x_est = initialX;
	        y_est = initialY;
	        t_est = initialT;
						
	        // Set desired state
	        desiredX = initialX;
	        desiredY = initialY;
	        desiredT = initialT;

	        // Reset Localization Variables
	        currentEncoderPulse1 = 0;
	        currentEncoderPulse2 = 0;
	        lastEncoderPulse1 = 0;
	        lastEncoderPulse2 = 0;
	        wheelDistanceR = 0;
	        wheelDistanceL = 0;

            // Set default to simulator mode
	        robotType = ROBOT_TYPE_SIMULATED;

	        // Stop all experiments by default
	        controllerType = CONTROLLERTYPE_MANUALCONTROL;

	        // Set random start for particles
	        InitializeParticles();

	        // Set default to no motionPlanRequired
	        motionPlanRequired = false;

	        // Set visual display
	        tiltAngle = 25.0;
	        zoom = 2.0;
	        displayParticles = true;
	        displayNodes = true;
	        displaySimRobot = true;
    }
}

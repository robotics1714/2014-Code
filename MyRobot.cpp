#include "WPILib.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	RobotDrive* myRobot; // robot drive system
	Joystick* leftStick;
	Joystick* rightStick;

public:
	RobotDemo()
	{
		myRobot = new RobotDrive(1, 2);
		leftStick = new Joystick(1);
		rightStick = new Joystick(2);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous()
	{
		GetWatchdog().SetEnabled(false);
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl()
	{
		GetWatchdog().SetEnabled(true);
		while (IsOperatorControl())
		{
			myRobot->TankDrive(leftStick, rightStick);
			GetWatchdog().Feed();
			Wait(0.005);				// wait for a motor update time
		}
	}
	
	/**
	 * Runs during test mode
	 */
	void Test() {

	}
};

START_ROBOT_CLASS(RobotDemo);


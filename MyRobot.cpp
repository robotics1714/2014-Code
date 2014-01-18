#include "WPILib.h"
#include "RaspberryPi.h"
#include "GlobalDefines.h"
#include "Intake.h"

//Ports for the Intake TODO Change port numbers to real values
#define INTAKE_ROLLER_PORT 1
#define INTAKE_LEFT_PIVOT_PORT 1
#define INTAKE_RIGHT_PIVOT_PORT 1
#define INTAKE_UPPER_LIMIT_PORT 1
#define INTAKE_LOWER_LIMIT_PORT 1
#define INTAKE_POSITION_POT_PORT 1

class RobotDemo : public SimpleRobot
{
	RobotDrive* myRobot; // robot drive system
	Joystick* leftStick;
	Joystick* rightStick;
	
	Intake* intake;

	//Camera tracking objects
	RaspberryPi* rpi;
	Relay* LEDLight;
	
	DriverStationLCD* lcd;

public:
	RobotDemo()
	{
		myRobot = new RobotDrive(1, 2);
		leftStick = new Joystick(1);
		rightStick = new Joystick(2);
		
		intake = new Intake(INTAKE_ROLLER_PORT, INTAKE_LEFT_PIVOT_PORT, INTAKE_RIGHT_PIVOT_PORT,
				INTAKE_UPPER_LIMIT_PORT, INTAKE_LOWER_LIMIT_PORT, INTAKE_POSITION_POT_PORT);
		
		rpi = new RaspberryPi("17140");
		LEDLight = new Relay(1);
		LEDLight->Set(Relay::kForward);
		
		lcd = DriverStationLCD::GetInstance();
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
		while (IsOperatorControl() && !IsDisabled())
		{
			//myRobot->TankDrive(leftStick, rightStick);
			rpi->Read();
			lcd->Clear();
			if(rpi->GetXPos() != -2)
			{
				lcd->Printf(DriverStationLCD::kUser_Line1, 1, "x: %i", rpi->GetXPos());
			}
			else
			{
				lcd->Printf(DriverStationLCD::kUser_Line1, 1, "  not  ");
			}
			if(rpi->GetYPos() != -2)
			{
				lcd->Printf(DriverStationLCD::kUser_Line2, 1, "y: %i", rpi->GetYPos());
			}
			else
			{
				lcd->Printf(DriverStationLCD::kUser_Line2, 1, "   not   ");
			}

			lcd->UpdateLCD();
			GetWatchdog().Feed();
			Wait(0.005);				// wait for a motor update time
		}
	}

	void Disabled()
	{
		while(IsDisabled())
		{
			rpi->Read();
			lcd->Clear();
			if(rpi->GetXPos() != -2)
			{
				lcd->Printf(DriverStationLCD::kUser_Line1, 1, "x: %i", rpi->GetXPos());
			}
			else
			{
				lcd->Printf(DriverStationLCD::kUser_Line1, 1, "  not  ");
			}
			if(rpi->GetYPos() != -2)
			{
				lcd->Printf(DriverStationLCD::kUser_Line2, 1, "y: %i", rpi->GetYPos());
			}
			else
			{
				lcd->Printf(DriverStationLCD::kUser_Line2, 1, "   not   ");
			}

			lcd->UpdateLCD();
		}
	}

	/**
	 * Runs during test mode
	 */
	void Test() {

	}
};

START_ROBOT_CLASS(RobotDemo);


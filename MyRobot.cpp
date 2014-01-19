#include <cmath>
#include "WPILib.h"
#include "RaspberryPi.h"
#include "GlobalDefines.h"
#include "Intake.h"
#include "Catapult.h"

//Ports for the Intake TODO Change port numbers to real values
#define INTAKE_ROLLER_PORT 1
/*#define INTAKE_LEFT_PIVOT_PORT 1
#define INTAKE_RIGHT_PIVOT_PORT 1
#define INTAKE_UPPER_LIMIT_PORT 1
#define INTAKE_LOWER_LIMIT_PORT 1
#define INTAKE_POSITION_POT_PORT 1*/

//Ports for the Catapult TODO Change port number to real values
#define LOADING_MOTOR_PORT 1
#define HOLDING_MOTOR_PORT 1
#define LOADED_LIMIT_PORT 1
#define LOADING_ENCO_PORT_1 1
#define LOADING_ENCO_PORT_2 1
#define HOLDING_ENCO_PORT_1 1
#define HOLDING_ENCO_PORT_2 1

#define GYRO_PORT 1

#define LEFT_ENCO_PORT_1 1
#define LEFT_ENCO_PORT_2 1
#define RIGHT_ENCO_PORT_1 1
#define RIGHT_ENCO_PORT_2 1

//Definitions for the different autonomous modes
#define ONE_BALL_AUTON 1
#define TWO_BALL_AUTON 2

//Autonomous step name definitions
#define AUTON_ONE_DRIVE_FORWARDS 1
#define AUTON_ONE_SHOOT 2
#define AUTON_END 0

#define RPI_ERROR_VALUE -2 //The value the Raspberry Pi class sends when there is an error

class RobotDemo : public SimpleRobot
{
	RobotDrive* myRobot; // robot drive system
	Encoder* leftEnco;
	Encoder* rightEnco;
	Joystick* leftStick;
	Joystick* rightStick;

	Gyro* gyro;

	//Manipulators
	Intake* intake;
	Catapult* catapult;
	
	//Camera tracking objects
	RaspberryPi* rpi;
	Relay* LEDLight;

	int autonMode;
	int autonStep;

	DriverStationLCD* lcd;

public:
	RobotDemo()
	{
		myRobot = new RobotDrive(1, 2);
		leftEnco = new Encoder(LEFT_ENCO_PORT_1, LEFT_ENCO_PORT_2);
		rightEnco = new Encoder(RIGHT_ENCO_PORT_1, RIGHT_ENCO_PORT_2);
		leftEnco->SetDistancePerPulse(1);//TODO change this to the real number
		rightEnco->SetDistancePerPulse(1);
		leftStick = new Joystick(1);
		rightStick = new Joystick(2);

		gyro = new Gyro(GYRO_PORT);

		intake = new Intake(INTAKE_ROLLER_PORT);
		catapult = new Catapult(LOADING_MOTOR_PORT, HOLDING_MOTOR_PORT, LOADED_LIMIT_PORT,
				LOADING_ENCO_PORT_1, LOADING_ENCO_PORT_2, HOLDING_ENCO_PORT_1, HOLDING_ENCO_PORT_2);

		rpi = new RaspberryPi("17140");
		LEDLight = new Relay(1);
		LEDLight->Set(Relay::kForward);

		autonMode = ONE_BALL_AUTON;
		autonStep = AUTON_ONE_DRIVE_FORWARDS;

		lcd = DriverStationLCD::GetInstance();
	}

	//robot turns to desired position with a deadband of 2 degrees in each direction
	bool GyroTurn (float desiredTurnAngle, float turnSpeed)
	{
		bool turning = true;
		float myAngle = gyro->GetAngle();
		//normalizes angle from gyro to [-180,180) with zero as straight ahead
		while(myAngle >= 180)
		{
			GetWatchdog().Feed();
			myAngle = myAngle - 360;
		}
		while(myAngle < -180)
		{
			GetWatchdog().Feed();
			myAngle = myAngle + 360;
		}
		if(myAngle < (desiredTurnAngle - 2))// if robot is too far left, turn right a bit
		{
			myRobot->Drive(turnSpeed, -turnSpeed); //(right,left)
		}
		if(myAngle > (desiredTurnAngle + 2))// if robot is too far right, turn left a bit
		{
			myRobot->Drive(-turnSpeed, turnSpeed); //(right,left)
		}
		else
		{
			myRobot->Drive(0, 0);
			turning = false;
		}

		return turning;
	}

	bool GyroDrive(float desiredDriveAngle, float speed, int desiredDistance)
	{
		bool driving = true;
		double encoderInchesTraveled = fabs(leftEnco->GetDistance());//absolute value distance
		float myAngle = gyro->GetAngle();
		//normalizes angle from gyro to [-180,180) with zero as straight ahead
		while(myAngle >= 180)
		{
			GetWatchdog().Feed();
			myAngle = myAngle - 360;
		}
		while(myAngle < -180)
		{
			GetWatchdog().Feed();
			myAngle = myAngle + 360;
		}

		float my_speed = 0.0;
		float turn = 0.0;

		if(speed > 0)
			//30.0 is the number you have to change to adjust properly
			turn = -((myAngle + desiredDriveAngle) / 30.0); //proportionally adjust turn. As the robot gets more off 0, the greater the turn will be
		else
			turn = (myAngle + desiredDriveAngle) / 30.0; //proportionally adjust turn. As the robot gets more off 0, the greater the turn will be

		if (encoderInchesTraveled < desiredDistance)
			my_speed = speed;
		else
		{
			my_speed = 0.0;
			driving = false;
		}

		myRobot->Drive(my_speed, turn);

		return driving;
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous()
	{
		GetWatchdog().SetEnabled(false);
		Timer* shootTimer = new Timer();
		bool goalFound = false;
		shootTimer->Reset();
		
		while(IsAutonomous() && !IsDisabled())
		{
			rpi->Read();
			if(autonMode == ONE_BALL_AUTON)
			{
				switch(autonStep)
				{
				case AUTON_ONE_DRIVE_FORWARDS:
					//Drive forwards into the alliance zone and start the shoot timer
					shootTimer->Start();
					if(!GyroDrive(0, 1, 48))
					{
						autonStep = AUTON_ONE_SHOOT;
					}
					break;
				case AUTON_ONE_SHOOT:
					//Shoot the ball if the goal is hot or 6 seconds passes
					if(!goalFound)
					{
						//This is put into an if statement to protect against the 
						//rpi finding the hot goal and then losing it
						goalFound = ((rpi->GetXPos() != RPI_ERROR_VALUE) &&
								(rpi->GetYPos() != RPI_ERROR_VALUE));
					}
					if((goalFound) ||(shootTimer->Get() >= 6))
					{
						if(!catapult->Shoot())
						{
							autonStep = AUTON_END;
						}
					}
					break;
				case AUTON_END:
					break;
				}
			}
		}
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

			lcd->Printf(DriverStationLCD::kUser_Line1, 1, "x: %i", rpi->GetXPos());
			lcd->Printf(DriverStationLCD::kUser_Line2, 1, "y: %i", rpi->GetYPos());

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
			if(rpi->GetXPos() != RPI_ERROR_VALUE)
			{
				lcd->Printf(DriverStationLCD::kUser_Line1, 1, "x: %i", rpi->GetXPos());
			}
			else
			{
				lcd->Printf(DriverStationLCD::kUser_Line1, 1, "  not  ");
			}
			if(rpi->GetYPos() != RPI_ERROR_VALUE)
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


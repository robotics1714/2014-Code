#include <cmath>
#include "WPILib.h"
#include "RaspberryPi.h"
#include "GlobalDefines.h"
#include "Intake.h"
#include "Catapult.h"

//Ports for the Intake TODO Change port numbers to real values
#define INTAKE_ROLLER_PORT 1
#define BALL_SENSOR_PORT 1
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
#define HOLDING_POT_PORT 1

#define GYRO_PORT 1

#define LEFT_ENCO_PORT_1 1
#define LEFT_ENCO_PORT_2 1
#define RIGHT_ENCO_PORT_1 1
#define RIGHT_ENCO_PORT_2 1

//Definitions for the different autonomous modes
#define ONE_BALL_AUTON 1
#define TWO_BALL_AUTON 2

//One ball autonomous step name definitions
#define AUTON_ONE_RELOAD 1
#define AUTON_ONE_SHOOT 2
#define AUTON_ONE_WAIT 3
#define AUTON_ONE_DRIVE_FORWARDS 4
#define AUTON_END 0

//Two ball autonomous step name definitions
#define AUTON_TWO_RELOAD 1
#define AUTON_TWO_FIRST_TURN 2
#define AUTON_TWO_FIRST_SHOOT 3
#define AUTON_TWO_FIRST_WAIT 4
#define AUTON_TWO_SECOND_TURN 5
#define AUTON_TWO_GET_SECOND_BALL 6
#define AUTON_TWO_THIRD_TURN 7
#define AUTON_TWO_SECOND_SHOOT 8
#define AUTON_TWO_SECOND_WAIT 9
#define AUTON_TWO_DRIVE_FORWARDS 10

//Defines for SmartDashboard Keys
#define NUM_BALL_AUTO "Number of Balls in Auto"

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
		//Initialize the objects for the drive train
		myRobot = new RobotDrive(1, 2);
		leftEnco = new Encoder(LEFT_ENCO_PORT_1, LEFT_ENCO_PORT_2);
		rightEnco = new Encoder(RIGHT_ENCO_PORT_1, RIGHT_ENCO_PORT_2);
		leftEnco->SetDistancePerPulse(1);//TODO change this to the real number
		rightEnco->SetDistancePerPulse(1);
		leftStick = new Joystick(1);
		rightStick = new Joystick(2);

		//Initialize the gyro
		gyro = new Gyro(GYRO_PORT);
		gyro->Reset();

		//Initialize the manipulators
		intake = new Intake(INTAKE_ROLLER_PORT, BALL_SENSOR_PORT);
		catapult = new Catapult(LOADING_MOTOR_PORT, HOLDING_MOTOR_PORT, LOADED_LIMIT_PORT,
				LOADING_ENCO_PORT_1, LOADING_ENCO_PORT_2, HOLDING_POT_PORT);

		//Initialize the objects needed for camera tracking
		rpi = new RaspberryPi("17140");
		LEDLight = new Relay(1);
		LEDLight->Set(Relay::kForward);

		//Set the autonomous modes
		autonMode = ONE_BALL_AUTON;
		autonStep = AUTON_ONE_SHOOT;

		//Initialize the lcd
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

	void Autonomous()
	{
		GetWatchdog().SetEnabled(false);
		Timer* hotGoalTimer = new Timer();
		Timer* reloadTimer = new Timer();
		bool goalFound = false;
		bool rightSideHot = false;
		hotGoalTimer->Reset();
		hotGoalTimer->Start();
		gyro->Reset();

		//Find out the type of autonomous we are using
		int autonType = (int)SmartDashboard::GetNumber(NUM_BALL_AUTO);
		if(autonType == 2)//Set the auton mode to two if we are doing a two ball auto
		{
			autonMode = TWO_BALL_AUTON;
			autonStep = AUTON_TWO_RELOAD;
			catapult->StartLoad();
		}
		else//Set the auton to one if the value on SD is set to 1 or another random value
		{
			autonMode = ONE_BALL_AUTON;
			autonStep = AUTON_ONE_RELOAD;
			catapult->StartLoad();
		}

		while(IsAutonomous() && !IsDisabled())
		{
			rpi->Read();
			if(autonMode == ONE_BALL_AUTON)
			{
				switch(autonStep)
				{
				case AUTON_ONE_RELOAD:
					//Reload the catapult and find the hot goal
					if(!goalFound)
					{
						//This is put into an if statement to protect against the 
						//rpi finding the hot goal and then losing it
						goalFound = ((rpi->GetXPos() != RPI_ERROR_VALUE) &&
								(rpi->GetYPos() != RPI_ERROR_VALUE));
					}
					if(!(bool)catapult->Load() && (goalFound || hotGoalTimer->Get() >= 5.25))
					{
						autonStep = AUTON_ONE_SHOOT;
					}
					break;
				case AUTON_ONE_SHOOT:
					//Shoot the catapult
					if(!catapult->ReleaseHold())
					{
						//Move on to the next step when the catapult is released
						autonStep = AUTON_ONE_WAIT;
						//Start the wait timer
						reloadTimer->Reset();
						reloadTimer->Start();
					}
					break;
				case AUTON_ONE_WAIT:
					if(reloadTimer->Get() >= CATAPULT_WAIT_TIME)
					{
						autonStep = AUTON_ONE_DRIVE_FORWARDS;
						reloadTimer->Stop();
						//Start reloading the catapult
						catapult->StartLoad();
					}
					break;
				case AUTON_ONE_DRIVE_FORWARDS:
					//Drive forwards into the alliance zone and reload the catapult
					if((!GyroDrive(0, 0.75, 48)) && (!(bool)catapult->Load()))
					{
						autonStep = AUTON_END;
					}
					break;
				case AUTON_END:
					break;
				}
			}
			else if(autonMode == TWO_BALL_AUTON)
			{
				switch(autonStep)
				{
				case AUTON_TWO_RELOAD:
					//Determine if the hot goal is on the right
					if((rpi->GetXPos() != RPI_ERROR_VALUE) && 
							(rpi->GetYPos() != RPI_ERROR_VALUE))
					{
						rightSideHot = true;
					}
					//Reload the catapult
					if(!catapult->Load())
					{
						autonStep = AUTON_TWO_FIRST_TURN;
					}
					/*if((goalFound))
					{
						//If the goal is found, the right side is hot and we can go to the next step
						rightSideHot = true;
						autonStep = AUTON_TWO_FIRST_SHOOT;
					}
					else if(hotGoalTimer->Get() >= 0.5)
					{
						//If the timer runs of, the right side is not hot and we can go to the next step
						rightSideHot = false;
						autonStep = AUTON_TWO_FIRST_TURN;
					}*/
					break;
				case AUTON_TWO_FIRST_TURN:
					//Turn to the left 5* if the right goal is not hot
					if(!rightSideHot)
					{
						if(!GyroTurn(-5, 0.5))
						{
							autonStep = AUTON_TWO_FIRST_SHOOT;
						}
					}
					else
					{
						autonStep = AUTON_TWO_FIRST_SHOOT;
					}
					break;
				case AUTON_TWO_FIRST_SHOOT:
					//Release the catapult to shoot
					if(!catapult->ReleaseHold())
					{
						autonStep = AUTON_TWO_FIRST_WAIT;
						reloadTimer->Reset();
						reloadTimer->Start();
					}
					break;
				case AUTON_TWO_FIRST_WAIT:
					if(reloadTimer->Get() >= CATAPULT_WAIT_TIME)
					{
						autonStep = AUTON_TWO_SECOND_TURN;
						catapult->StartLoad();
						reloadTimer->Stop();
					}
					break;
				case AUTON_TWO_SECOND_TURN:
					//Turn the robot so it's facing forwards and reload the catapult
					if((!GyroTurn(0, 0.5)) && (!(bool)catapult->Load()))
					{
						autonStep = AUTON_TWO_GET_SECOND_BALL;
					}
					break;
				case AUTON_TWO_GET_SECOND_BALL:
					//Start up the intake and drive back to pick up the second ball
					intake->RollIn();
					if(!GyroDrive(0, -0.5, -12))
					{
						autonStep = AUTON_TWO_THIRD_TURN;
						leftEnco->Reset();
						rightEnco->Reset();
					}
					break;
				case AUTON_TWO_THIRD_TURN:
					//If the right goal was originally hot, turn left
					if(rightSideHot)
					{
						if(!GyroTurn(-5, 0.5))
						{
							autonStep = AUTON_TWO_SECOND_SHOOT;
						}
					}
					else
					{
						autonStep = AUTON_TWO_SECOND_SHOOT;
					}
					break;
				case AUTON_TWO_SECOND_SHOOT:
					intake->Stop();
					if(!catapult->ReleaseHold())
					{
						autonStep = AUTON_TWO_SECOND_WAIT;
						reloadTimer->Reset();
						reloadTimer->Start();
					}
					break;
				case AUTON_TWO_SECOND_WAIT:
					if(reloadTimer->Get() >= CATAPULT_WAIT_TIME)
					{
						autonStep = AUTON_TWO_DRIVE_FORWARDS;
						catapult->StartLoad();
						reloadTimer->Stop();
					}
					break;
				case AUTON_TWO_DRIVE_FORWARDS:
					if(!GyroDrive(0, 0.75, 60) && (!(bool)catapult->Load()))
					{
						autonStep = AUTON_END;
					}
					break;
				case AUTON_END:
					break;
				}
			}
		}
	}

	void OperatorControl()
	{
		GetWatchdog().SetEnabled(true);
		while (IsOperatorControl() && !IsDisabled())
		{
			myRobot->TankDrive(leftStick, rightStick);
			rpi->Read();
			lcd->Clear();
			lcd->Printf(DriverStationLCD::kUser_Line1, 1, "L: %f", leftEnco->GetDistance());
			lcd->Printf(DriverStationLCD::kUser_Line2, 1, "R: %f", rightEnco->GetDistance());

			//Driver controls
			//Right trigger shoots the catapult
			if(rightStick->GetRawButton(1))
			{
				catapult->StartShoot();
			}

			//Move the intake in if the R-2 trigger is pressed
			if(rightStick->GetRawButton(2))
			{
				intake->RollIn();
			}
			//Move it out if the R-5 trigger is pressed
			else if(rightStick->GetRawButton(5))
			{
				intake->RollOut();
			}
			//Intake the ball only far enough for a pass if R-3 is pressed
			else if(rightStick->GetRawButton(3))
			{
				intake->GetBallForPass();
			}
			//If none of them are pressed, stop the intake
			else
			{
				intake->Stop();
			}


			//If the right-6 button and l-10 button are pressed, stop the catapult
			if(rightStick->GetRawButton(6) && leftStick->GetRawButton(10))
			{
				catapult->Stop();
			}

			//These functions need to called all of the time, but don't do anything until
			//Their start method is called
			catapult->Shoot();
			catapult->Load();

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

#include "Intake.h"

Intake::Intake(int rollerPort, int ballSensorPort)
{
	roller = new Talon(rollerPort);
	ballSensor = new DigitalInput(ballSensorPort);
	/*pivotL = new Victor(pivotLPort);
	pivotR = new Victor(pivotRPort);
	upperLimit = new DigitalInput(upperLimitPort);
	lowerLimit = new DigitalInput(lowerLimitPort);
	positionPot = new AnalogChannel(positionPotPort);*/
}

Intake::~Intake()
{
	delete roller;
	delete ballSensor;
	/*delete pivotL;
	delete pivotR;
	delete upperLimit;
	delete lowerLimit;
	delete positionPot;*/
}

/*void Intake::Move(float speed)
{
	//check if the intake is moving up
	if(speed > 0)
	{
		//checks to see if the upper limit switch is released
		if(upperLimit->Get() == RELEASED)
		{
			//move the intake up
			pivotL->Set(speed);
			pivotR->Set(speed);
		}
		//when the intake hits the upper limit switch, it makes it stop
		else
		{
			Stop();
		}

	}
	//check if the intake is moving down
	else if(speed < 0)
	{
		//checks to see if the lower limit switch is released
		if(lowerLimit->Get() == RELEASED)
		{
			//move the intake down
			pivotL->Set(speed);
			pivotR->Set(speed);
		}
		//when the intake hits the lower limit switch, it makes it stop
		else
		{
			Stop();
		}
	}
	//if the speed is 0, stop
	else
	{
		Stop();
	}


}

void Intake::MoveToPosition(float pos)
{
	//Check to see if the requested position is greater than the current position
	if(pos > GetPos())
	{
		//Move forwards
		Move(FULL_FORWARDS);
	}
	//If the requested position is less than the current position
	else
	{
		//Move downwards
		Move(FULL_BACKWARDS);
	}

	float Intake::GetPos(void)
	{
		return positionPot->GetAverageVoltage();
	}
}*/

/*
 * void RollIn():
 * Move the roller in so the ball goes in the robot
 */
void Intake::RollIn(void)
{
	//make roller motor move at full speed to pick up the ball
	roller->Set(FULL_FORWARDS);
}

/*
 * void RollOut():
 * Move the intake out to remove a ball from the intake
 */
void Intake::RollOut(void)
{
	//make the roller move full speed backwards to expell the ball
	roller->Set(FULL_BACKWARDS);
}

/*
 * void GetBallForPass():
 * Move the intake to load a ball until there is a ball in the intake
 */
void Intake::GetBallForPass(void)
{
	//Roll the intake if there is no ball in the intake
	if(ballSensor->Get() == RELEASED)
	{
		RollIn();
	}
	else
	{
		Stop();
	}
}

void Intake::Stop(void)
{
	//Stop all of the motors
	roller->Set(STOPPED);
	/*pivotL->Set(STOPPED);
	pivotR->Set(STOPPED);*/
}

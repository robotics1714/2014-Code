#include "Intake.h"

Intake::Intake(int rollerPort, int pivotLPort, int pivotRPort, 
		int upperLimitPort, int lowerLimitPort, int positionPotPort)
{
	roller = new Victor(rollerPort);
	pivotL = new Victor(pivotLPort);
	pivotR = new Victor(pivotRPort);
	upperLimit = new DigitalInput(upperLimitPort);
	lowerLimit = new DigitalInput(lowerLimitPort);
	positionPot = new AnalogChannel(positionPotPort);
}

Intake::~Intake()
{
	delete roller;
	delete pivotL;
	delete pivotR;
	delete upperLimit;
	delete lowerLimit;
	delete positionPot;
}

void Intake::Move(float speed)
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
}

void Intake::RollIn(float speed) 
{
	//make roller motor move at SPEED
	roller->Set(speed);
}

void Intake::Stop()
{
	//Stop all of the motors
	roller->Set(STOPPED);
	pivotL->Set(STOPPED);
	pivotR->Set(STOPPED);
}

float Intake::GetPos()
{
	return positionPot->GetAverageVoltage();
}

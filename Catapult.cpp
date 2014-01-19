#include "Catapult.h"

Catapult::Catapult(int loadingMotorPort, int holdingMotorPort, int loadedLimitPort, 
		int loadingEncoPort1, int loadingEncoPort2, int holdingEncoPort1, int holdingEncoPort2)
{
	//Initialize the components
	loadingMotor = new Talon(loadingMotorPort);
	holdingMotor = new Talon(holdingMotorPort);
	loadedLimit = new DigitalInput(loadedLimitPort);
	loadingEnco = new Encoder(loadingEncoPort1, loadingEncoPort2);
	holdingEnco = new Encoder(holdingEncoPort1, holdingEncoPort2);
	
	//Reset the encoder
	loadingEnco->Reset();
	holdingEnco->Reset();
}

Catapult::~Catapult()
{
	delete loadingMotor;
	delete holdingMotor;
	delete loadedLimit;
	delete loadingEnco;
	delete holdingEnco;
}

/*
 * bool Hold():
 * Move the holding motor to hold down the catapult
 * Returns: true if it's in the process of going into hold position, false if it's finished
 */
bool Catapult::Hold(void)
{
	//Move the holding motor until it gets to the hold position
	if(holdingEnco->GetDistance() < HOLD_MOTOR_HOLD_POS)
	{
		holdingMotor->Set(FULL_FORWARDS);
		return true;
	}
	else
	{
		holdingMotor->Set(STOPPED);
		return false;
	}
}

/*
 * bool ReleaseHold():
 * Move the holding motor to allow the catapult to release
 * Returns: true if it's in the process of going into release position, false it's finished
 */
bool Catapult::ReleaseHold(void)
{
	if(holdingEnco->GetDistance() > HOLD_MOTOR_RELEASE_POS)
	{
		holdingMotor->Set(FULL_BACKWARDS);
		return true;
	}
	else
	{
		holdingMotor->Set(STOPPED);
		return false;
	}
}

bool Catapult::Load()
{
	return false;
}

bool Catapult::Shoot(void)
{
	return false;
}

double Catapult::GetLoadingDist(void)
{
	return loadingEnco->GetDistance();
}

double Catapult::GetHoldingDist(void)
{
	return holdingEnco->GetDistance();
}

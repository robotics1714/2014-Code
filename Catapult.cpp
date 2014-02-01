#include "Catapult.h"

Catapult::Catapult(int loadingMotorPort, int holdingMotorPort, int loadedLimitPort, 
		int loadingEncoPort1, int loadingEncoPort2, int holdingPotPort)
{
	//Initialize the components
	loadingMotor = new Talon(loadingMotorPort);
	holdingMotor = new Talon(holdingMotorPort);
	loadedLimit = new DigitalInput(loadedLimitPort);
	loadingEnco = new Encoder(loadingEncoPort1, loadingEncoPort2);
	holdingPot = new AnalogChannel(holdingPotPort);
	waitTimer = new Timer();
	waitTimer->Reset();
	
	loadingState = IDLE_STATE;
	shootingState = IDLE_STATE;

	//Reset the encoder
	loadingEnco->Reset();
}

Catapult::~Catapult()
{
	delete loadingMotor;
	delete holdingMotor;
	delete loadedLimit;
	delete loadingEnco;
	delete holdingPot;
	delete waitTimer;
}

/*
 * bool Hold():
 * Move the holding motor to hold down the catapult
 * Returns: true if it's in the process of going into hold position, false if it's finished
 */
bool Catapult::Hold(void)
{
	//Move the holding motor until it gets to the hold position
	if(holdingPot->GetAverageVoltage() > HOLD_MOTOR_HOLD_POS)
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
	if(holdingPot->GetAverageVoltage() < HOLD_MOTOR_RELEASE_POS)
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

void Catapult::StartLoad(void)
{
	if((shootingState != SHOOT_RELOAD) && (loadingState == IDLE_STATE))
	{
		loadingState = LOAD_PULL_BACK;
	}
}

/*
 * int Load():
 * Bring the catapult down into the loaded position
 * Returns: the current loadingState
 */
int Catapult::Load(void)
{
	switch(loadingState)
	{
	//Step 1: Bring the catapult back to the limit
	case LOAD_PULL_BACK:
		loadingMotor->Set(FULL_FORWARDS);
		if((loadedLimit->Get() == PRESSED))
		{
			loadingMotor->Set(STOPPED);
			loadingState = LOAD_HOLD;
		}
		break;
	//Step 2: Hold the catapult in place
	case LOAD_HOLD:
		if(!Hold())
		{
			loadingState = LOAD_RELEASE_TENSION;
		}
		break;
	//Step 3: Move the loading catapult back to allow the catapult to be released freely
	case LOAD_RELEASE_TENSION:
		loadingMotor->Set(FULL_BACKWARDS);
		if(loadingEnco->GetDistance() <= LOAD_MOTOR_RELEASED)
		{
			loadingMotor->Set(STOPPED);
			loadingState = IDLE_STATE;
		}
		break;
	default:
		break;
	}
	return loadingState;
}

void Catapult::StartShoot(void)
{
	if((shootingState == IDLE_STATE) && (loadingState == IDLE_STATE))
	{
		shootingState = SHOOT_RELEASE;
	}
}

/*
 * int Shoot():
 * Release the catapult and then reload it
 * Returns: The current shootingState
 */
int Catapult::Shoot(void)
{
	switch(shootingState)
	{
	//Step 1: Release the holding motor
	case SHOOT_RELEASE:
		if(!ReleaseHold())
		{
			shootingState = SHOOT_WAIT;
			//Prepare the wait timer for the waiting step
			waitTimer->Reset();
			waitTimer->Start();
		}
		break;
	//Step 2: Wait for the catapult to finish shooting
	case SHOOT_WAIT:
		if(waitTimer->Get() >= CATAPULT_WAIT_TIME)
		{
			StartLoad();
			waitTimer->Stop();
			shootingState = SHOOT_RELOAD;
		}
		break;
	//Step 3: Re-Loading the catapult
	case SHOOT_RELOAD:
		if(!((bool)Load()))
		{
			shootingState = IDLE_STATE;
		}
		break;
	default:
		break;
	}
	
	return shootingState;
}

//Returns the encoder value of the loading motor
double Catapult::GetLoadingDist(void)
{
	return loadingEnco->GetDistance();
}

//Returns the pot value of the holding motor
double Catapult::GetHoldingDist(void)
{
	return holdingPot->GetAverageVoltage();
}

//Returns whether or not the loading limit is pressed
int Catapult::GetLoadedLimit(void)
{
	return loadedLimit->Get();
}

//Manually moves the holding motor
void Catapult::MoveHoldingMotor(float speed)
{
	holdingMotor->Set(speed);
}

//Manually moves the loading motor and stops it if it hits the limit switch
void Catapult::MoveLoadingMotor(float speed)
{
	//Check for if the catapult will hit the limit switch
	if((speed < 0) && (loadedLimit->Get() == PRESSED))//TODO check whether negative speed moves the catapult down
	{
		loadingMotor->Set(STOPPED);
	}
	else
	{
		holdingMotor->Set(speed);
	}
}

//Stops EVERYTHING
void Catapult::Stop(void)
{
	loadingMotor->Set(STOPPED);
	holdingMotor->Set(STOPPED);
	loadingState = IDLE_STATE;
	shootingState = IDLE_STATE;
}

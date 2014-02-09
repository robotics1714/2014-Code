#include "Catapult.h"

Catapult::Catapult(int loadingMotorPort, int holdingMotorPort, int loadedLimitPort, 
		int holdingLimitPort)
{
	//Initialize the components
	loadingMotor = new Talon(loadingMotorPort);
	holdingMotor = new Talon(holdingMotorPort);
	loadedLimit = new DigitalInput(loadedLimitPort);
	holdingLimit = new DigitalInput(holdingLimitPort);
	//loadingEnco = new Encoder(loadingEncoPort1, loadingEncoPort2);
	//holdingPot = new AnalogChannel(holdingPotPort);
	waitTimer = new Timer();
	waitTimer->Reset();
	unwindTimer = new Timer();
	unwindTimer->Reset();
	releaseTimer = new Timer();
	releaseTimer->Reset();
	
	loadingState = IDLE_STATE;
	shootingState = IDLE_STATE;
	
	releasing = false;
}

Catapult::~Catapult()
{
	delete loadingMotor;
	delete holdingMotor;
	delete loadedLimit;
	delete holdingLimit;
	//delete loadingEnco;
	//delete holdingPot;
	delete waitTimer;
	delete unwindTimer;
	delete releaseTimer;
}

/*
 * bool Hold():
 * Move the holding motor to hold down the catapult
 * Returns: true if it's in the process of going into hold position, false if it's finished
 */
/*bool Catapult::Hold(void)
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
}*/

void Catapult::StartRelease(void)
{
	//Make sure the catapult is not already releasing
	if(!releasing)
	{
		releasing = true;
		releaseTimer->Reset();
		releaseTimer->Start();
	}
}

/*
 * bool ReleaseHold():
 * Move the holding motor to allow the catapult to release
 * Returns: true if it's in the process of going into release position, false it's finished
 */
bool Catapult::ReleaseHold(void)
{
	//If the robot is supposed to release, move the motor
	if(releasing)
	{
		//If a time has passed and the limit is pressed, stop moving the release
		if((releaseTimer->Get() >= RELEASE_TIMER) && (holdingLimit->Get() == PRESSED))
		{
			holdingMotor->Set(STOPPED);
			releaseTimer->Stop();
			releasing = false;
		}
		else
		{
			holdingMotor->Set(FULL_FORWARDS);
			return true;
		}
	}
	else
	{
		holdingMotor->Set(STOPPED);
		return false;
	}
	
	//If we get here, something went wrong so stop the motor
	holdingMotor->Set(STOPPED);
	return false;
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
		loadingMotor->Set(FULL_BACKWARDS);
		if((loadedLimit->Get() == PRESSED))
		{
			loadingMotor->Set(STOPPED);
			loadingState = LOAD_RELEASE_TENSION;
			unwindTimer->Reset();
			unwindTimer->Start();
		}
		break;
	//Step 2: Move the loading catapult back to allow the catapult to be released freely
	case LOAD_RELEASE_TENSION:
		loadingMotor->Set(FULL_FORWARDS);
		if(unwindTimer->Get() >= UNWIND_TIME)
		{
			loadingMotor->Set(STOPPED);
			unwindTimer->Stop();
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
		StartRelease();
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
/*double Catapult::GetLoadingDist(void)
{
	return loadingEnco->GetDistance();
}*/

//Returns the pot value of the holding motor
/*double Catapult::GetHoldingDist(void)
{
	return holdingPot->GetAverageVoltage();
}*/

int Catapult::GetHoldingLimit(void)
{
	return holdingLimit->Get();
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
		loadingMotor->Set(speed);
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

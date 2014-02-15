#ifndef CATAPULT_H
#define CATAPULT_H

#include "Talon.h"
#include "DigitalInput.h"
#include "Encoder.h"
#include "Timer.h"
#include "GlobalDefines.h"

//States for loading
#define LOAD_PULL_BACK 1
#define LOAD_RELEASE_TENSION 2

//States for shooting
#define SHOOT_RELEASE 1
#define SHOOT_WAIT   2
#define SHOOT_RELOAD 3

//The amount of time the program needs to wait for the catapult to stop shooting so it can be reloaded
#define CATAPULT_WAIT_TIME 1.5
#define RELEASE_TIMER 0.05
#define UNWIND_TIME 1.4
#define WIND_TIMER 5

#define IDLE_STATE 0

class Catapult
{
private:
	Talon* loadingMotor; //Motor used to bring the catapult into position
	Talon* holdingMotor; //Motor that's on a cam used to hold the catapult in place
	DigitalInput* loadedLimit1; //Limmit switch the signals if the catapult is the lowest in can go
	DigitalInput* loadedLimit2;
	DigitalInput* holdingLimit; //Limit switch that signals if the holding motor has gone far enough
	//Encoder* loadingEnco; //Encoder for the loading motor
	//AnalogChannel* holdingPot; //Potentiometer for the holding motor
	Timer* waitTimer;//Timer to wait for the catapult to finish shooting before pulling it back
	Timer* unwindTimer;//Timer that waits for the winch to unwind
	Timer* releaseTimer;
	Timer* windTimer;//Timer that stops winding the catapult after a certain amount of time as a safety
	int loadingState;
	int shootingState;
	bool releasing;
	
public:
	Catapult(int loadingMotorPort, int holdingMotorPort, int loadedLimit1Port,
			int loadedLimit2Port, int holdingLimitPort);
	~Catapult();
	
	//bool Hold(void); //Move the holding motor to hold down the catapult
	void StartRelease(void);
	bool ReleaseHold(void); //Move the holding motor to allow the catapult to release
	void StartLoad(void);
	int Load(void); //Bring the catapult down into the loaded position
	void StartShoot(void);
	int Shoot(void); //Load the catapult and release it

	//Accessor functions
	//double GetLoadingDist(void);//Returns the encoder value of the loading motor
	//double GetHoldingDist(void);//Returns the encoder value of the holding motor
	int GetHoldingLimit(void);//Returns the value of the limit switch on the holding cam
	int GetLoadedLimit1(void);//Returns whether or not the loaded limit switch is pressed
	int GetLoadedLimit2(void);
	int GetLoadingState(void){return loadingState;}
	int GetShootingState(void){return shootingState;}
	
	//Manual Functions
	void MoveHoldingMotor(float speed);
	void MoveLoadingMotor(float speed);
	void Stop(void);
};

#endif

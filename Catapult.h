#ifndef CATAPULT_H
#define CATAPULT_H

#include "Talon.h"
#include "DigitalInput.h"
#include "Encoder.h"
#include "AnalogChannel.h"
#include "GlobalDefines.h"

//Potentiometer values for the holding motor
#define HOLD_MOTOR_HOLD_POS 1 //TODO Change these to the real values
#define HOLD_MOTOR_RELEASE_POS 2

//Encoder values for the loading motors
#define LOAD_MOTOR_RELEASED 0
#define LOAD_MOTOR_LOADED 1000

//States for loading
#define LOAD_PULL_BACK 1
#define LOAD_HOLD 2
#define LOAD_RELEASE_TENSION 3

//States for shooting
#define SHOOT_RELEASE 1
#define SHOOT_RELOAD 2

#define IDLE_STATE 0

class Catapult
{
private:
	Talon* loadingMotor; //Motor used to bring the catapult into position
	Talon* holdingMotor; //Motor used to hold the catapult in place
	DigitalInput* loadedLimit; //Limmit switch the signals if the catapult is the lowest in can go
	Encoder* loadingEnco; //Encoder for the loading motor
	AnalogChannel* holdingPot; //Potentiometer for the holding motor
	int loadingState;
	int shootingState;
	
public:
	Catapult(int loadingMotorPort, int holdingMotorPort, int loadedLimitPort, int loadingEncoPort1,
			int loadingEncoPort2, int holdingPotPort);
	~Catapult();
	
	bool Hold(void); //Move the holding motor to hold down the catapult
	bool ReleaseHold(void); //Move the holding motor to allow the catapult to release
	void StartLoad(void);
	int Load(void); //Bring the catapult down into the loaded position
	void StartShoot(void);
	int Shoot(void); //Load the catapult and release it

	//Accessor functions
	double GetLoadingDist(void);//Returns the encoder value of the loading motor
	double GetHoldingDist(void);//Returns the encoder value of the holding motor
	int GetLoadedLimit(void);//Returns whether or not the loaded limit switch is pressed
	
	//Manual Functions
	void MoveHoldingMotor(float speed);
	void MoveLoadingMotor(float speed);
	void Stop(void);
};

#endif

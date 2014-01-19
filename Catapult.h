#ifndef CATAPULT_H
#define CATAPULT_H

#include "Talon.h"
#include "DigitalInput.h"
#include "Encoder.h"
#include "GlobalDefines.h"

//Encoder values for the holding motor
#define HOLD_MOTOR_HOLD_POS 1000
#define HOLD_MOTOR_RELEASE_POS 0

class Catapult
{
private:
	Talon* loadingMotor; //Motor used to bring the catapult into position
	Talon* holdingMotor; //Motor used to hold the catapult in place
	DigitalInput* loadedLimit; //Limmit switch the signals if the catapult is the lowest in can go
	Encoder* loadingEnco; //Encoder for the loading motor
	Encoder* holdingEnco; //Encoder for the holding motor
	
public:
	Catapult(int loadingMotorPort, int holdingMotorPort, int loadedLimitPort, int loadingEncoPort1,
			int loadingEncoPort2, int holdingEncoPort1, int holdingEncoPort2);
	~Catapult();
	
	bool Hold(void); //Move the holding motor to hold down the catapult
	bool ReleaseHold(void); //Move the holding motor to allow the catapult to release
	bool Load(void); //Bring the catapult down into the loaded position
	bool Shoot(void); //Load the catapult and release it
	
	double GetLoadingDist(void);//Returns the encoder value of the loading motor
	double GetHoldingDist(void);//Returns the encoder value of the holding motor
};

#endif

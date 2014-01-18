#ifndef INTAKE_H
#define INTAKE_H

#include "Victor.h"
#include "DigitalInput.h"
#include "AnalogChannel.h"
#include "GlobalDefines.h"

#define FULL_FORWARDS 1
#define FULL_BACKWARDS -1
#define STOPPED 0

class Intake
{
private:
	Victor* roller;
	Victor* pivotL;
	Victor* pivotR;
	DigitalInput* upperLimit;
	DigitalInput* lowerLimit;
	AnalogChannel* positionPot;
	
public:
	
	Intake(int rollerPort, int pivotLPort, int pivotRPort, 
			int upperLimitPort, int lowerLimitPort, int positionPotPort);
	~Intake();
	void Move(float speed);
	void MoveToPosition(float pos);
	void RollIn(float speed);
	void Stop();
	float GetPos();
	
	
};

#endif

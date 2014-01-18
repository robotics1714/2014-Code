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
	/*Victor* pivotL;
	Victor* pivotR;
	DigitalInput* upperLimit;
	DigitalInput* lowerLimit;
	AnalogChannel* positionPot;*/
	
public:
	
	Intake(int rollerPort);
	~Intake();
	void RollIn(float speed);
	void Stop(void);
};

#endif

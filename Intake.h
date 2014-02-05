#ifndef INTAKE_H
#define INTAKE_H

#include "Talon.h"
#include "DigitalInput.h"
#include "GlobalDefines.h"

class Intake
{
private:
	Talon* roller;
	DigitalInput* ballSensor;
	/*Victor* pivotL;
	Victor* pivotR;
	DigitalInput* upperLimit;
	DigitalInput* lowerLimit;
	AnalogChannel* positionPot;*/
	
public:
	
	Intake(int rollerPort, int ballSensorPort);
	~Intake();
	void RollIn(void);
	void RollOut(void);
	void GetBallForPass(void);
	int GetBallSensor(void);
	void Stop(void);
};

#endif

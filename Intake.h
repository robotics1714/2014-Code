#ifndef INTAKE_H
#define INTAKE_H

#include "Talon.h"
#include "DigitalInput.h"
#include "Servo.h"
#include "GlobalDefines.h"

#define SERVO_UP 20
#define SERVO_DOWN 110

class Intake
{
private:
	Talon* roller;
	DigitalInput* ballSensor;
	Servo* dropServo;//Servo to knock down the intake in auto
	/*Victor* pivotL;
	Victor* pivotR;
	DigitalInput* upperLimit;
	DigitalInput* lowerLimit;
	AnalogChannel* positionPot;*/
	
public:
	
	Intake(int rollerPort, int ballSensorPort, int servoPort);
	~Intake();
	void RollIn(void);
	void RollOut(void);
	void GetBallForPass(void);
	void DropIntake(void);
	int GetBallSensor(void);
	void Stop(void);
};

#endif

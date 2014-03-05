#ifndef INTAKE_H
#define INTAKE_H

#include "Talon.h"
#include "DigitalInput.h"
#include "Servo.h"
#include "GlobalDefines.h"

#define LEFT_SERVO_UP 20
#define LEFT_SERVO_DOWN 110
#define RIGHT_SERVO_UP 160
#define RIGHT_SERVO_DOWN 80

class Intake
{
private:
	Talon* roller;
	DigitalInput* ballSensor;
	Servo* leftServo;//Servo to knock down the intake in auto
	Servo* rightServo;
	/*Victor* pivotL;
	Victor* pivotR;
	DigitalInput* upperLimit;
	DigitalInput* lowerLimit;
	AnalogChannel* positionPot;*/
	
public:
	
	Intake(int rollerPort, int ballSensorPort, int leftServoPort, int rightServoPort);
	~Intake();
	void RollIn(void);
	void RollOut(void);
	void GetBallForPass(void);
	void DropIntake(void);
	void LiftIntake(void);
	int GetBallSensor(void);
	void Stop(void);
};

#endif

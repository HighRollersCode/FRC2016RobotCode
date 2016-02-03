/*
 * Intake.h
 *
 *  Created on: Jan 30, 2016
 *      Author: 987
 */

#ifndef SRC_INTAKE_H_
#define SRC_INTAKE_H_

#include "WPILib.h"

class IntakeClass
{
public:
	IntakeClass(int Intake_PWM, int IntakeLift_PWM, int LiftEncoder_PWM, int LiftEncoder2_PWM);
	virtual ~IntakeClass();

<<<<<<< HEAD
	void Intake_In();
	void Intake_Out();
	void Intake_Off();
	void Motors(float intake, float intakelift);
=======

>>>>>>> ffde5bb6b30f9cddf011ec59c48223a6af19c89f

	Talon *Intake;
	Talon *IntakeLift;

	Encoder *LiftEncoder;

	void Intake_In();
	void Intake_Out();
	void Intake_Off();
	int GetLiftEncoder();
	void SendData();
};

#endif /* SRC_INTAKE_H_ */

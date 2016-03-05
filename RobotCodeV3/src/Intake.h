/*
 * Intake.h
 *
 *  Created on: Jan 30, 2016
 *      Author: 987
 */

#ifndef SRC_INTAKE_H_
#define SRC_INTAKE_H_

#include "WPILib.h"
#include "ResettableEncoder.h"


class IntakeClass
{
public:
	IntakeClass();
	virtual ~IntakeClass();



	Talon *Intake;
	Talon *IntakeLift;
	ResettableEncoderClass *LiftEncoder;

	int LiftEncoder_Cur = 0;
	int LiftEncoder_Targ = 0;

	float LifterCommand_Cur = 0.0f;
	float LifterCommand_Prev = 0.0f;
	float kpLifter = .001f;
	PIDController *LiftPIDController;
	//void Intake_In();
	//void Intake_Out();
	//void Intake_Off();
	void Update(float intake, float intakelift);
	void GotoFloor();
	void GotoIntake();
	void GotoDefense();
	void IntakeOn();
	void IntakeOff();
	void IntakeOut();
	//void GoToInEndGame();
	void ResetEncoderLift();
	void ResetEncoderLiftDown();

	int GetLiftEncoder();
	void SetLift(int targ);
	void SendData();
};

#endif /* SRC_INTAKE_H_ */

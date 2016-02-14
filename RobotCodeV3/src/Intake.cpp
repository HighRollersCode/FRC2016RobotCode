/*
 * Intake.cpp
 *
 *  Created on: Jan 30, 2016
 *      Author: 987
 */

#include "Intake.h"
#include "Defines.h"


IntakeClass::IntakeClass()
{
	Intake = new Talon(Tal_Intake_Roller);//Intake_PWM
	IntakeLift = new Talon(Tal_Intake_Lift);//IntakeLift_PWM
	LiftEncoder = new Encoder(Encoder_Intake_Lift_1, Encoder_Intake_Lift_2);

	LiftEncoder_Cur = 0;
	LiftEncoder_Targ = 0;
	LifterCommand_Cur = 0.0f;
	LifterCommand_Prev = 0.0f;
	kpLifter = .004f;
}

void IntakeClass::Update(float intake, float intakelift)
{
	LifterCommand_Prev = LifterCommand_Cur;
	LifterCommand_Cur = intakelift;
	float lifterOut = 0;
	float I = intake;
	Intake->Set(I);

	LiftEncoder_Cur = GetLiftEncoder();
	if((fabs(LifterCommand_Prev) > .1f) && (fabs(LifterCommand_Cur) < .1f))
	{
		LiftEncoder_Targ = LiftEncoder_Cur;
	}

	if(fabs(LifterCommand_Cur) != 0)
	{
		LiftEncoder_Targ = -1.0f;
		lifterOut = LifterCommand_Cur;
		SmartDashboard::PutNumber("STATEE2",1);
	}
	else
	{
		lifterOut = -(LiftEncoder_Targ - LiftEncoder_Cur) * kpLifter + .1f;
		SmartDashboard::PutNumber("STATEE",0);
	}
	IntakeLift->Set(-lifterOut);
}

IntakeClass::~IntakeClass() {
	// TODO Auto-generated destructor stub
}
/*void IntakeClass::Intake_In()
{
	Intake->Set(1.0);
}
void IntakeClass::Intake_Out()
{
	Intake->Set(-1.0);
}
void IntakeClass::Intake_Off()
{
	Intake->Set(0);
}*/
void IntakeClass::GotoFloor()
{
	SetLift(0);
}
void IntakeClass::GotoIntake()
{
	//height here
	SetLift(85);
}
void IntakeClass::GotoDefense()
{
	SetLift(495);
}
void IntakeClass::SetLift(int targ)
{
	LiftEncoder_Targ = targ;
}
int IntakeClass::GetLiftEncoder()
{
	return LiftEncoder->Get();
}
void IntakeClass::ResetEncoderLift()
{
	LiftEncoder->Reset();
	SetLift(0);
}
void IntakeClass::SendData()
{
	SmartDashboard::PutNumber("BeaterBarEncoder",LiftEncoder->Get());
}

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
	// TODO Auto-generated constructor stub
	Intake = new Talon(Tal_Intake_Roller);//Intake_PWM
	IntakeLift = new Talon(Tal_Intake_Lift);//IntakeLift_PWM
	LiftEncoder = new Encoder(Encoder_Intake_Lift_1, Encoder_Intake_Lift_2);
}

void IntakeClass::Motors(float intake, float intakelift)
{
	float I = intake;
	float IL = intakelift;

	Intake->Set(I);
	IntakeLift->Set(IL);

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
int IntakeClass::GetLiftEncoder()
{
	return LiftEncoder->Get();
}
void IntakeClass::SendData()
{
	SmartDashboard::PutNumber("LiftEncoder",LiftEncoder->Get());
}

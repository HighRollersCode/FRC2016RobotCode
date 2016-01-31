/*
 * Intake.cpp
 *
 *  Created on: Jan 30, 2016
 *      Author: 987
 */

#include <Intake.h>

IntakeClass::IntakeClass(int Intake_PWM, int IntakeLift_PWM, int LiftEncoder_PWM, int LiftEncoder2_PWM)
{
	// TODO Auto-generated constructor stub
	Intake = new Talon(Intake_PWM);
	IntakeLift = new Talon(IntakeLift_PWM);
	LiftEncoder = new Encoder(LiftEncoder_PWM, LiftEncoder2_PWM);

}

IntakeClass::~IntakeClass() {
	// TODO Auto-generated destructor stub
}
void IntakeClass::Intake_In()
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
}

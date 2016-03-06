/*
 * Intake.cpp
 *
 *  Created on: Jan 30, 2016
 *      Author: 987
 */

#include "Intake.h"
#include "Defines.h"


class IntakeLiftVictorClass : public Victor
{
public:
	explicit IntakeLiftVictorClass(uint32_t channel,IntakeClass * intake);
	virtual void Set(float value, uint8_t syncGroup = 0) override;
	virtual void PIDWrite(float output) override;

protected:
	IntakeClass * m_Intake;
};


IntakeLiftVictorClass::IntakeLiftVictorClass(uint32_t channel,IntakeClass * intake) :
			Victor(channel),
			m_Intake(intake)

{
}
void IntakeLiftVictorClass::Set(float value, uint8_t syncGroup)
{
	value = m_Intake->Validate_Lift_Command(value);
	Victor::Set(value,syncGroup);
}
void IntakeLiftVictorClass::PIDWrite(float value)
{
	value = m_Intake->Validate_Lift_Command(value);
	Victor::Set(value);
}








IntakeClass::IntakeClass()
{
	Intake = new Talon(Tal_Intake_Roller);//Intake_PWM
	IntakeLift = new Talon(Tal_Intake_Lift);//IntakeLift_PWM
	LiftEncoder = new ResettableEncoderClass(Encoder_Intake_Lift_1, Encoder_Intake_Lift_2);

	LimitSwitch = new DigitalInput(Intake_Lift_Limit_Switch);

	LiftEncoder_Cur = 0;
	LiftEncoder_Targ = 0;
	LifterCommand_Cur = 0.0f;
	LifterCommand_Prev = 0.0f;
	kpLifter = .003f;

	//	.00075f
	LiftEncoder->Reset();
	LiftPIDController= new PIDController(.01f,.0005f,0,LiftEncoder,IntakeLift,.05f);
	LiftPIDController->SetContinuous(false);
	LiftPIDController->Disable();
	LiftPIDController->SetAbsoluteTolerance(1);
}

void IntakeClass::Update(float intake, float intakelift)
{
	LifterCommand_Prev = LifterCommand_Cur;
	LifterCommand_Cur = intakelift;
	Intake->Set(intake);

	LiftEncoder_Cur = GetLiftEncoder();
	if((fabs(LifterCommand_Prev) > .1f) && (fabs(LifterCommand_Cur) < .1f))
	{
		printf("Intake lift released, HOLD: %d\r\n",LiftEncoder_Cur);
		SetLift(LiftEncoder_Cur);
		LiftPIDController->Reset();
		LiftPIDController->Enable();
	}

	if(fabs(LifterCommand_Cur) > 0.1f)
	{
		LiftEncoder_Targ = -1.0f;
		LiftPIDController->Disable();
		IntakeLift->Set(-LifterCommand_Cur);
	}
}

IntakeClass::~IntakeClass() {
	// TODO Auto-generated destructor stub
}

void IntakeClass::GotoFloor()
{
	SetLift(Preset_Intake_Floor);
}
void IntakeClass::GotoIntake()
{
	SetLift(Preset_Intake_Intake);
}
void IntakeClass::GotoDefense()
{
	SetLift(Preset_Intake_Defense);
}
void IntakeClass::IntakeOn()
{
	Intake->Set(1);
}
void IntakeClass::IntakeOff()
{
	Intake->Set(0);
}
void IntakeClass::IntakeOut()
{
	Intake->Set(-.5f);
}
void IntakeClass::SetLift(int targ)
{
	LiftEncoder_Targ = targ;
	LiftPIDController->SetSetpoint((float)targ);
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

void IntakeClass::ResetEncoderLiftDown()
{
	LiftEncoder->Reset_To_Value(Preset_Intake_Down);
	SetLift(Preset_Intake_Down);
}

void IntakeClass::SendData()
{
	SmartDashboard::PutNumber("BeaterBarEncoder",LiftEncoder->Get());
	SmartDashboard::PutData("IntakePID", LiftPIDController);
	SmartDashboard::PutBoolean("IntakeLimit", LimitSwitch->Get());
}
float IntakeClass::Validate_Lift_Command(float cmd)
{
#if 0
	int lift = GetLiftEncoder();
	if (limitswitch)   // if cmd is moving turret toward lower angle...
	{
		if (lift < INTAKE_LIFT_MIN)  // and it is past the lowest angle allowed
		{
			//float error = ARM_TURRET_MIN_ENCODER - tur;
			// BAD!
			return 0.0f;
		}
	}
#endif
	return cmd;
}

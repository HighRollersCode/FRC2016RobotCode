/*
 * Arm.cpp
 *
 *  Created on: Jan 30, 2016
 *      Author: HighRollers
 */

#include "Arm.h"
#include <Defines.h>

ArmClass::ArmClass() {
	ArmShooter = new Talon(Tal_ArmShooter_Left);
	ArmShooter2 = new Talon(Tal_ArmShooter_Right);
	ArmLifter = new Talon(Tal_ArmLifter);
	ArmTurret = new Talon(Tal_ArmTurret);

	TurretEncoder = new Encoder(Encoder_Arm_Turret_1, Encoder_Arm_Turret_2, false,Encoder::EncodingType::k4X);
	LifterEncoder = new Encoder(Encoder_Arm_Lift_1, Encoder_Arm_Lift_2, false,Encoder::EncodingType::k4X);

	BallIn = new Solenoid(Sol_Ball_In);
	BallPusher = new Solenoid(Sol_Ball_Pusher);

	CurrentBallTog = false;
	PrevBallTog = false;

	CurrentResetInput = false;
	PrevResetInput = false;


	TurretEncoder_Cur = 0;
	TurretEncoder_Targ = 0;

	TurretCommand_Cur = 0.0f;
	TurretCommand_Prev = 0.0f;
	kpTurret = .01f;

	TurretEncoder->Reset();
	LifterEncoder->Reset();
}
void ArmClass::Update(float Lift, float Arm, float Turret, bool Ball, bool Reset)
{

	float b = Lift;
	float a = Arm;
	TurretCommand_Prev = TurretCommand_Cur;
	TurretCommand_Cur = Turret;
	float turretOut = 0;

	PrevBallTog = CurrentBallTog;
	CurrentBallTog = Ball;

	PrevResetInput = CurrentResetInput;
	CurrentResetInput = Reset;

	TurretEncoder_Cur = GetTurretEncoder();
	if((fabs(TurretCommand_Prev) > .1f) && (fabs(TurretCommand_Cur) < .1f))
	{
		TurretEncoder_Targ = TurretEncoder_Cur;
	}
	if(fabs(TurretCommand_Cur) > .1f)
	{
		TurretEncoder_Targ = -1.0f;
		turretOut = TurretCommand_Cur;
	}
	else
	{
		turretOut = -(TurretEncoder_Targ - TurretEncoder_Cur) * kpTurret;
	}
	ArmTurret->Set(turretOut);
	ArmShooter->Set(a);
	ArmShooter2->Set(-a);
	ArmLifter->Set(b);
	if(CurrentResetInput && !PrevResetInput)
	{
		ResetPostion();
	}
	if(Ball)
	{
		BallPusher->Set(false);
		BallIn->Set(true);
	}
	else
	{
		BallPusher->Set(true);
		BallIn->Set(false);
	}

}
void ArmClass::SetTurret(int targ)
{
	TurretEncoder_Targ = targ;
}
int ArmClass::GetTurretEncoder()
{
	return TurretEncoder->Get();
}
int ArmClass::GetLifterEncoder()
{
	return LifterEncoder->Get();
}

void ArmClass::ResetPostion()
{
	TurretEncoder_Targ = 0;
}

void ArmClass::SendData()
{
	SmartDashboard::PutNumber("TurretEncoder",TurretEncoder->Get());
	SmartDashboard::PutNumber("LifterEncoder",LifterEncoder->Get());
}

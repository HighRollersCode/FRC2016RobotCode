/*
 * Arm.cpp
 *
 *  Created on: Jan 30, 2016
 *      Author: HighRollers
 */

#include "Arm.h"
#include <Defines.h>

ArmClass::ArmClass() {
	ArmShooter = new Talon(4);
	ArmShooter2 = new Talon(5);
	ArmLifter = new Talon(6);
	ArmTurret = new Talon(7);

	LifterEncoder = new Encoder(Encoder_Turret_1, false,Encoder::EncodingType::k4X);
	TurretEncoder = new Encoder(Encoder_Lift_2, false,Encoder::EncodingType::k4X);

	BallIn = new Solenoid(4);
	BallPusher = new Solenoid(5);

	CurrentBallTog = false;
	PrevBallTog = false;

	TurretEncoder->Reset();
	LifterEncoder->Reset();
}
void ArmClass::Motors(float Lift, float Arm)
{
	float b = Lift;
	float a = Arm;

	ArmShooter->Set(a);
	ArmShooter2->Set(a);
	ArmLifter->Set(b);
}
int ArmClass::GetTurretEncoder()
{
	return TurretEncoder->Get();
}
int ArmClass::GetLifterEncoder()
{
	return LifterEncoder->Get();
}
void ArmClass::Arm_Update(bool Ball)
{
	PrevBallTog = CurrentBallTog;
	CurrentBallTog = Ball;

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
void ArmClass::SendData()
{
	SmartDashboard::PutNumber("TurretEncoder",TurretEncoder->Get());
	SmartDashboard::PutNumber("LifterEncoder",LifterEncoder->Get());
}

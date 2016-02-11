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

	Resetting = false;
	CurrentResetInput = false;
	PrevResetInput = false;

	ResetState = 0;

	ArmLifterEncoder_Cur = 0;
	ArmLifterEncoder_Trag = 0;

	ArmLifterCommand_Cur = 0.0f;
	ArmLifterCommand_Prev = 0.0f;
	kpArmLifter = .004f;


	TurretEncoder_Cur = 0;
	TurretEncoder_Targ = 0;

	TurretCommand_Cur = 0.0f;
	TurretCommand_Prev = 0.0f;
	kpTurret = .003f;

	TurretEncoder->Reset();
	LifterEncoder->Reset();
}
void ArmClass::Update(float ArmLift, float Shooter, float Turret, bool Ball, bool Reset,bool EnableTracking,float cX,float cY)
{
	float a = Shooter;
	ArmLifterCommand_Prev = ArmLifterCommand_Cur;
	ArmLifterCommand_Cur = ArmLift;
	float ArmLifterOut = 0;

	TurretCommand_Prev = TurretCommand_Cur;
	TurretCommand_Cur = Turret;
	float turretOut = 0;

	PrevBallTog = CurrentBallTog;
	CurrentBallTog = Ball;

	PrevResetInput = CurrentResetInput;
	CurrentResetInput = Reset;

	ArmLifterEncoder_Cur = GetLifterEncoder();
	if((fabs(ArmLifterCommand_Prev) > .1f) && (fabs(ArmLifterCommand_Cur) < .1f))
	{
		ArmLifterEncoder_Trag = ArmLifterEncoder_Cur;
	}
	if(fabs(ArmLifterCommand_Cur) > .1f)
	{
		ArmLifterEncoder_Trag = -1.0f;
		ArmLifterOut = ArmLifterCommand_Cur;
	}
	else
	{
		ArmLifterOut = -(ArmLifterEncoder_Trag - ArmLifterEncoder_Cur) * kpArmLifter;
	}

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


	if(EnableTracking)
	{
		HandleTarget(cX,cY);
	}
	ArmTurret->Set(turretOut);
	ArmShooter->Set(a);
	ArmShooter2->Set(-a);
	ArmLifter->Set(-ArmLifterOut);
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

	//HandleSoftLimitsBoi(turretOut);

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
	Resetting = true;
	ResetState = 0;
}

void ArmClass::SetArm(int targ)
{
	ArmLifterEncoder_Trag = targ;
}

void ArmClass::ResetArm()
{
	SetArm(0);
}

void ArmClass::ResetTurret()
{
	SetTurret(0);
}



void ArmClass::HandleSoftLimitsBoi(float currentOutput)
{

	if(TurretEncoder_Cur >= 1000)
	{
		if(currentOutput < 0)
		{
			ArmTurret->Set(0);
		}
	}
	if(TurretEncoder_Cur <= -1000)
	{
		if(currentOutput > 0)
		{
			ArmTurret->Set(0);
		}
	}
}
void ArmClass::HandleTarget(float centerX,float centerY)
{
	float moveByX_Pixels = 0;
	float moveByY_Pixels = 0;

	float moveByX_Degrees = 0;
	float moveByY_Degrees = 0;

	float moveByX_Ticks = 0;
	float moveByY_Ticks = 0;

	float xFOV = 57.12f;
	float yFOV = 44.42f;

	float xPixels = 1280;
	float yPixels = 720;

	float degreesPerPixelX = xFOV / xPixels;
	float degreesPerPixelY = yFOV / yPixels;

	float ticksPerDegreeX = 5000/180.0f;
	float ticksPerDegreeY = 2000/30.0f;

	moveByX_Pixels = centerX - (xPixels/2);
	moveByY_Pixels = centerY - (yPixels/2);

	moveByX_Degrees = moveByX_Pixels * degreesPerPixelX;
	moveByY_Degrees = moveByY_Pixels * degreesPerPixelY;

	moveByX_Ticks = moveByX_Degrees * ticksPerDegreeX;
	moveByY_Ticks = moveByY_Degrees * ticksPerDegreeY;

	SetTurret(GetTurretEncoder()+moveByX_Ticks);
	SetArm(GetLifterEncoder()+moveByY_Ticks);
}
void ArmClass::GotoShooting()
{
	SetArm(2200);
}
void ArmClass::SendData()
{
	SmartDashboard::PutNumber("TurretEncoder",TurretEncoder->Get());
	SmartDashboard::PutNumber("LifterEncoder",LifterEncoder->Get());
}

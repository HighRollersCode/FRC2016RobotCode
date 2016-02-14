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
	kpTurret = .0035f;

	TurretEncoder->Reset();
	LifterEncoder->Reset();
}
void ArmClass::UpdateLift(float ArmLift)
{
	ArmLifterCommand_Prev = ArmLifterCommand_Cur;
	ArmLifterCommand_Cur = ArmLift;
	float ArmLifterOut = 0;

	ArmLifterEncoder_Cur = GetLifterEncoder();
	if((fabs(ArmLifterCommand_Prev) > .1f) && (fabs(ArmLifterCommand_Cur) < .1f))
	{
		ArmLifterEncoder_Trag = ArmLifterEncoder_Cur;
	}
	if(fabs(ArmLifterCommand_Cur) > .1f)
	{
		if(GetLifterEncoder() >= 5000)
		{
			if((TurretCommand_Cur) < -.2f)
			{
				TurretCommand_Cur *= .5f;
			}
		}
		if (TurretRoughlyCentered())
		{
			if(GetLifterEncoder() >= 8750)
			{
				ArmLifterOut = Clamp_Target(ArmLifterCommand_Cur, 0, 1);
				SetTurret(8751);
			}
			else if(GetLifterEncoder() <= 0)
			{
				ArmLifterOut = Clamp_Target(ArmLifterCommand_Cur, -1, 0);
				SetTurret(0);
			}
			else
			{
				ArmLifterOut = ArmLifterCommand_Cur;
				ArmLifterEncoder_Trag = -1.0f;
			}
		}
		else
		{
			if(GetLifterEncoder() >= 8750)
			{
				ArmLifterOut = Clamp_Target(ArmLifterCommand_Cur, 0, 1);
				SetTurret(8751);
			}
			else if(GetLifterEncoder() <= 2000)
			{
				ArmLifterOut = Clamp_Target(ArmLifterCommand_Cur, -1, 0);
				SetTurret(-1990);
			}
			else
			{
				ArmLifterOut = ArmLifterCommand_Cur;
				ArmLifterEncoder_Trag = -1.0f;
			}
		}
	}
	else
	{
		if (TurretRoughlyCentered())
		{
			ArmLifterEncoder_Trag = Clamp_Target(ArmLifterEncoder_Trag, 0, 8750);
		}
		else
		{
			ArmLifterEncoder_Trag = Clamp_Target(ArmLifterEncoder_Trag, 2000, 8750);
		}
		ArmLifterOut = (ArmLifterEncoder_Trag - ArmLifterEncoder_Cur) * kpArmLifter+(FSign(ArmLifterEncoder_Trag-ArmLifterEncoder_Trag)*.15f);
	}
	ArmLifter->Set(ArmLifterOut);

}
void ArmClass::UpdateTurret(float Turret)
{
	TurretCommand_Prev = TurretCommand_Cur;
	TurretCommand_Cur = Turret;
	float turretOut = 0;

	TurretEncoder_Cur = GetTurretEncoder();
	if((fabs(TurretCommand_Prev) > .1f) && (fabs(TurretCommand_Cur) < .1f))
	{
		TurretEncoder_Targ = TurretEncoder_Cur;
	}
	if(fabs(TurretCommand_Cur) > .1f)
	{
		if(fabs(TurretEncoder_Cur) >= 800)
		{
			if(TurretEncoder_Cur >= 800)
			{
				if((TurretCommand_Cur) < -.2f)
				{
					TurretCommand_Cur *= .5f;
				}
			}
			else if(TurretEncoder_Cur <= 800)
			{
				if((TurretCommand_Cur) > .2f)
				{
					TurretCommand_Cur *= .5f;
				}
			}
		}
		if(TurretEncoder_Cur >= 1221)
		{
			turretOut = Clamp_Target(TurretCommand_Cur, 0, 1);
			SetTurret(1230);
		}
		else if(TurretEncoder_Cur <= -1221)
		{
			turretOut = Clamp_Target(TurretCommand_Cur, -1, 0);
			SetTurret(-1230);
		}
		else
		{
			turretOut = TurretCommand_Cur;
			TurretEncoder_Targ = -1.0f;
		}
	}
	else
	{
		TurretEncoder_Targ = Clamp_Target(TurretEncoder_Targ, -1221, 1221);
		turretOut = -(((TurretEncoder_Targ - TurretEncoder_Cur) * kpTurret )+(FSign(TurretEncoder_Targ-TurretCommand_Cur)*.15f));
	}

	ArmTurret->Set(turretOut);
}
void ArmClass::Update(float ArmLift, float Shooter, float Turret, bool Ball, bool Reset,bool EnableTracking,float cX,float cY,float calX,float calY)
{

	if(EnableTracking)
	{
		HandleTarget(cX,cY,calX,calY);
	}

	UpdateLift(ArmLift);
	UpdateTurret(Turret);

	float a = Shooter;
	PrevBallTog = CurrentBallTog;
	CurrentBallTog = Ball;

	PrevResetInput = CurrentResetInput;
	CurrentResetInput = Reset;

	ArmShooter->Set(a);
	ArmShooter2->Set(-a);
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
float ArmClass::Clamp_Target(float tar, float lowerlim, float upperlim)
{
	if(tar >= upperlim)
	{
		return upperlim;
	}
	else if(tar <= lowerlim)
	{
		return lowerlim;
	}
	else
	{
		return tar;
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
void ArmClass::HandleTarget(float centerX,float centerY,float calX,float calY)
{
	if(fabs (centerX) >= 1 || fabs(centerY) >= 1)
	{
		return;
	}
	else
	{
		float moveByX_Degrees = 0;
		float moveByY_Degrees = 0;

		float moveByX_Ticks = 0;
		float moveByY_Ticks = 0;

		float xFOV = 76.00f;
		float yFOV = 61.2f;

		float ticksPerDegreeX = 1280/90.0f;
		float ticksPerDegreeY = 5000/90.0f;

		moveByX_Degrees = (centerX - calX) * (xFOV*.5f);
		moveByY_Degrees = (centerY - calY) * (yFOV*.5f);

		moveByX_Ticks = moveByX_Degrees * ticksPerDegreeX;
		moveByY_Ticks = moveByY_Degrees * ticksPerDegreeY;

		SetTurret(GetTurretEncoder()+(moveByX_Ticks* .50f));
		SetArm(GetLifterEncoder()-(moveByY_Ticks* .25f));
	}
	/*
	if(fabs (centerX) >= 640 || fabs(centerY) >= 480)
	{
		return;
	}
	else
	{
		float moveByX_Pixels = 0;
		float moveByY_Pixels = 0;

		float moveByX_Degrees = 0;
		float moveByY_Degrees = 0;

		float moveByX_Ticks = 0;
		float moveByY_Ticks = 0;

		//float xFOV = 57.12f;
		//float yFOV = 44.42f;
		float xFOV = 76.00f;
		float yFOV = 61.2f;

		float xPixels = 320;
		float yPixels = 240;

		float degreesPerPixelX = xFOV / xPixels;
		float degreesPerPixelY = yFOV / yPixels;

		float ticksPerDegreeX = 1280/90.0f;
		float ticksPerDegreeY = 5000/90.0f;

		moveByX_Pixels = centerX - (xPixels/2);
		moveByY_Pixels = centerY - (yPixels/2);

		moveByX_Degrees = moveByX_Pixels * degreesPerPixelX;
		moveByY_Degrees = moveByY_Pixels * degreesPerPixelY;

		moveByX_Ticks = moveByX_Degrees * ticksPerDegreeX;
		moveByY_Ticks = moveByY_Degrees * ticksPerDegreeY;

		SetTurret(GetTurretEncoder()+(moveByX_Ticks* .50f));
		SetArm(GetLifterEncoder()-(moveByY_Ticks* .25f));

	}
	*/
}
void ArmClass::GotoShooting()
{
	SetArm(3332);
}
void ArmClass::ResetEncoderLifter()
{

	LifterEncoder->Reset();
	ResetArm();
}
void ArmClass::ResetEncoderTurret()
{
	TurretEncoder->Reset();
	ResetTurret();
}
void ArmClass::SendData()
{
	SmartDashboard::PutNumber("TurretEncoder",TurretEncoder->Get());
	SmartDashboard::PutNumber("TurretLiftEncoder",LifterEncoder->Get());
}
float ArmClass::FSign(float a)
{
	if(a >= 0)
	{
		return 1.0;
	}
	else
	{
		return -1.0;
	}
}
bool ArmClass::TurretRoughlyCentered()
{
	if(fabs (GetTurretEncoder() <= 100))
	{
		return true;
	}
	else
	{
		return false;
	}
}

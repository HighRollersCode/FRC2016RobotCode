/*
 * Arm.cpp
 *
 *  Created on: Jan 30, 2016
 *      Author: HighRollers
 */

#include "Arm.h"
#include "Defines.h"
#include "ResettableEncoder.h"


//////////////////////////////////////////////////////////////////////////
//
//  Specialized 'Victor' Wrapper classes for the arm which validate
//  any commands they recieve by calling the ArmClass::Validate_Cmd
//  functions.
//
//////////////////////////////////////////////////////////////////////////

// ArmLiftVictorClass ensures that the arm stays within the 15" envelope
// arount the robot.
class ArmLiftVictorClass : public Victor
{
public:
	explicit ArmLiftVictorClass(uint32_t channel,ArmClass * arm);
	virtual void Set(float value, uint8_t syncGroup = 0) override;
	virtual void PIDWrite(float output) override;

protected:
	ArmClass * m_Arm;
};


ArmLiftVictorClass::ArmLiftVictorClass(uint32_t channel,ArmClass * arm) :
			Victor(channel),
			m_Arm(arm)

{
}
void ArmLiftVictorClass::Set(float value, uint8_t syncGroup)
{
	value = m_Arm->Validate_Lift_Command(value);
	Victor::Set(value,syncGroup);
}
void ArmLiftVictorClass::PIDWrite(float value)
{
	value = m_Arm->Validate_Lift_Command(value);
	Victor::Set(value);
}


// ArmTurretVictorClass ensures that the turret does not move past
// its mechanical limits.
class ArmTurretVictorClass : public Victor
{
public:
	explicit ArmTurretVictorClass(uint32_t channel,ArmClass * arm);
	virtual void Set(float value, uint8_t syncGroup = 0) override;
	virtual void PIDWrite(float output) override;

protected:
	ArmClass * m_Arm;
};

ArmTurretVictorClass::ArmTurretVictorClass(uint32_t channel,ArmClass * arm) :
	Victor(channel),
	m_Arm(arm)
{
}

void ArmTurretVictorClass::Set(float value, uint8_t syncGroup)
{
	value = m_Arm->Validate_Turret_Command(value);
	Victor::Set(value,syncGroup);
}

void ArmTurretVictorClass::PIDWrite(float value)
{
	value = m_Arm->Validate_Turret_Command(value, true);
	Victor::Set(value);
}








ArmClass::ArmClass()
{
	ArmShooter = new Victor(Tal_ArmShooter_Left);
	ArmShooter2 = new Victor(Tal_ArmShooter_Right);
	ArmLifter = new ArmLiftVictorClass(Tal_ArmLifter, this);
	ArmTurret = new ArmTurretVictorClass(Tal_ArmTurret, this);

	TurretEncoder = new Encoder(Encoder_Arm_Turret_1, Encoder_Arm_Turret_2, false,Encoder::EncodingType::k4X);
	LifterEncoder = new ResettableEncoderClass(Encoder_Arm_Lift_1, Encoder_Arm_Lift_2, false,Encoder::EncodingType::k4X);

	ShotExtend = new Solenoid(Sol_Shot_Extend);
	ShotRetract = new Solenoid(Sol_Shot_Retract);

	CurrentBallTog = false;
	PrevBallTog = false;

	Resetting = false;
	CurrentResetInput = false;
	PrevResetInput = false;

	isShooting = false;
	ShotStage = 0;
	ShotTimer = new Timer();

	ResetState = 0;

	ArmLifterEncoder_Cur = 0;
	ArmLifterEncoder_Trag = 0;

	ArmLifterCommand_Cur = 0.0f;
	ArmLifterCommand_Prev = 0.0f;
	kpArmLifter = .001f;

	isTracking = false;
	CurrentEnableTracking = false;
	PreviousEnableTracking = false;

	LastMoveByDegreesX = 360.0f;
	LastMoveByDegreesY = 360.0f;

	TurretEncoder_Cur = 0;
	TurretEncoder_Targ = 0;

	TurretCommand_Cur = 0.0f;
	TurretCommand_Prev = 0.0f;
	kpTurret = .00095f;

	ArmTimer = new Timer();
	ArmTimer->Reset();
	ArmTimer->Start();
isauto = false;
	TurretEncoder->Reset();
	LifterEncoder->Reset();
	TargetingLights= new Relay(0,Relay::kBothDirections);
	//ArmPIDController= new PIDController(.00085f,.00000945f,0.00000945f,LifterEncoder,ArmLifter,.01f);
//	ArmPIDController= new PIDController(.0013f,.00001f,0,LifterEncoder,ArmLifter,.01f);
	ArmPIDController= new PIDController(-.0013f,-.00001f,.0002,LifterEncoder,ArmLifter,.01f);
	ArmPIDController->SetContinuous(false);
	ArmPIDController->Enable();
	ArmPIDController->SetAbsoluteTolerance(10);
	ArmPIDController->SetOutputRange(-.5f,.5f);
	//ArmPIDController = Clamp_Target(0.5, 0.5, 0.5);
	ArmPIDController->SetInputRange(Preset_Arm_Floor ,Preset_Arm_Floor + 10000);

	//ArmPIDController->SetAbsoluteTolerance(100);

	//TurretPIDController = new PIDController(-.000459f, -.000035f ,.00000f,TurretEncoder,ArmTurret,.01f);
//	TurretPIDController = new PIDController(-.00085f, -.000015f ,0,TurretEncoder,ArmTurret,.01f);
	TurretPIDController = new PIDController(-.0025f, -.0000065f ,0,TurretEncoder,ArmTurret,.01f);
	//135
	//245
	TurretPIDController->SetContinuous(false);
	TurretPIDController->Disable();
	//TurretPIDController->SetAbsoluteTolerance(1);
	TurretPIDController->SetInputRange(-1350,1350);
	TurretPIDController->SetOutputRange(-.75f,.75f);
	TunerPIDController = new PIDController(0,0,0,TurretEncoder,ArmTurret,.01f);
	TunerPIDController->SetContinuous(false);
	TunerPIDController->Disable();
	TunerPIDController->SetAbsoluteTolerance(1);
	TunerPIDController->SetInputRange(-1221,1221);

}
void ArmClass::UpdateLift(float ArmLift)
{

	ArmLifterCommand_Prev = ArmLifterCommand_Cur;
	ArmLifterCommand_Cur = ArmLift;
	float ArmLifterOut = 0;

	ArmLifterEncoder_Cur = GetLifterEncoder();
	//if((fabs(ArmLifterCommand_Prev) > .1f) && (fabs(ArmLifterCommand_Cur) < .1f))
	//{
		//SetArm(ArmLifterEncoder_Cur);
		//ArmPIDController->Enable();
	//}

	if(fabs(ArmPIDController->GetError()) < 1000)
	{
		ArmPIDController->SetPID(ArmPIDController->GetP(),-.00001f,ArmPIDController->GetD());
	}
	else
	{
		ArmPIDController->SetPID(ArmPIDController->GetP(),0,ArmPIDController->GetD());
	}


	if(fabs(ArmLifterCommand_Cur) > .1f)
	{
		ArmPIDController->Disable();
		if(GetLifterEncoder() >= 5000)
		{
			if((ArmLifterCommand_Cur) < -.2f)
			{
				ArmLifterCommand_Cur *= .5f;
			}
		}
		/*
		if (TurretRoughlyCentered())
		{
			if(GetLifterEncoder() >= 8750)
			{
				ArmLifterOut = Clamp_Target(ArmLifterCommand_Cur, 0, 1);
				SetArm(8751);
			}
			else if(GetLifterEncoder() <= 0)
			{
				ArmLifterOut = Clamp_Target(ArmLifterCommand_Cur, -1, 0);
				SetArm(0);
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
				SetArm(8751);
			}
			else if(GetLifterEncoder() <= 2000)
			{
				ArmLifterOut = Clamp_Target(ArmLifterCommand_Cur, -1, 0);
				SetArm(-1990);
			}
			else
			{
				ArmLifterOut = ArmLifterCommand_Cur;
				ArmLifterEncoder_Trag = -1.0f;
			}
		}
		*/
		ArmLifterOut = ArmLifterCommand_Cur;
		//ArmLifterEncoder_Trag = -1.0f;
		ArmLifter->Set(-ArmLifterOut);
	}
	else
	{
		if(!ArmPIDController->IsEnabled())
		{
			ArmLifterOut = 0;
			ArmLifter->Set(ArmLifterOut);
		}
		//if (TurretRoughlyCentered())
		//{
		//	ArmLifterEncoder_Trag = Clamp_Target(ArmLifterEncoder_Trag, 0, 8750);

		//}
		//else
		//{
		//	ArmLifterEncoder_Trag = Clamp_Target(ArmLifterEncoder_Trag, 2000, 8750);
		//}
	//	ArmLifterOut =- ((ArmLifterEncoder_Trag - ArmLifterEncoder_Cur) * kpArmLifter+(FSign(ArmLifterEncoder_Trag-ArmLifterEncoder_Cur)*.10f));
	}
		//ArmLifter->Set(-ArmLifterOut);
	//SmartDashboard::PutBoolean("ArmPIDController", ArmPIDController->IsEnabled());
		SmartDashboard::PutData("PID", ArmPIDController);
		SmartDashboard::PutData("TUNERYaAXIS", TunerPIDController);

}
void ArmClass::UpdateTurret(float Turret)
{
	TurretCommand_Prev = TurretCommand_Cur;
	TurretCommand_Cur = Turret;
	float turretOut = 0;

	TurretEncoder_Cur = GetTurretEncoder();

	if(fabs(TurretPIDController->GetError()) < 200)
	{
		TurretPIDController->SetPID(TurretPIDController->GetP(),0,0);//TurretPIDController->GetD());
	}
	else
	{
		TurretPIDController->SetPID(TurretPIDController->GetP(),0,0);//TurretPIDController->GetD());
	}
	if((fabs(TurretCommand_Prev) > .1f) && (fabs(TurretCommand_Cur) < .1f))
	{
		//SetTurret(TurretEncoder_Cur);
		//TurretPIDController->Disable();
	}
	if(fabs(TurretCommand_Cur) > .2f)
	{
		TurretPIDController->Disable();
		/*if(fabs(TurretEncoder_Cur) >= 800)
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
		}*/
		/*
		if(TurretEncoder_Cur >= 1221)
		{
			//turretOut = Clamp_Target(TurretCommand_Cur, 0, 1);
			SetTurret(1230);
		}
		else if(TurretEncoder_Cur <= -1221)
		{
		//	turretOut = Clamp_Target(TurretCommand_Cur, -1, 0);
			SetTurret(-1230);
		}
		else
		*/
		{
			turretOut = TurretCommand_Cur;

		}
		//turretOut = TurretCommand_Cur;
		//TurretEncoder_Targ = -1.0f;
		ArmTurret->Set(turretOut);
	}
	else
	{
		if(!TurretPIDController-> IsEnabled())
		{
			ArmTurret->Set(0);
		}
		//turretOut = -(((TurretEncoder_Targ - TurretEncoder_Cur) * kpTurret )+(FSign(TurretEncoder_Targ-TurretEncoder_Cur)*.15f));
	}

	//ArmTurret->Set(turretOut);
	//SmartDashboard::PutBoolean("TurretPIDController", TurretPIDController->IsEnabled());
	SmartDashboard::PutData("TurretPID", TurretPIDController);
}
void ArmClass::Update(float ArmLift, float Shooter, float Turret, bool Ball, bool Reset, bool EnableTracking,float cX,float cY,float calX,float calY)
{
	PreviousEnableTracking = CurrentEnableTracking;
	CurrentEnableTracking = EnableTracking;
	if(CurrentEnableTracking && !PreviousEnableTracking)
	{
		ArmPIDController->Reset();
		TurretPIDController->Reset();
	}
	if(CurrentEnableTracking)
	{
		ArmPIDController->Enable();
		TurretPIDController->Enable();
		SetArm(GetLifterEncoder());
		SetTurret(GetTurretEncoder());
	}
	if(!CurrentEnableTracking && PreviousEnableTracking)
	{
		ArmPIDController->Disable();
		TurretPIDController->Disable();
	}

	//TurretPIDController->SetPID(P*.001f, I*.00001f, 0.0f);

	if(EnableTracking && ArmTimer->Get() > .01f)
	{
		HandleTarget(cX,cY,calX,calY);
		ArmTimer->Reset();
		ArmTimer->Start();
	}

	UpdateLift(ArmLift);
	UpdateTurret(Turret);
	if(CurrentResetInput && !PrevResetInput)
	{
		ResetPostion();
	}
	if(!isauto )
	{
		float a = Shooter;
		PrevBallTog = CurrentBallTog;
		CurrentBallTog = Ball;

		PrevResetInput = CurrentResetInput;
		CurrentResetInput = Reset;

		ArmShooter->Set(a);
		ArmShooter2->Set(-a);

		if(Ball)
		{
			ShotRetract->Set(false);
			ShotExtend->Set(true);
		}
		else
		{
			ShotRetract->Set(true);
			ShotExtend->Set(false);
		}
	}
	//SmartDashboard::PutData("ArmPID",ArmPIDController);
	//SmartDashboard::PutData("TurretPID",TurretPIDController);
	//LiveWindow::GetInstance()->AddActuator("ArmPID","ArmPIDController",ArmPIDController);
}
void ArmClass::StartTracking(int enable)
{
	isTracking = (enable != 0);
}

void ArmClass::AutonomousTrackingUpdate(float tx, float ty, float crossX, float crossY)
{
	if(fabs(ArmPIDController->GetError()) < 1000)
	{
		ArmPIDController->SetPID(ArmPIDController->GetP(),-.00001f,ArmPIDController->GetD());
	}
	else
	{
		ArmPIDController->SetPID(ArmPIDController->GetP(),0,ArmPIDController->GetD());
	}

	if(fabs(TurretPIDController->GetError()) < 200)
	{
		TurretPIDController->SetPID(TurretPIDController->GetP(),TunerPIDController->GetI(),TurretPIDController->GetD());
	}
	else
	{
		TurretPIDController->SetPID(TurretPIDController->GetP(),0,TurretPIDController->GetD());
	}

	if(isTracking)
	{
		if(ArmTimer->Get() > .01f)
		{
			HandleTarget(tx,ty,crossX,crossY);
			ArmTimer->Reset();
			ArmTimer->Start();
		}
	}
}

void ArmClass::FullShot()
{
	isShooting = true;
	ShotStage = 0;
	ShotTimer->Reset();
	ShotTimer->Start();
}
void ArmClass::FullShotUpdate()
{
	if(isShooting)
	{
		switch(ShotStage)
		{
		case 0:
			ArmShooter->Set(1);
			ArmShooter2->Set(-1);
			ShotTimer->Reset();
			ShotTimer->Start();
			ShotStage = 1;
			break;
		case 1:
			if(ShotTimer->Get() > 1)
			{
				ShotExtend->Set(true);
				ShotRetract->Set(false);
				ShotTimer->Reset();
				ShotTimer->Start();
				ShotStage = 2;
			}
			break;
		case 2:
			if(ShotTimer->Get() > .25f)
			{
				ShotExtend->Set(false);
				ShotRetract->Set(true);
				ArmShooter->Set(0);
				ArmShooter2->Set(0);
				isShooting = false;
			}
			break;
		}
	}
}
void ArmClass::ShooterIntake()
{
	ArmShooter->Set(-.5f);
	ArmShooter2->Set(.5f);
}
void ArmClass::ShooterOff()
{
	ArmShooter->Set(0);
	ArmShooter2->Set(0);
}
void ArmClass::ShooterOutake()
{
	ArmShooter->Set(.5f);
	ArmShooter2->Set(-.5f);
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
void ArmClass::SetTurret(int targ)//,bool relative = false)
{
	TurretEncoder_Targ = targ;
//	TurretEncoder_Targ = Clamp_Target(TurretEncoder_Targ, -1221, 1221);
	//if(!relative)
	//{
	TurretPIDController->SetSetpoint(TurretEncoder_Targ);
	//}
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
	//ArmLifterEncoder_Trag = targ;
	ArmPIDController->SetSetpoint(targ);

}

void ArmClass::ResetArm()
{
	SetArm(Preset_Arm_Floor);
}

void ArmClass::ResetTurret()
{
//	TurretPIDController->SetPID(-.000955f,TurretPIDController->GetI(),TurretPIDController->GetD());
	SetTurret(0);

}
void ArmClass::HandleTarget(float centerX,float centerY,float calX,float calY)
{
	if(fabs (centerX) >= 1 || fabs(centerY) >= 1)
	{
		LastMoveByDegreesX = LastMoveByDegreesY = 360.0f;
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
		//centerX = Clamp_Target(centerX,-.8f,.8f);

		moveByX_Degrees = (calX - centerX) * (xFOV*.5f);
		moveByY_Degrees = (centerY - calY) * (yFOV*.5f);

		LastMoveByDegreesX = moveByX_Degrees;
		LastMoveByDegreesY = moveByY_Degrees;

		moveByX_Ticks = moveByX_Degrees * ticksPerDegreeX;
		moveByY_Ticks = moveByY_Degrees * ticksPerDegreeY;

		if(GetLifterEncoder() >= 0)
		{
			moveByX_Ticks *=-1.0f;
			moveByY_Ticks *=-1.0f;
		}

		SetTurret(GetTurretEncoder()-(moveByX_Ticks* 1.0f)); //.85
		SetArm(GetLifterEncoder()+(moveByY_Ticks* 1.0f));
		SmartDashboard::PutNumber("MoveByTicks",moveByX_Ticks);
		SmartDashboard::PutNumber("TurretError",TurretPIDController->GetError());
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
void ArmClass::GotoTowerShot()
{
	SetArm(Preset_Arm_Tower_Shot);
}
void ArmClass::GotoShooting()
{
	SetArm(Preset_Arm_Far_Shot);
}
void ArmClass::GoToArm()
{
	SetArm(Preset_Arm_Intake);
}
void ArmClass::ResetEncoderLifter()
{
	LifterEncoder->Reset();
	ArmPIDController->Reset();
}
void ArmClass::ResetEncoderTurret()
{
	TurretEncoder->Reset();
	TurretPIDController->Reset();
}

void ArmClass::ResetEncoderLifterDown()
{
	LifterEncoder->Reset_To_Value(Preset_Arm_Down);
	ArmPIDController->Reset();
}


void ArmClass::SendData()
{
	SmartDashboard::PutNumber("ArmTurretEncoder",TurretEncoder->Get());
	SmartDashboard::PutNumber("ArmLiftEncoder",LifterEncoder->Get());
	SmartDashboard::PutNumber("TURRCOM",ArmTurret->Get());
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
	if(abs(GetTurretEncoder()) <= 100)
	{
		return true;
	}
	else
	{
		return false;
	}
}

float ArmClass::Turret_Encoder_To_Degrees(int enc)
{
	return enc * ARM_TURRET_DEGREES_PER_TICK;
}

float ArmClass::Validate_Turret_Command(float cmd, bool ispidcmd)
{
#if 01
	int tur = GetTurretEncoder();
	if (cmd >= 0.0f)   // if cmd is moving turret toward lower angle...
	{
		if (tur < ARM_TURRET_MIN_ENCODER)  // and it is past the lowest angle allowed
		{
			//float error = ARM_TURRET_MIN_ENCODER - tur;
			// BAD!
			return 0.0f;
		}
	}
	else if (cmd <= 0.0f)  // if cmd is moving turret toward higher angles...
	{
		if (tur > ARM_TURRET_MAX_ENCODER)  // and it is past the highest angle allowed
		{
			// BAD!
			return 0.0f;
		}
	}
#endif

	if(ispidcmd)
	{
		//randon .002 commands...
		if(cmd > 0.003f)
		{
			cmd = fmaxf(cmd, MIN_TURRET_CMD);
		}
		else if (cmd < -.003f)
		{
			cmd = fminf(cmd, -MIN_TURRET_CMD);
		}
		else if (cmd < .003f && cmd > -.003)
		{
			cmd = 0;
		}

	}
	return cmd;
}

float ArmClass::Lift_Encoder_To_Degrees(int enc)
{
	return 90.0f + ARM_LIFT_DEGREES_PER_TICK * enc;
}

float ArmClass::Compute_Lift_Error_Correction_Command(float error)
{
	float cmd = -ARM_LIFT_CORRECTION_P * error;
	if (cmd > 0.3f) return 0.3f;
	if (cmd < -0.3f) return -0.3f;

	return cmd;
}

float ArmClass::Validate_Lift_Command(float cmd, bool ispidcmd)
{
#if 01
	// Validating the lift command.
	// We broke the limits down into two cases.
	// 1)  When the turret is facing forward (0deg) the arm can go down to 0deg or up to (180-39.2 = 140.8deg)
	// 2)  When the turret is facing sideways (90deg or 270deg) the arm can go down to 14.6deg or up to (180-14.6 = 165.4deg)

	int lift_enc = GetLifterEncoder();
	float lift_angle = Lift_Encoder_To_Degrees(lift_enc);
	int turret_enc = GetTurretEncoder();
	float turret_angle = Turret_Encoder_To_Degrees(turret_enc);

	// Special case: Turret is centered.
	if (TurretRoughlyCentered())
	{
		if (cmd >= 0.0f)	// if we are pushing the arm down (might have to switch this to > 0.0
		{
			if (lift_angle < ARM_LIFT_MIN_WHEN_CENTERED)
			{
				// BAD!  pushing the arm into the frame of the robot
				float error = ARM_LIFT_MIN_WHEN_CENTERED - lift_angle;
				return Compute_Lift_Error_Correction_Command(error);
			}
		}
		if (cmd <= 0.0f) // lifting, limit at 140 deg
		{
			if (lift_angle > ARM_LIFT_MAX_WHEN_CENTERED)
			{
				float error = ARM_LIFT_MAX_WHEN_CENTERED - lift_angle;
				return Compute_Lift_Error_Correction_Command(error);
			}
		}
	}
	else if (fabs(turret_angle) < 45.0f)   // Mostly Forward case
	{
		if (cmd >= 0.0f)	// if we are pushing the arm down (might have to switch this to > 0.0
		{
			if (lift_angle < ARM_LIFT_MIN_WHEN_FWD)
			{
				// BAD!  pushing the arm into the frame of the robot
				float error = ARM_LIFT_MIN_WHEN_FWD - lift_angle;
				return Compute_Lift_Error_Correction_Command(error);
			}
		}
		if (cmd <= 0.0f) // lifting, limit at 140 deg
		{
			if (lift_angle > ARM_LIFT_MAX_WHEN_FWD)
			{
				float error = ARM_LIFT_MAX_WHEN_FWD - lift_angle;
				return Compute_Lift_Error_Correction_Command(error);
			}
		}
	}
	else  // Sideways case
	{
		if (cmd >= 0.0f)	// if we are pushing the arm down (might have to switch this to > 0.0
		{
			if (lift_angle < ARM_LIFT_MIN_WHEN_SIDEWAYS)
			{
				// BAD!  violating the envelop to one side of the robot
				float error = ARM_LIFT_MIN_WHEN_SIDEWAYS - lift_angle;
				return Compute_Lift_Error_Correction_Command(error);
			}
		}
		if (cmd <= 0.0f) // lifting, limit at 140 deg
		{
			if (lift_angle > ARM_LIFT_MAX_WHEN_SIDEWAYS)
			{
				// BAD! violating the envelope to the other side of the robot
				float error = ARM_LIFT_MAX_WHEN_SIDEWAYS - lift_angle;
				return Compute_Lift_Error_Correction_Command(error);
			}
		}
	}
#endif

	return cmd;
}




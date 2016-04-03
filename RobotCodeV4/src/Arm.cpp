/*
 * Arm.cpp
 *
 *  Created on: Jan 30, 2016
 *      Author: HighRollers
 */

#include "Arm.h"
#include "Defines.h"
#include "ResettableEncoder.h"

//const float MIN_ARM_LIFT_CMD = 0.1f;
const float MIN_ARM_LIFT_CMD = 0.35f;
//const float ARM_LIFT_P = -.0005f;
//const float ARM_LIFT_I = -0.00001f;
const float ARM_LIFT_P = -.0005f;
const float ARM_LIFT_I = -0.000005f;//1f;
const float ARM_LIFT_D = 0;//.0001f;

//const float MIN_TURRET_CMD = 0.12f;
const float MIN_TURRET_CMD_HIGH_ANGLE = 0.10f;
const float MIN_TURRET_CMD_LOW_ANGLE = 0.09f;
//const float ARM_TURRET_P = -.001125f;
//const float ARM_TURRET_I = -.000025f;
const float ARM_TURRET_P = -.0005f;
const float ARM_TURRET_I = -.000001f;
const float ARM_TURRET_D = 0.0f;

const float ARM_TURRET_TOLERANCE = 1;

const float LOCKON_DEGREES_X = 1.75f;
const float LOCKON_DEGREES_Y = 2.0f;
const float LOCKON_SECONDS = 0.35f;

//////////////////////////////////////////////////////////////////////////
//
//  Specialized 'Victor' Wrapper classes for the arm which validate
//  any commands they receive by calling the ArmClass::Validate_Cmd
//  functions.
//
//////////////////////////////////////////////////////////////////////////

// ArmLiftVictorClass ensures that the arm stays within the 15" envelope
// around the robot.
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

	TurretEncoder = new Encoder(Encoder_Arm_Turret_1, Encoder_Arm_Turret_2, false,Encoder::EncodingType::k1X);
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
	isTracking = false;

	BypassEncoderLimits = false;

	ResetState = 0;

	ArmLifterEncoder_Cur = 0;
	ArmLifterEncoder_Trag = 0;

	ArmLifterCommand_Cur = 0.0f;
	ArmLifterCommand_Prev = 0.0f;
	kpArmLifter = .001f;

	CurrentEnableTracking = false;
	PreviousEnableTracking = false;

	CurEmergencyToggle = false;
	PrevEmergencyToggle = false;

	LastMoveByDegreesX = 360.0f;
	LastMoveByDegreesY = 360.0f;

	TurretEncoder_Cur = 0;
	TurretEncoder_Targ = 0;

	TurretCommand_Cur = 0.0f;
	TurretCommand_Prev = 0.0f;
	kpTurret = .00095f;

	Min = 0;
	Angle = 0;

	ArmTimer = new Timer();
	ArmTimer->Reset();
	ArmTimer->Start();

	ArmLockonTimer = new Timer();
	ArmLockonTimer->Reset();
	ArmLockonTimer->Start();
	LastShotTimer = new Timer();
	LastShotTimer->Reset();
	LastShotTimer->Start();

	isauto = false;
	TurretEncoder->Reset();
	LifterEncoder->Reset();
	TargetingLights= new Relay(0,Relay::kBothDirections);
	//ArmPIDController= new PIDController(.00085f,.00000945f,0.00000945f,LifterEncoder,ArmLifter,.01f);
//	ArmPIDController= new PIDController(.0013f,.00001f,0,LifterEncoder,ArmLifter,.01f);
	// Last used:ArmPIDController= new PIDController(-.0013f,-.00001f,.0002f,LifterEncoder,ArmLifter,.01f);
	ArmPIDController= new PIDController(ARM_LIFT_P,ARM_LIFT_I,ARM_LIFT_D,LifterEncoder,ArmLifter,.01f);

	ArmPIDController->SetContinuous(false);
	ArmPIDController->Enable();
	ArmPIDController->SetAbsoluteTolerance(20);
	ArmPIDController->SetOutputRange(-1.0f,1.0f);
	//ArmPIDController = Clamp_Target(0.5, 0.5, 0.5);
	ArmPIDController->SetInputRange(Preset_Arm_Floor ,Preset_Arm_Floor + 10000);

	//ArmPIDController->SetAbsoluteTolerance(100);

	//TurretPIDController = new PIDController(-.000459f, -.000035f ,.00000f,TurretEncoder,ArmTurret,.01f);
//	TurretPIDController = new PIDController(-.00085f, -.000015f ,0,TurretEncoder,ArmTurret,.01f);
//	TurretPIDController = new PIDController(-.0025f, -.0000065f ,0,TurretEncoder,ArmTurret,.01f);
	TurretPIDController = new PIDController(ARM_TURRET_P, ARM_TURRET_I ,ARM_TURRET_D,TurretEncoder,ArmTurret,.01f);
	//135
	//245
	TurretPIDController->SetContinuous(false);
	TurretPIDController->Disable();

	//TurretPIDController->SetAbsoluteTolerance(1);
	TurretPIDController->SetInputRange(ARM_TURRET_MIN_ENCODER,ARM_TURRET_MAX_ENCODER);
	TurretPIDController->SetOutputRange(-.75f,.75f);
	TunerPIDController = new PIDController(ARM_TURRET_P * 1000,ARM_TURRET_I * 1000,ARM_TURRET_D * 1000,TurretEncoder,ArmTurret,.01f);
	TunerPIDController->SetContinuous(false);
	TunerPIDController->Disable();
	TunerPIDController->SetAbsoluteTolerance(ARM_TURRET_TOLERANCE);
	TunerPIDController->SetInputRange(ARM_TURRET_MIN_ENCODER,ARM_TURRET_MAX_ENCODER);

}

void ArmClass::Auto_Start()
{
	CurrentEnableTracking = false;
	PreviousEnableTracking = false;

	LastMoveByDegreesX = 360.0f;
	LastMoveByDegreesY = 360.0f;

	ResetEncoderLifter();
	ResetEncoderTurret();

	ArmPIDController->Disable();
	ArmPIDController->Reset();
	TurretPIDController->Disable();
	TurretPIDController->Reset();

	SetTurret(GetTurretEncoder());
	SetArm(GetLifterEncoder());

	isShooting = false;
	isTracking = false;

	ShotStage = 0;
	ShotTimer->Reset();

	ArmLockonTimer->Reset();
	ArmLockonTimer->Start();

	LastShotTimer->Reset();
	LastShotTimer->Start();
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
	float tempp = ARM_LIFT_P;
	float tempi = ARM_LIFT_I;
	float tempd = ARM_LIFT_D;
	if(fabs(ArmPIDController->GetError()) > 1000)
	{
		tempi = 0;
	}
	if(GetLifterEncoder() > 0)
	{
		const float backscale = 0.75f;

		tempp *= backscale;
		tempi *= backscale;
		tempd *= backscale;
	}
	ArmPIDController->SetPID(tempp,tempi,tempd);

	if(fabs(ArmLifterCommand_Cur) > .1f)
	{
		if(ArmPIDController->IsEnabled())
		{
			ArmPIDController->Disable();
		}
		if(GetLifterEncoder() >= 5000)
		{
			if((ArmLifterCommand_Cur) < -.2f)
			{
				ArmLifterCommand_Cur *= .5f;
			}
		}
		ArmLifterOut = ArmLifterCommand_Cur;
		ArmLifter->Set(-ArmLifterOut);
	}
	else
	{
		if(!ArmPIDController->IsEnabled())
		{
			ArmLifterOut = 0;
			ArmLifter->Set(ArmLifterOut);
		}
	}
}
void ArmClass::UpdateTurret(float Turret)
{
	TurretCommand_Prev = TurretCommand_Cur;
	TurretCommand_Cur = Turret;
	float turretOut = 0;

	TurretEncoder_Cur = GetTurretEncoder();

	if(fabs(TurretPIDController->GetError()) < 300.0f)
	{
		//TurretPIDController->SetPID(ARM_TURRET_P,ARM_TURRET_I,ARM_TURRET_D);//TurretPIDController->GetD());
		TurretPIDController->SetPID(TunerPIDController->GetP() / 1000.0f,TunerPIDController->GetI() / 1000.0f,TunerPIDController->GetD() / 1000.0f);
	}
	else
	{
		TurretPIDController->SetPID(TunerPIDController->GetP() / 1000.0f,0.0f,TunerPIDController->GetD() / 1000.0f);
		//TurretPIDController->SetPID(ARM_TURRET_P,0.0f,ARM_TURRET_D);//TurretPIDController->GetD());
	}
	if((fabs(TurretCommand_Prev) > .1f) && (fabs(TurretCommand_Cur) < .1f))
	{
		//SetTurret(TurretEncoder_Cur);
		//TurretPIDController->Disable();
	}
	if(fabs(TurretCommand_Cur) > .2f)
	{
		if(TurretPIDController->IsEnabled())
		{
			TurretPIDController->Disable();
		}
		turretOut = TurretCommand_Cur;
		ArmTurret->Set(turretOut);
	}
	else
	{
		if(!TurretPIDController-> IsEnabled())
		{
			ArmTurret->Set(0);
		}
	}
}
void ArmClass::Tele_Start()
{
	CurrentEnableTracking = false;
	PreviousEnableTracking = false;

	LastMoveByDegreesX = 360.0f;
	LastMoveByDegreesY = 360.0f;

	ArmPIDController->Disable();
	ArmPIDController->Reset();
	TurretPIDController->Disable();

	TurretPIDController->Reset();
	SetTurret(GetTurretEncoder());
	SetArm(GetLifterEncoder());

	isShooting = false;
	isTracking = false;

	ShotStage = 0;
	ShotTimer->Reset();

	ArmLockonTimer->Reset();
	ArmLockonTimer->Start();

	LastShotTimer->Reset();
	LastShotTimer->Start();
}
void ArmClass::Update(float ArmLift, float Shooter, float Turret, bool Ball, bool Reset, bool EnableTracking,float cX,float cY,float calX,float calY)
{
	PreviousEnableTracking = CurrentEnableTracking;
	CurrentEnableTracking = EnableTracking;
	if(CurrentEnableTracking && !PreviousEnableTracking)
	{
		ArmPIDController->Reset();
		TurretPIDController->Reset();
		ArmLockonTimer->Reset();
		ArmLockonTimer->Start();
	}
	if(!CurrentEnableTracking && PreviousEnableTracking)
	{
		ArmPIDController->Disable();
		TurretPIDController->Disable();

		ArmLockonTimer->Stop();
		ArmLockonTimer->Reset();
	}
	if(isShooting)
	{
		ArmPIDController->Disable();
		TurretPIDController->Disable();
		ArmPIDController->Reset();
		TurretPIDController->Reset();
	}
	if(CurrentEnableTracking)
	{
		// When auto-aiming, turn on the shooter wheels
		Shooter = 1.0f;

		ArmPIDController->Enable();
		TurretPIDController->Enable();
		if((fabs(LastMoveByDegreesX) > LOCKON_DEGREES_X) || (fabs(LastMoveByDegreesY) > LOCKON_DEGREES_Y))
		{
			ArmLockonTimer->Reset();
		}
		else if((ArmLockonTimer->Get() > LOCKON_SECONDS) && (!isShooting))
		{
			// if the user was holding the shooter wheels on AND we have been locked on,
			// then do a quick auto shot.
			if(Shooter == 1.0f)  // NOTE, we now force Shooter on when auto-aiming (top of this CurrentEnableTracking block)
			{
				if(LastShotTimer->Get() > 1.5f)
				{
					FullShotQuick();
					printf("SHOT!\r\n");
					ArmLockonTimer->Reset();
					LastShotTimer->Reset();
					LastShotTimer->Start();
				}
			}
			else
			{
				printf("SHOT!\r\n");
			}
		}
		else if (Ball)
		{
			FullShotQuick();
			printf("Manual shot during tracking!\r\n");
			ArmLockonTimer->Reset();
		}
	}
	FullShotUpdate();

	//TurretPIDController->SetPID(P*.001f, I*.00001f, 0.0f);

	if(ArmTimer->Get() > .01f)
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

	if (!isauto)
	{
		ArmShooter->Set(Shooter);
		ArmShooter2->Set(-Shooter);
	}

	if(!isauto && !CurrentEnableTracking)
	{
		PrevBallTog = CurrentBallTog;
		CurrentBallTog = Ball;

		PrevResetInput = CurrentResetInput;
		CurrentResetInput = Reset;

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
}
void ArmClass::StartTracking(int enable)
{
	// Set isTracking to true, then the Auto code will pass it in to Update
	// In Update, CurrentEnableTracking will get set.  This way the transition to
	// and from CurrentEnableTracking will work correctly.
	isTracking = enable;
	PreviousEnableTracking = false;
	//CurrentEnableTracking = (enable != 0);
}

void ArmClass::AutonomousTrackingUpdate(float tx, float ty, float crossX, float crossY)
{
	if(fabs(ArmPIDController->GetError()) < 1000)
	{
		ArmPIDController->SetPID(ARM_LIFT_P,ARM_LIFT_I,ARM_LIFT_D);
	}
	else
	{
		ArmPIDController->SetPID(ARM_LIFT_P,0,ARM_LIFT_D);
	}

	if(fabs(TurretPIDController->GetError()) < 200)
	{
		TurretPIDController->SetPID(ARM_TURRET_P,ARM_TURRET_I,ARM_TURRET_D);
	}
	else
	{
		TurretPIDController->SetPID(ARM_TURRET_P,0,ARM_TURRET_D);
	}

	if(ArmTimer->Get() > .01f)
	{
		HandleTarget(tx,ty,crossX,crossY);
		ArmTimer->Reset();
		ArmTimer->Start();
	}
}

void ArmClass::FullShot()
{
	isShooting = true;
	ShotStage = 0;
	ShotTimer->Reset();
	ShotTimer->Start();
}
void ArmClass::FullShotQuick()
{
	isShooting = true;
	ShotStage = 2;
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
				ShotStage = 2;
			}
			break;
		case 2:
			ShotExtend->Set(true);
			ShotRetract->Set(false);
			ShotTimer->Reset();
			ShotTimer->Start();
			ShotStage = 3;
			break;
		case 3:
			if(ShotTimer->Get() > .3f)
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
	ArmShooter->Set(1.0f);
	ArmShooter2->Set(-1.0f);
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
	TurretPIDController->SetSetpoint(TurretEncoder_Targ);
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
void ArmClass::SetArmStartPosition(int value)
{
	LifterEncoder->Reset_To_Value(value);
}
void ArmClass::SetArm(int targ)
{
	ArmPIDController->SetSetpoint(targ);
}

void ArmClass::ResetArm()
{
	SetArm(Preset_Arm_Floor);
}

void ArmClass::ResetTurret()
{
	SetTurret(0);
}

void ArmClass::HandleTarget(float centerX,float centerY,float calX,float calY)
{
	if(fabs (centerX) >= 1 || fabs(centerY) >= 1)
	{
		LastMoveByDegreesX = LastMoveByDegreesY = 360.0f;
		if (CurrentEnableTracking)
		{
			SetTurret(GetTurretEncoder());
			SetArm(GetLifterEncoder());
		}
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

		moveByX_Degrees = (calX - centerX) * (xFOV*.5f);
		moveByY_Degrees = (calY - centerY) * (yFOV*.5f);

		LastMoveByDegreesX = moveByX_Degrees;
		LastMoveByDegreesY = moveByY_Degrees;

		moveByX_Ticks = moveByX_Degrees / ARM_TURRET_DEGREES_PER_TICK;
		moveByY_Ticks = moveByY_Degrees / ARM_LIFT_DEGREES_PER_TICK;

		if(GetLifterEncoder() >= 0)
		{
			moveByX_Ticks *=-1.0f;
			moveByY_Ticks *=-1.0f;
		}

		if (CurrentEnableTracking)
		{
			const float FRACTION = 1.0f;
			int new_turret_target = GetTurretEncoder()-(moveByX_Ticks* FRACTION);
			int new_arm_target = GetLifterEncoder()-(moveByY_Ticks* FRACTION);
			SetTurret(new_turret_target); //.85
			SetArm(new_arm_target);
		}
	}
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
	SmartDashboard::PutData("PID", ArmPIDController);
	SmartDashboard::PutData("TUNERYaAXIS", TunerPIDController);
	SmartDashboard::PutData("TurretPID", TurretPIDController);
	SmartDashboard::PutNumber("TurretError",TurretPIDController->GetError());
	SmartDashboard::PutNumber("LastMoveByDegreesX", LastMoveByDegreesX);
	SmartDashboard::PutNumber("LastMoveByDegreesY", LastMoveByDegreesY);
	SmartDashboard::PutNumber("ArmLockonTimer", ArmLockonTimer->Get());
	SmartDashboard::PutNumber("Min", Min);
	SmartDashboard::PutNumber("Angle", Angle);

	SmartDashboard::PutBoolean("CurrentEnableTracking",CurrentEnableTracking);
	SmartDashboard::PutBoolean("LiftPIDEnabled", ArmPIDController->IsEnabled());
	SmartDashboard::PutBoolean("TurretPIDEnabled", TurretPIDController->IsEnabled());
	SmartDashboard::PutBoolean("isShooting", isShooting);

}
void ArmClass::UpdateEmergency(bool curtog)
{
	PrevEmergencyToggle = CurEmergencyToggle;
	CurEmergencyToggle = curtog;
	if(!PrevEmergencyToggle && CurEmergencyToggle)
	{
		BypassEncoderLimits = !BypassEncoderLimits;
	}
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
	if(abs(GetTurretEncoder()) <= 250)
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
	if(BypassEncoderLimits)
	{
		return cmd;
	}
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
		float min = 0;
		float dif = MIN_TURRET_CMD_LOW_ANGLE-MIN_TURRET_CMD_HIGH_ANGLE;
		float arm_angle = Lift_Encoder_To_Degrees(LifterEncoder->Get());
		if(arm_angle < 20)
		{
			arm_angle = 20;
		}
		if(arm_angle > 160)
		{
			arm_angle = 160;
		}
		if(arm_angle < 90.0f)
		{
			float norm = arm_angle / 70.0f;
			norm *= -1;
			norm += 1;
			min = (norm * dif) + MIN_TURRET_CMD_HIGH_ANGLE;
		}
		else
		{
			float norm = (arm_angle - 90.0f) / 70.0f;
			min = (norm * dif) + MIN_TURRET_CMD_HIGH_ANGLE;
		}
		Min = min;
		Angle = arm_angle;
		//randon .002 commands...
		if(cmd > 0.0f)
		{
			cmd = fmaxf(cmd, min);
		}
		else if (cmd < 0.0f)
		{
			cmd = fminf(cmd, -min);
		}
		//else if (cmd < .003f && cmd > -.003f)
		//{
		//	cmd = 0.0f;
		//}

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
	if(BypassEncoderLimits)
	{
		return cmd;
	}
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

	if(ispidcmd)
	{
		//randon .002 commands...
		if(cmd > 0.0f)
		{
			cmd = fmaxf(cmd, MIN_ARM_LIFT_CMD);
		}
		else if (cmd < -0.0f)
		{
			cmd = fminf(cmd, -MIN_ARM_LIFT_CMD);
		}
		//else if (cmd < .003f && cmd > -.003f)
		//{
		//	cmd = 0.0f;
		//}
	}

	return cmd;
}

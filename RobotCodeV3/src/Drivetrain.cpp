/*
 * Drivetrain.cpp
 *
 *  Created on: Jan 10, 2016
 *      Author: 987
 */

#include <Drivetrain.h>
#include "Defines.h"

Drivetrain::Drivetrain()
{
#if USING_MXP
	//uint8_t update_rate_hz = 50;
	imu = new AHRS (SerialPort::Port::kMXP);
//	imu = NULL;
#else
	imu = NULL;
#endif

	first_iteration = true;
	LeftDrive = new Victor(Tal_Drive_Left_1);
	LeftDrive2 = new Victor(Tal_Drive_Left_2);
	RightDrive = new Victor(Tal_Drive_Right_1);
	RightDrive2 = new Victor(Tal_Drive_Right_2);
	LeftEncoder = new Encoder(Encoder_Drive_Left_1, Encoder_Drive_Left_2, false,Encoder::EncodingType::k4X);
	RightEncoder = new Encoder(Encoder_Drive_Right_1, Encoder_Drive_Right_2, false,Encoder::EncodingType::k4X);
	disableInput = false;
	//gyro = new Gyro(1);
	//gyro->Calibrate(.007);
	//gyro->InitGyro();
	currentGyro = 0;
	targetGyro = 0;

	NormalShiftHigh = new Solenoid(Sol_Shifter_Inner_in);
	NormalShiftLow = new Solenoid(Sol_Shifter_Inner_out);

	NeutralEngaged = new Solenoid(Sol_Shifter_Outer_in);
	NeutralDisEngaged = new Solenoid(Sol_Shifter_Outer_out);
	Ebrakemult=.5f;
	PTO0 = new Solenoid(Sol_PTO_Enable);
	PTO1 = new Solenoid(Sol_PTO_Disable);

	CurrentPTOToggleTrig = false;
	PrevPTOToggleTrig = false;

	CurrentInnerShifterToggleTrig = false;
	PrevInnerShifterToggleTrig = false;
	ToggleState = -1;

	CurrentOuterShifterToggleTrig = false;
	PrevOuterShifterToggleTrig = false;


	mult = -.15f;

	ToggleState = 1;
	ToggleStateNeutral = -1;
	ToggleStatePTO =-1;
	inPTO = 0;
	TransitionState = 0;
	TransitionToPTO = false;
	TransitionToDriveTrain = false;
	LeftEncoder->Reset();
	RightEncoder->Reset();
	transitionwait = new Timer();
	transitionwait->Reset();
	syncMotors = false;
}
void Drivetrain::StandardTank(float Left, float Right)
{
	if(!disableInput)
	{
		float l = Left;
		float r = -Right;
		if(inPTO == 1)
		{
			if(l <= 0)
			{
				l = 0;
			}
		}
		if(syncMotors)
		{
			if(l <= 0)
			{
				l = 0;
			}
			LeftDrive->Set(l);
			LeftDrive2->Set(l);
			RightDrive->Set(-l);
			RightDrive2->Set(-l);
		}

		else
		{
			LeftDrive->Set(l);
			LeftDrive2->Set(l);
			RightDrive->Set(r);
			RightDrive2->Set(r);
		}
	}
}

void Drivetrain::StandardArcade(float fwd,float turn)
{
	float l = fwd + turn;
	float r = fwd - turn;
	StandardTank(l,r);
}
void Drivetrain::UpdateEBrake(int enable,int targ)
{
	PrevBrakeBTN = CurrentBrakeBTN;
	CurrentBrakeBTN = enable;
	if((CurrentBrakeBTN == 1)&&(PrevBrakeBTN == 0))
	{
		LeftEncoder->Reset();
		RightEncoder->Reset();
	}
	if(enable == 1)
	{
		int LeftSideTicks = -LeftEncoder->Get();
		//RightSideTicks = (int)(RightDriveEnc->Get()*2.5f);
		int RightSideTicks = -RightEncoder->Get();
		float Lerror = LeftSideTicks+targ;
		float Rerror = RightSideTicks+targ;

		float Leftout = -Lerror*Ebrakemult;
		float Rightout = -Rerror*Ebrakemult;

		LeftDrive->Set(-Leftout);
	//	LeftDrive1->Set(Leftout);
		RightDrive->Set(Rightout);
	//	RightDrive1->Set(-Rightout);
	}
	if((CurrentBrakeBTN == 0) && (PrevBrakeBTN == 1))
	{
		LeftEncoder->Reset();

		RightEncoder->Reset();
	}
}
void Drivetrain::Zero_Yaw()
{
	if (imu != NULL)
	{
		imu->ZeroYaw();
	}
}

void Drivetrain::ResetEncoders_Timers()
{
	LeftEncoder->Reset();
	RightEncoder->Reset();
}
int Drivetrain::GetLeftEncoder()
		{
	return LeftEncoder->Get();
		}
int Drivetrain::GetRightEncoder()
{
	return RightEncoder->Get();
}

void Drivetrain::IMUCalibration()
{
	if ( first_iteration && (imu != NULL))
	{
		bool is_calibrating = imu->IsCalibrating();
		if ( !is_calibrating ) {
			Wait( 0.3 );
			imu->ZeroYaw();
			first_iteration = false;
		}
	}
}
float Drivetrain::ComputeAngleDelta(float t)
{
	if(imu == NULL)
	{
		//return 0.0f;
	}
	float cur = imu->GetYaw();
	float err2 = t - cur;
	return err2;
}

void Drivetrain::Shifter_Update(bool DriveTrainShift,bool PTOEnable,bool syncEnable)
//0 pto
{
	if(syncEnable && inPTO == 1)
	{
		syncMotors = true;
	}
	else
	{
		syncMotors = false;
	}
	PrevInnerShifterToggleTrig = CurrentInnerShifterToggleTrig;
	CurrentInnerShifterToggleTrig = DriveTrainShift;

	PrevOuterShifterToggleTrig = CurrentOuterShifterToggleTrig;
	CurrentOuterShifterToggleTrig = PTOEnable;

	if(PrevInnerShifterToggleTrig == false && CurrentInnerShifterToggleTrig == true)
	{
		ToggleState = -ToggleState;
		ToggleStateNeutral = -1;
	}

	if(ToggleState == 1)
	{
		//Highgear
		NormalShiftHigh->Set(true);
		NormalShiftLow->Set(false);
	}
	else if(ToggleState == -1)
	{
		//LowGear
		NormalShiftHigh->Set(false);
		NormalShiftLow->Set(true);
	}


	if(PrevOuterShifterToggleTrig == false && CurrentOuterShifterToggleTrig == true)
	{
		if(inPTO == 1)
		{
			TransitionToDriveTrain = true;
		}
		else
		{
			TransitionToPTO = true;
		}

		TransitionState = 0;
	}

	if(ToggleStateNeutral == 1)
	{
		NeutralEngaged->Set(true);
		NeutralDisEngaged->Set(false);
	}
	else if(ToggleStateNeutral == -1)
	{
		NeutralEngaged->Set(false);
		NeutralDisEngaged->Set(true);
	}
	if(ToggleStatePTO == 1)
	{
		PTO0->Set(false);
		PTO1->Set(true);
	}
	else if(ToggleStatePTO == -1)
	{
		PTO0->Set(true);
		PTO1->Set(false);
	}

	if(TransitionToPTO)
	{
		switch(TransitionState)
		{
		case 0:
			ToggleState = -1;
			//ToggleStateNeutral = 1;
			ToggleStatePTO = -1;
			transitionwait->Reset();
			transitionwait->Start();
			TransitionState = 1;
			break;
		case 1:
			if(transitionwait->Get() >= .25f)
			{
				ToggleStatePTO = 1;
				TransitionState = 2;
				transitionwait->Reset();
				transitionwait->Start();
				inPTO = 1;
				TransitionToPTO = false;
			}
			break;
		case 2:
			if(transitionwait->Get() >= .25f)
			{
				ResetEncoders_Timers();
				LeftDrive->Set(.3f);
				LeftDrive2->Set(.3f);
				TransitionState = 3;
				disableInput = true;
			}
			break;
		case 3:
			if(GetLeftEncoder() >= 90)
			{
				LeftDrive->Set(0.0f);
				LeftDrive2->Set(0.0f);
				ResetEncoders_Timers();
				RightDrive->Set(-.3f);
				RightDrive2->Set(-.3f);
				TransitionState = 4;
			}
			break;
		case 4:
			if(GetRightEncoder() >= 90)
			{
				RightDrive->Set(0.0f);
				RightDrive2->Set(0.0f);
				TransitionState = 5;
				TransitionToPTO = false;
				disableInput = false;

			}
		}
	}

	if(TransitionToDriveTrain)
	{
		switch(TransitionState)
		{
		case 0:
			//ToggleState = -1;
			//ToggleStateNeutral = 1;
			ToggleStatePTO = -1;
			transitionwait->Reset();
			transitionwait->Start();
			TransitionState = 1;
			break;
		case 1:
			if(transitionwait->Get() >= 1.0f)
			{
				ToggleStateNeutral = -1;
				ToggleState = 1;
				TransitionState = 3;
				TransitionToPTO = false;
							inPTO = 0;
			}


			break;
		}
	}



}
void Drivetrain::PTO_Update(bool PTOenable)
{
	/*
	PrevPTOToggleTrig = CurrentPTOToggleTrig;
	CurrentPTOToggleTrig = PTOenable;

	if(PrevPTOToggleTrig == false && CurrentPTOToggleTrig == true)
	{
		ToggleState = -ToggleState;
	}
	else if(ToggleState == 1)
	{
		PTOEnable->Set(true);
		PTODisable->Set(false);
	}
	else if(ToggleState == -1)
	{
		PTOEnable->Set(false);
		PTODisable->Set(true);
	}

	if((PTOEnable->Get() == false))
	{
		ptoenable = true;
		ptoenable = false;
	}
	if((PTOEnable->Get() == true))
	{
		ptoenable = false;
		ptoenable = true;
	}
	*/
}
void Drivetrain::Drive_Auton(float Fwd, float Turn)
{
	StandardArcade(Fwd,Turn);
}
void Drivetrain::Failsafe_Update()
{

}
void Drivetrain::SendData()
{
	SmartDashboard::PutNumber("LeftEncoder",LeftEncoder->Get());
	SmartDashboard::PutNumber("RightEncoder",RightEncoder->Get());
	SmartDashboard::PutNumber("ToggleDT",ToggleState);
	SmartDashboard::PutNumber("ToggleNeut",ToggleStateNeutral);
	SmartDashboard::PutNumber("timer",transitionwait->Get());
	SmartDashboard::PutNumber("Yaw",  imu->GetYaw());
}

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
#if 0
	serial_port = new SerialPort(56700,SerialPort::kMXP);
	imu = new AHRS (SerialPort::Port::kMXP);
	if(imu)
	{
		LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", imu);
	}
#endif

	first_iteration = true;
	LeftDrive = new Talon(Tal_Drive_Left_1);
	LeftDrive2 = new Talon(Tal_Drive_Left_2);
	RightDrive = new Talon(Tal_Drive_Right_1);
	RightDrive2 = new Talon(Tal_Drive_Right_2);
	LeftEncoder = new Encoder(Encoder_Drive_Left_1, Encoder_Drive_Left_2, false,Encoder::EncodingType::k4X);
	RightEncoder = new Encoder(Encoder_Drive_Right_1, Encoder_Drive_Right_2, false,Encoder::EncodingType::k4X);
	disableInput = false;
//	gyro = new Gyro(1);
	//gyro->SetSensitivity(.007);
	//gyro->InitGyro();
	currentGyro = 0;
	targetGyro = 0;

	NormalShiftHigh = new Solenoid(Sol_Shifter_Inner_in);
	NormalShiftLow = new Solenoid(Sol_Shifter_Inner_out);

	NeutralEngaged = new Solenoid(Sol_Shifter_Outer_in);
	NeutralDisEngaged = new Solenoid(Sol_Shifter_Outer_out);

	PTO0 = new Solenoid(Sol_PTO_Enable);
	PTO1 = new Solenoid(Sol_PTO_Disable);

	CurrentPTOToggleTrig = false;
	PrevPTOToggleTrig = false;

	CurrentInnerShifterToggleTrig = false;
	PrevInnerShifterToggleTrig = false;
	ToggleState = -1;

	CurrentOuterShifterToggleTrig = false;
	PrevOuterShifterToggleTrig = false;


	mult = .15f;

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
void Drivetrain::StandardArcade(float Forward, float Turn)
{
	if(!disableInput)
	{
		float l = Forward;
		float r = -Turn;
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
#if USING_MXP
	if(imu == NULL)
	{
		return 0.0f;
	}
	float cur = imu->GetYaw();
	float err2 = t - imu->GetYaw();
#else
	float cur = gyro->get();
	float err2 = t - gyro->get();
#endif

	if(t < 0 && cur > 0)
	{
		cur -= 360;
	}
	else if (t > 0 && cur < 0)
	{
		cur += 360;
	}
#if USING_MXP
	float err1 = t - cur;
	if(fabs(err1) < fabs(err2))
	{
		return err1;
	}
	else
	{
		return err2;
	}
#else

		return err2;

#endif
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
void Drivetrain::Drive_Auton(float Forward, float Turn)
{
	StandardArcade(Forward,Turn);
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

}

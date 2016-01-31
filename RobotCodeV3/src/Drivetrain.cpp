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
	serial_port = new SerialPort(56700,SerialPort::kMXP);
	imu = new AHRS (SerialPort::Port::kMXP);
	if(imu)
	{
		LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", imu);
	}

	first_iteration = true;
	LeftDrive = new Talon(0);
	LeftDrive2 = new Talon(1);
	RightDrive = new Talon(2);
	RightDrive2 = new Talon(3);
	LeftEncoder = new Encoder(Encoder_Left_1, Encoder_Left_2, false,Encoder::EncodingType::k4X);
	RightEncoder = new Encoder(Encoder_Left_1, Encoder_Left_2, false,Encoder::EncodingType::k4X);

//	gyro = new Gyro(1);
	//gyro->SetSensitivity(.007);
	//gyro->InitGyro();
	currentGyro = 0;
	targetGyro = 0;

	ShifterHigh = new Solenoid(0);
	ShifterLow = new Solenoid(1);

	CurrentShifterToggleTrig = false;
	PrevShifterToggleTrig = false;
	ToggleState = -1;
	Highgear = false;
	Lowgear = false;

	mult = .15f;

	ToggleState = 1;

	LeftEncoder->Reset();
	RightEncoder->Reset();
}
void Drivetrain::StandardArcade(float Forward, float Turn)
{
	float l = Forward;
	float r = -Turn;
if(ToggleState == -1)
{
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
	if ( first_iteration )
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
void Drivetrain::Shifter_Update(bool ShifterEnable)
//0 pto
{
	PrevShifterToggleTrig = CurrentShifterToggleTrig;
	CurrentShifterToggleTrig = ShifterEnable;

	if(PrevShifterToggleTrig == false && CurrentShifterToggleTrig == true)
	{
		ToggleState = -ToggleState;
	}
	else if(ToggleState == 1)
	{
		ShifterHigh->Set(true);
		ShifterLow->Set(false);
	}
	else if(ToggleState == -1)
	{
		ShifterHigh->Set(false);
		ShifterLow->Set(true);
	}

	if((ShifterHigh->Get() == false))
	{
		Lowgear = true;
		Highgear = false;
	}
	if((ShifterHigh->Get() == true))
	{
		Lowgear = false;
		Highgear = true;
	}

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
	SmartDashboard::PutNumber("LeftEncoder",LeftEncoder->Get());
}

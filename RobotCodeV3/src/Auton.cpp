/*
 * Auton.cpp
 *
 *  Created on: Jan 29, 2016
 *      Author: HighRollers
 */

#include "Auton.h"
#include "HRLogger.h"
#include "math.h"
#include "stdio.h"
#include "defines.h"
#include "MyRobot.h"


Auton::Auton
(
Drivetrain *D,
ArmClass *A,
DriverStation *Ds
)
{
	DriveTrain = D;
	Arm = A;
	HRLogger::Log("autodrive created\r\n");
	ds = Ds;
	turningp = -.1f;
	AutonTimer = new Timer();
	SendTimer =new Timer();
	Abort = false;
}
void Auton::Auto_Start()
{
	DriveTrain->imu->ZeroYaw();
	AutonTimer->Reset();
	AutonTimer->Start();

	SendTimer->Reset();
	SendTimer->Start();
	DriveTrain->ResetEncoders_Timers();

}
void Auton::Auto_End()
{

}
bool Auton::Running()
{
	if(Abort)
	{
		return false;
	}
	else if (ds->IsOperatorControl() == true){return false;}
	else if (ds->IsEnabled() == false){return false;}
	return true;
}
void Auton::AutonWait(float Seconds)
{
	float targ = AutonTimer->Get() + Seconds;
	while((AutonTimer->Get() < targ)&&(Running()))
	{
	}
}
void Auton::AutonWait2(float Seconds,int brake)
{
	float targ = AutonTimer->Get() + Seconds;
	DriveTrain->ResetEncoders_Timers();
	while((AutonTimer->Get() < targ)&&(Running()))
	{
	}
	DriveTrain->Drive_Auton(0.0f,0.0f);
}
void Auton::Auto_DriveTimer(float Forward, float Turn, float Ticks, float seconds)
{
	DriveTrain->Drive_Auton(Forward, Turn);
	AutonWait(seconds);
	DriveTrain->Drive_Auton(0.0f, 0.0f);
}
void Auton::Auto_GYROTURN(float heading)
{
	DriveTrain->ResetEncoders_Timers();

	float angle_error = DriveTrain->ComputeAngleDelta(heading);
	float turn = DriveTrain->mult * angle_error;

	while((fabs(angle_error) > 3.0f)&&(Running()))
	{
		angle_error = DriveTrain->ComputeAngleDelta(heading);
		turn = DriveTrain->mult * angle_error;

		Auto_System_Update();
		DriveTrain->StandardArcade(0,turn);
	}
}
void Auton::Auto_GYROTURN_TIMED(float heading, float seconds)
{
	DriveTrain->ResetEncoders_Timers();

	float timtarg = AutonTimer->Get()+seconds;

	while(AutonTimer->Get() < timtarg && Running())
	{
		float angle_error = DriveTrain->ComputeAngleDelta(heading);
		float turn = DriveTrain->mult * angle_error;

		Auto_System_Update();
		DriveTrain->StandardArcade(0,turn);
	}
	DriveTrain->StandardArcade(0,0);
}
void Auton::Auto_GYROSTRAIGHT(float forward, float ticks, float desheading)
{
	DriveTrain->ResetEncoders_Timers();
	float MAINTAIN = desheading;
	float GYRO_P = DriveTrain->mult;

	float angle_error = MAINTAIN - DriveTrain->currentGyro;
	float turn = GYRO_P * angle_error;
	if(ticks > 0)
	{
		while((-DriveTrain->GetLeftEncoder() < ticks)&&(Running()))
		{
			float err = DriveTrain->ComputeAngleDelta(MAINTAIN);
			turn = err * GYRO_P;

			DriveTrain->StandardArcade(forward, turn);
			Auto_System_Update();
			//DriveTrain->FailSafe_Update;
		}
	}
	else
	{
		while((-DriveTrain->GetLeftEncoder() > ticks)&&(Running()))
		{
			float err = DriveTrain->ComputeAngleDelta(MAINTAIN);
			turn = err * GYRO_P;

			DriveTrain->StandardArcade(forward, turn);
			Auto_System_Update();
			//DriveTrain->Failsafe_Update();
		}
	}
	DriveTrain->Drive_Auton(0,0);
}
void Auton::Auto_DriveGyro_Encoder(float Forward, float Angle, float Ticks)
{
	DriveTrain->ResetEncoders_Timers();
	Auto_GYROTURN(Angle);
	AutonWait(.7f);
	Auto_GYROSTRAIGHT(Forward, Ticks, Angle);
}
bool Auton::Auto_System_Update()
{
	if(Running())
	{
#if !USING_MXP
		DriveTrain-currentGyro = DriveTrain->gyro->GetAngle();
#else
		DriveTrain->currentGyro = DriveTrain->imu->GetYaw();
#endif
		SendData();

		if(AutonTimer->Get() > 14.95)
		{

		}
		Wait(.001f);
	}
	return true;
}
void Auton::SendData()
{
	if(SendTimer->Get() > .05)
	{
		SendTimer->Reset();
		SendTimer->Start();

		DriveTrain->SendData();
		Arm->SendData();
		SmartDashboard::PutNumber("AUTOTIMER",AutonTimer->Get());
	}
}
/*void Auto_fullShot()
{

}*/

void Auton::Auto_DriveEncoder(float Forward, float Turn, float Ticks)
{
	DriveTrain->ResetEncoders_Timers();

	if(Ticks > 0)
	{
		while((-DriveTrain->GetLeftEncoder() < Ticks)&& Running())
		{
			DriveTrain->Drive_Auton(Forward ,Turn);
			Auto_System_Update();
			//DriveTrain->Failsafe_Update();
		}
	}
	else
	{
		while((-DriveTrain->GetLeftEncoder() > Ticks)&& Running())
		{
			DriveTrain->Drive_Auton(Forward, Turn);
			Auto_System_Update();
			//DriveTrain->FailSafe_Update();
		}
	}
	printf("Finished Driving");
	DriveTrain->Drive_Auton(0, 0);
}
/*void Auto_CameraAim()
{

}*/

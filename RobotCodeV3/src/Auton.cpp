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
DriverStation *Ds
)
{
	DriveTrain = D;
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
	//AutonTimer->Reset;
	AutonTimer->Start();

	SendTimer->Reset();
	SendTimer->Start();
	//Drivetrain->ResetEncoders_Timers();

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
/*void Auton::Auto_DriveTimer(float Forward, float Turn, float Ticks)
{

}
void Auton::Auto_GYROTURN(float heading)
{

}
void Auton::Auto_GYROTURN_TIMED(float heading, float seconds)
{

}
void Auton::Auto_GYROSTRAIGHT(float forward, float ticks, float desheading)
{

}
void Auton::Auto_DriveGyro_Encoder(float Forward, float Ticks)
{

}
void Auton::SendData()
{
	if(SendTimer->Get() > .05)
	{
		SendTimer->Reset();
		SendTimer->Start();

		Auton->SendData();
		SmartDashboard::PutNumber("AUTOTIMER",AutonTimer->Get());
	}
}
void Auto_fullShot()
{

}

void Auto_DriveEncoder(float Forward, float Turn, float Ticks)
{

}
void Auto_CameraAim()
{

}*/

/*
 * Drivetrain.cpp
 *
 *  Created on: Jan 10, 2016
 *      Author: 987
 */

#include <Drivetrain.h>
#include "Defines.h"
int te = 0;
Drivetrain::Drivetrain()
{
	LeftDrive = new Talon(0);
	LeftDrive2 = new Talon(1);
	RightDrive = new Talon(2);
	RightDrive2 = new Talon(3);

	ArmShooter = new Talon(4);
	ArmShooter2 = new Talon(5);
	ArmLifter = new Talon(6);
	ArmTurret = new Talon(7);

	ShifterHigh = new Solenoid(0);
	ShifterLow = new Solenoid(1);

	BallIn = new Solenoid(4);
	BallPusher = new Solenoid(5);


	CurrentShifterToggleTrig = false;
	PrevShifterToggleTrig = false;
	ToggleState = -1;
	Highgear = false;
	Lowgear = false;

	ToggleState = 1;
	CurrentBallTog = false;
	PrevBallTog = false;
}
void Drivetrain::StandardArcade(float Forward, float Turn, float Arm, float Lift, float Turret)
{
	float l = Forward;
	float a = Arm;
	float r = -Turn;
	float b = Lift;
	float t = Turret;
/*if(ToggleState == -1)
{
	LeftDrive->Set(l);
	LeftDrive2->Set(l);
	RightDrive->Set(-1);
	RightDrive2->Set(-1);

}
else
{*/
	LeftDrive->Set(l);
	LeftDrive2->Set(l);
	RightDrive->Set(r);
	RightDrive2->Set(r);
//}

	ArmShooter->Set(a);
	ArmShooter2->Set(-a);
	ArmLifter->Set(b);
	ArmTurret->Set(t);
}
Drivetrain::~Drivetrain() {
	// TODO Auto-generated destructor stub
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
void Drivetrain::Arm_Update(bool Ball)
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

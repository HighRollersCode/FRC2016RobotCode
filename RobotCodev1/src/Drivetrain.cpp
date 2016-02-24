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

	ShifterHigh = new Solenoid(0);
	ShifterLow = new Solenoid(1);

	CurrentShifterToggleTrig = false;
	PrevShifterToggleTrig = false;
	ToggleState = -1;
	Highgear = false;
	Lowgear = false;

}
void Drivetrain::StandardArcade(float Forward, float Turn, float Arm, float Lift, float Turret)
{
	float l = Forward;
	float r = -Turn;
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

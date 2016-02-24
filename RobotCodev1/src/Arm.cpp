/*
 * Arm.cpp
 *
 *  Created on: Feb 17, 2016
 *      Author: 987
 */

#include <Arm.h>

Arm::Arm() {
	ArmShooter = new Talon(4);
	ArmShooter2 = new Talon(5);
	ArmLifter = new Talon(6);
	ArmTurret = new Talon(7);

	BallIn = new Solenoid(4);
	BallPusher = new Solenoid(5);

	ToggleState = 1;
	CurrentBallTog = false;
	PrevBallTog = false;
}
void Arm::Update()
{
	float a = Shooter;
	float b = Lift;
	float t = Turret;

	ArmShooter->Set(a);
	ArmShooter2->Set(-a);
	ArmLifter->Set(b);
	ArmTurret->Set(t);
}
void Arm::Arm_Update(bool Ball)
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
Arm::~Arm() {
	// TODO Auto-generated destructor stub
}


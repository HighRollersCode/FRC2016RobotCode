
#ifndef MYROBOT_H
#define MYROBOT_H

#include "WPILib.h"
#include "DriveTrain.h"
#include "Arm.h"
#include "Intake.h"
#include <unistd.h>
#include <stdio.h>
#include "CollisionManager.h"
class RobotDemo: public SampleRobot
{
public:

	float commandForward;
	float commandArmShooter;
	float commandTurn;
	float commandLift;
	float commandTurret;
	float commandintake;
	float commandintakelift;
	Joystick *rightStick;			// joystick 2
	Joystick *leftStick;			// joystick 1
	Joystick *turretStick;			// joystick 3
	Drivetrain *DriveTrain;
	IntakeClass *Intake;
	ArmClass *Arm;
	CollisionManager *CollManager;

	RobotDemo(void);
	~RobotDemo(void);

	static RobotDemo * Get() { return TheRobot; }

	void Autonomous(void);
	void UpdateInputs();
	void OperatorControl(void);
	protected:

	static RobotDemo * TheRobot;
};

#endif

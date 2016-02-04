#include "MyRobot.h"

RobotDemo * RobotDemo::TheRobot = NULL;
//auto grip = NetworkTable::GetTable("grip");
RobotDemo::RobotDemo(void)
{
	TheRobot = this;
	commandForward = 0;
	commandArmShooter = 0;
	commandTurn = 0;
	commandLift = 0;
	commandTurret = 0;
	commandintake = 0;
	commandintakelift = 0;

	leftStick = new Joystick(0);
	rightStick = new Joystick(1);			// create the joysticks
	turretStick = new Joystick(2);
	DriveTrain = new Drivetrain();
	Intake = new IntakeClass(8,9,0,0);
	Arm = new ArmClass();
	const char * const JAVA = "/usr/local/frc/JRE/bin/java";
	char *GRIP_ARGS[5] = { "java", "-jar", "/home/lvuser/grip.jar", "/home/lvuser/project.grip", NULL };
	if (fork() == 0)
	{
		if (execv(JAVA, GRIP_ARGS) == -1)
		{
			perror("Error running GRIP");
		}
	}
}
RobotDemo::~RobotDemo(void)
{
	TheRobot = NULL;
}
void RobotDemo::Autonomous(void)
{
}
void RobotDemo::UpdateInputs()
{
	//float deadzone = .1;
	commandForward = leftStick->GetY();
	commandTurn = rightStick->GetY();
	commandArmShooter = turretStick->GetZ();
	commandLift = turretStick->GetY();
	commandTurret = turretStick->GetX();

	/*if(rightStick->GetY() == deadzone || rightStick->GetY() > 0 )
	{
		commandTurn = rightStick->GetY()-rightStick->GetY();
	}
	if(rightStick->GetY() == -deadzone || rightStick->GetY() < 0 )
	{
				commandTurn = rightStick->GetY()+rightStick->GetY();
	}*/
	if(leftStick->GetTrigger())
	{
		commandintake = .5f;
	}
	else if(leftStick->GetRawButton(3))
	{
		commandintake = -.5f;
	}
	else
	{
		commandintake = 0;
	}
	if(-rightStick->GetRawButton(3))
	{
		commandintakelift = 1.0f;
	}
	else if(rightStick->GetRawButton(2))
	{
		commandintakelift = 1.0f;
	}
	else
	{
		commandintakelift = 0;
	}
	if(turretStick->GetRawButton(6))
	{
		commandArmShooter = (turretStick->GetZ()-1)*.5f;
	}
	else if(turretStick->GetRawButton(7))
	{
		commandArmShooter = -(turretStick->GetZ()-1)*.5f;
//		commandArmShooter = 0.5f;
	}
	else
	{
		commandArmShooter = 0;
	}
}
void RobotDemo::OperatorControl(void)
{
	while (IsOperatorControl())
	{
		UpdateInputs();
		DriveTrain->StandardArcade(-commandForward, -commandTurn);
		Arm->Motors(-commandLift, -commandArmShooter, -commandTurret);
		Intake->Motors(-commandintake, -commandintakelift);
		DriveTrain->Shifter_Update(rightStick->GetTrigger());
		Arm->Arm_Update(turretStick->GetTrigger());
		Wait(0.001);
	}

}
START_ROBOT_CLASS(RobotDemo);

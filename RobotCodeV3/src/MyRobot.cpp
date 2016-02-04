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
	commandintake = 0;
	commandintakelift = 0;

	leftStick = new Joystick(0);
	rightStick = new Joystick(1);			// create the joysticks
	turretStick = new Joystick(2);
	DriveTrain = new Drivetrain();
#if 0
	Intake= new IntakeClass(0,0,0,0);
	Arm = new ArmClass();
#endif

#if 0 // disable GRIP for now...
	const char * const JAVA = "/usr/local/frc/JRE/bin/java";
	char *GRIP_ARGS[5] = { "java", "-jar", "/home/lvuser/grip.jar", "/home/lvuser/project.grip", NULL };
	if (fork() == 0)
	{
		if (execv(JAVA, GRIP_ARGS) == -1)
		{
			perror("Error running GRIP");
		}
	}
#endif
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
#if 0
	//float deadzone = .1;
	commandForward = leftStick->GetY();
	commandTurn = rightStick->GetY();
	commandArmShooter = turretStick->GetZ();
	commandLift = turretStick->GetY();
	commandintake = leftStick->GetTrigger();

	/*if(rightStick->GetY() == deadzone || rightStick->GetY() > 0 )
	{
		commandTurn = rightStick->GetY()-rightStick->GetY();
	}
	if(rightStick->GetY() == -deadzone || rightStick->GetY() < 0 )
	{
				commandTurn = rightStick->GetY()+rightStick->GetY();
	}*/
	if(turretStick->GetRawButton(6))
	{
		commandArmShooter = (turretStick->GetZ()-1)*.5f;
	}
	else if(turretStick->GetRawButton(7))
	{
		commandArmShooter = (-turretStick->GetZ()+1)*.5f;
	}
	else
	{
		commandArmShooter = 0;
	}
#endif
}
void RobotDemo::OperatorControl(void)
{
#if 0
	while (IsOperatorControl())
	{
		UpdateInputs();
		DriveTrain->StandardArcade(-commandForward, -commandTurn);
		DriveTrain->Shifter_Update(rightStick->GetTrigger());
		Arm->Arm_Update(turretStick->GetTrigger());
		Wait(0.001);
	}
#endif
}
START_ROBOT_CLASS(RobotDemo);

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
	Intake = new IntakeClass();
	Arm = new ArmClass();
	CollManager = new CollisionManager(Intake, Arm);
	//std::shared_ptr<USBCamera> camera(new USBCamera("cam0", true));
	//camera->SetExposureManual(50); // change this value
	//camera->SetBrightness(20); // change this value
	//CameraServer::GetInstance()->StartAutomaticCapture("cam0");
SmartDashboard::init();
//#if 0 // GRIP

	/*const char * const JAVA = "/usr/local/frc/JRE/bin/java";
	char *GRIP_ARGS[5] = { "java", "-jar", "/home/lvuser/grip.jar", "/home/lvuser/project.grip", NULL };
	if (fork() == 0)
	{
		if (execv(JAVA, GRIP_ARGS) == -1)
		{
			perror("Error running GRIP");
		}
	}
	*/
//#endif

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
	commandArmShooter = 0;

	/*if(rightStick->GetY() == deadzone || rightStick->GetY() > 0 )
	{
		commandTurn = rightStick->GetY()-rightStick->GetY();
	}
	if(rightStick->GetY() == -deadzone || rightStick->GetY() < 0 )
	{
				commandTurn = rightStick->GetY()+rightStick->GetY();
	}*/
	if(rightStick->GetTrigger())
	{
		commandintake = -1.0f;
		commandArmShooter = 1.0f;
	}
	else if(rightStick->GetRawButton(3))
	{
		commandintake = 0.5f;
	}
	else
	{
		commandintake = 0;
	}
	if(leftStick->GetRawButton(2))
	{
		commandintakelift = -.7f;
	}
	else if(leftStick->GetRawButton(3))
	{
		commandintakelift = .7f;
	}
	else
	{
		commandintakelift = 0;
	}
	if(turretStick->GetRawButton(6))
	{
		commandArmShooter = -1.0f;
	}
	else if(turretStick->GetRawButton(7))
	{
		//commandArmShooter = -(turretStick->GetZ()-1)*.f;
		commandArmShooter = 1.0f;
	}
}
void RobotDemo::OperatorControl(void)
{
	while (IsOperatorControl())
	{
	     //auto grip = NetworkTable::GetTable("grip");

		/* Get published values from GRIP using NetworkTables */
		//auto centers = grip->get("targets/centers", llvm::ArrayRef<double>());

		UpdateInputs();

		DriveTrain->StandardArcade(-commandForward, -commandTurn);
		Arm->Update(-commandLift, -commandArmShooter, -commandTurret, turretStick->GetTrigger(), turretStick->GetRawButton(11),turretStick->GetRawButton(2),0,0);
		Intake->Update(-commandintake, -commandintakelift);
		DriveTrain->Shifter_Update(leftStick->GetTrigger(), leftStick->GetRawButton(10),leftStick->GetRawButton(11));
		//DriveTrain->PTO_Update(leftStick->GetRawButton(4));
		DriveTrain->SendData();
		Intake->SendData();
		Arm->SendData();
		CollManager->Update(turretStick->GetRawButton(5), rightStick->GetRawButton(4), leftStick->GetRawButton(5));
		SmartDashboard::PutNumber("INTAKELIFT",commandintakelift);
		Wait(0.001);
	}

}
START_ROBOT_CLASS(RobotDemo);

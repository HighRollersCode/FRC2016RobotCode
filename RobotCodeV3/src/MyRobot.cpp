#include "MyRobot.h"

class ShutdownJetsonCommand : public Command
{
public:
	ShutdownJetsonCommand() {}
	virtual ~ShutdownJetsonCommand() {}

protected:
	virtual void Initialize()
	{
		RobotDemo::Get()->Shutdown_Jetson();
	}
	virtual void Execute() {

	}
	virtual bool IsFinished() { return false;}
	virtual void End() {}
	virtual void Interrupted() {}

};

RobotDemo * RobotDemo::TheRobot = NULL;
//auto grip = NetworkTable::GetTable("grip");
RobotDemo::RobotDemo(void)
{
	printf("Make Smartdash\r\n");
	SmartDashboard::init();
	SmartDashboard::PutData("Shutdown Jetson",new ShutdownJetsonCommand());

	TheRobot = this;
	commandLeft = 0;
	commandArmShooter = 0;
	commandRight = 0;
	commandLift = 0;
	commandTurret = 0;
	commandintake = 0;
	commandintakelift = 0;
	connectionattempts = 0;
	leftStick = new Joystick(0);
	rightStick = new Joystick(1);			// create the joysticks
	turretStick = new Joystick(2);
	DriveTrain = new Drivetrain();
	Intake = new IntakeClass();
	Arm = new ArmClass();
	CollManager = new CollisionManager(Intake, Arm);
	printf("Basic Initialization\r\n");
	TargClient = new TargetingSystemClient();
	TargClient->Connect("10.9.87.10",9870);
	printf("TargClient Initialized\r\n");
	AutonomousControl = new Auton(DriveTrain,Arm,&DriverStation::GetInstance(),Intake,CollManager,TargClient);

	ReConnectTimer = new Timer();
	ReConnectTimer->Reset();
	ReConnectTimer->Start();

	SmartDashTimer = new Timer();
	SmartDashTimer->Reset();
	SmartDashTimer->Start();

	SafeTimer = new Timer();
	SafeTimer->Reset();
	SafeTimer->Start();

	JetsonConnected = false;

	prevIntakeArm = false;
	curIntakeArm = false;

	Auto_Index = 0;
	m_ScriptSystem = 0;
	Init_Script_System();

	int connectionattempts = 0;
	printf("Will block to connect... \r\n");
	while ((connectionattempts < 3) && (TargClient->Get_Connected()  == false))
	{
		if(ReConnectTimer->Get() > 2)
		{
			TargClient->Connect("10.9.87.10",9870);
			ReConnectTimer->Reset();
			connectionattempts++;
		}
	}

	//std::shared_ptr<USBCamera> camera(new USBCamera("cam0", true));
	//camera->SetExposureManual(50); // change this value
	//camera->SetBrightness(20); // change this value
	//CameraServer::GetInstance()->StartAutomaticCapture("cam0");

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
void RobotDemo::Jetson_Connection()
{
	if(TargClient->Get_Connected() == false && ReConnectTimer->Get() > 5)
	{
		TargClient->Connect("10.9.87.10",9870);
		ReConnectTimer->Reset();
	}
	JetsonConnected = true;
}
RobotDemo::~RobotDemo(void)
{
	TheRobot = NULL;
}
void RobotDemo::Shutdown_Jetson(void)
{
	TargClient->Shutdown_Jetson();
}
void RobotDemo::Disabled()
{
	while(IsDisabled())
	{
		Jetson_Connection();
	}
}
void RobotDemo::LightUpdate()
{
	if(Arm->GetLifterEncoder() > 0)
	{
		TargClient->FlipEnable();
	}
	else
	{
		TargClient->FlipDisable();
	}
	//when the arm is low, we turn off tracking and clear image
	if(Arm->GetLifterEncoder() < Preset_Arm_Safe_Zone)
	{
		TargClient->EqualizeEnable();
	}
	else
	{
		TargClient->EqualizeDisable();
	}
	if(Arm->GetLifterEncoder() < Preset_Arm_Safe_Zone)
	{
		Arm->TargetingLights->Set(Relay::kOff);
	}
	else
	{
		Arm->TargetingLights->Set(Relay::kForward);
	}
}
void RobotDemo::Autonomous(void)
{
	//DriveTrain->gyro->Reset();
		//HRLogger::Log("FMS ATTATCHED %d\n",DriverStation::GetInstance()->IsFMSAttached());
		//HRLogger::Log("AUTO BEGIN \n");
		Load_Scripts();
		printf("loaded\n");
		Arm->isauto = true;
		m_ScriptSystem->Run_Auto_Script(0);
		printf("ransettings\n");
		AutonomousControl->Auto_Start();
		printf("startfunction\n");
		m_ScriptSystem->Run_Auto_Script(Auto_Index);
		//HRLogger::Log("runauto\n");
		AutonomousControl->Auto_End();
		//HRLogger::Log("autoend\n");
}
void RobotDemo::UpdateInputs()
{
	//float deadzone = .1;
	commandLeft = leftStick->GetY();
	commandRight = rightStick->GetY();
	commandArmShooter = turretStick->GetZ();
	commandLift = turretStick->GetY()*.85f;
	commandTurret = turretStick->GetX()*.5f;
	commandArmShooter = 0;
	prevIntakeArm = curIntakeArm;
	curIntakeArm = rightStick->GetTrigger();

	/*if(rightStick->GetY() == deadzone || rightStick->GetY() > 0 )
	{http://roborio-987-frc.local/
		commandTurn = rightStick->GetY()-rightStick->GetY();
	}
	if(rightStick->GetY() == -deadzone || rightStick->GetY() < 0 )
	{
				commandTurn = rightStick->GetY()+rightStick->GetY();
	}*/
	if(CollManager->currentMode == CollisionManager::RobotMode::Intake)
	{
		if(curIntakeArm == true && prevIntakeArm == false)
		{
			Arm->ArmPIDController->Enable();
			Arm->SetArm(Preset_Arm_Intake);
			SafeTimer->Reset();
			SafeTimer->Start();
		}
		else if(curIntakeArm == false && prevIntakeArm == true)
		{
			Arm->ArmPIDController->Enable();
			Arm->SetArm(Preset_Arm_Safe_Zone);
			SafeTimer->Reset();
			SafeTimer->Start();
		}
	}
	if(SafeTimer->Get() > 1.0)
	{
		Arm->ArmPIDController->Disable();
		SafeTimer->Reset();
		SafeTimer->Stop();
	}
	if(rightStick->GetTrigger())
	{
		commandintake = -1.0f;
		commandArmShooter = .85f;
	}
	else if(rightStick->GetRawButton(3))
	{
		commandintake = 0.5f;
	}
	else
	{
		commandintake = 0;
	}
	if(leftStick->GetRawButton(3))
	{
		commandintakelift = -.7f;
	}
	else if(leftStick->GetRawButton(2))
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
		commandArmShooter = 0.50f;
	}
	/*if(turretStick->GetRawButton(2))
	{
		commandArmShooter = -1.0f;
	}*/
}

void RobotDemo::Send_Smartdashboard_Data(void)
{
	if(SmartDashTimer->Get() > .1f)
	{
		DriveTrain->SendData();
		Intake->SendData();
		Arm->SendData();
		SmartDashTimer->Reset();
		SmartDashboard::PutBoolean("Jetson Connection", JetsonConnected);
	}
}

void RobotDemo::OperatorControl(void)
{
	Arm->ArmPIDController->Reset();
	Arm->TurretPIDController->Reset();
	Intake->LiftPIDController->Reset();
	Arm->ArmPIDController->Disable();
	Arm->TurretPIDController->Disable();
	Intake->LiftPIDController->Disable();
	CollManager->transitioning = false;
	CollManager->currentMode = CollisionManager::Free;
	Arm->isauto = false;
	while (IsOperatorControl())
	{
		Send_Smartdashboard_Data();

	     //auto grip = NetworkTable::GetTable("grip");

		/* Get published values from GRIP using NetworkTables */
		//auto centers = grip->get("targets/centers", llvm::ArrayRef<double>());

		UpdateInputs();
		TargClient->Update();
		SmartDashboard::PutNumber("INTAKELIFT",commandintakelift);
		SmartDashboard::PutNumber("x",TargClient->Get_Target_Distance());
		SmartDashboard::PutNumber("y",TargClient->Get_Target_Angle());
		DriveTrain->StandardTank(-commandLeft, -commandRight);
		Arm->Update(commandLift, -commandArmShooter, -commandTurret, turretStick->GetTrigger(), turretStick->GetRawButton(11),
				turretStick->GetRawButton(2),TargClient->Get_Target_Distance(),TargClient->Get_Target_Angle(),
				TargClient->Get_Cal_X(),TargClient->Get_Cal_Y());
		Intake->Update(-commandintake, -commandintakelift);
		DriveTrain->Shifter_Update(leftStick->GetTrigger(), leftStick->GetRawButton(10),leftStick->GetRawButton(11));
		DriveTrain->PTO_Update(leftStick->GetRawButton(4));
		CollManager->Update(turretStick->GetRawButton(4), rightStick->GetRawButton(4), leftStick->GetRawButton(5), turretStick->GetRawButton(5));
		if(turretStick->GetRawButton(11))
		{
			TargClient->StartCalibrate();
		}
		if (turretStick->GetRawButton(8) && turretStick->GetRawButton(9))
		{
			Shutdown_Jetson();
		}

		if (turretStick->GetRawButton(10))
		{
			Arm->ResetEncoderTurret();
			Arm->ResetEncoderLifter();
			Intake->ResetEncoderLift();
			DriveTrain->ResetEncoders_Timers();
			DriveTrain->imu->ZeroYaw();
		}
		LightUpdate();
		Wait(0.001);
	}

}
START_ROBOT_CLASS(RobotDemo);

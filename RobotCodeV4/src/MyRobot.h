
#ifndef MYROBOT_H
#define MYROBOT_H

#include "WPILib.h"
#include "DriveTrain.h"
#include "Arm.h"
#include "Intake.h"
#include "Auton.h"
#include <unistd.h>
#include <stdio.h>
#include "CollisionManager.h"
#include "TargetingSystemClient.h"
#include "HRscript.h"



class RobotDemo: public SampleRobot
{
public:

	float commandLeft;
	float commandArmShooter;
	float commandRight;
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
	TargetingSystemClient *TargClient;
	Auton *AutonomousControl;

	Compressor *Comp;
	Relay *CompRelay;

	Timer *ReConnectTimer;
	Timer *SmartDashTimer;
	Timer *SafeTimer;
	Timer *ArmIntakeTimer;
	Timer *MatchTimer;
	int intele;
	int connectionattempts;

	bool JetsonConnected;
	bool prevIntakeArm;
	bool curIntakeArm;

	bool ConnectionPrevTog;
	bool ConnectionCurTog;
	bool DisConnectionPrevTog;
	bool DisConnectionCurTog;


	RobotDemo(void);
	~RobotDemo(void);

	static RobotDemo * Get() { return TheRobot; }

	void Disabled();
	void Autonomous(void);
	void UpdateInputs();
	void Init_Script_System();
	void Load_Scripts();
	void OperatorControl(void);
	void Send_Smartdashboard_Data(void);
	void LightUpdate();
	void ConnectionUpdate();
	void CompressorUpdate();

	void ResetState();
	void Shutdown_Jetson(void);
	void Jetson_Connection();

	int Auto_Index;
protected:

	HrScriptSystemClass * m_ScriptSystem;
	static RobotDemo * TheRobot;
};

#endif

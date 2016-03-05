/*
 * Arm.h
 *
 *  Created on: Jan 30, 2016
 *      Author: HighRollers
 */

#ifndef SRC_ARM_H_
#define SRC_ARM_H_

#include "WPILib.h"
class ResettableEncoderClass;



class ArmClass {
public:
	Victor *ArmShooter;
	Victor *ArmShooter2;
	Victor *ArmLifter;
	Victor *ArmTurret;
	Encoder *TurretEncoder;
	ResettableEncoderClass *LifterEncoder;

	Solenoid *ShotRetract;
	Solenoid *ShotExtend;

	bool CurrentBallTog;
	bool PrevBallTog;

	bool Resetting;
	bool CurrentResetInput;
	bool PrevResetInput;
	bool CurrentEnableTracking;
	bool PreviousEnableTracking;

	bool isShooting;
	Timer *ShotTimer;
	int ShotStage;

	bool isTracking;

	float LastMoveByDegreesX;
	float LastMoveByDegreesY;
	Relay *TargetingLights;
	int ResetState;

	int ArmLifterEncoder_Cur;
	int ArmLifterEncoder_Trag;
	int TurretEncoder_Cur;
	int TurretEncoder_Targ;

	float ArmLifterCommand_Prev;
	float ArmLifterCommand_Cur;
	float kpArmLifter;
	float TurretCommand_Prev;
	float TurretCommand_Cur;
	float kpTurret;

	Timer *ArmTimer;

	PIDController *ArmPIDController;
	PIDController *TurretPIDController;
	PIDController *TunerPIDController;

	void Update(float ArmLift, float Shooter, float Turret, bool Ball, bool Reset,bool EnableTracking,float cX, float cY,float calX,float calY);
	void UpdateLift(float ArmLift);
	void UpdateTurret(float Turret);
	int GetTurretEncoder();
	int GetLifterEncoder();
	void ResetEncoders_Timers2();
	void SetTurret(int targ);//,bool relative = false);
	void SetArm(int targ);
	void GoToArm();
	void ResetArm();
	void ResetTurret();
	void ResetPostion();
	void FullShot();
	void FullShotUpdate();
	void ShooterIntake();
	void ShooterOutake();
	void ShooterOff();
	void StartTracking(int enable);
	void AutonomousTrackingUpdate(float tx, float ty, float crossX, float crossY);

	void HandleTarget(float centerX, float centerY,float calX, float calY);

	void GotoShooting();
	void GotoTowerShot();
	//void GoToEndGame();
	void SendData();
	void ResetEncoderLifter();
	void ResetEncoderLifterDown();
	void ResetEncoderTurret();
	float FSign(float a);
	float Clamp_Target(float tar, float lowerlim, float upperlim);
	bool TurretRoughlyCentered();
	ArmClass();
	~ArmClass();
	bool isauto;

	// These two functions will return a modified command if the given command would push the arm into
	// an illegal configuration.  If the command would help move the arm out of the invalid
	// configuration it should return it unmodified.  If the arm is already in an invalid configuration
	// then these commands could return a command that moves it toward a valid configuration.
	// The 'ArmTurretVictorClass' and 'ArmLiftVictorClass' call these functions.
	float Turret_Encoder_To_Degrees(int enc);
	float Validate_Turret_Command(float cmd, bool ispidcmd = false);

	float Lift_Encoder_To_Degrees(int enc);
	float Compute_Lift_Error_Correction_Command(float error);
	float Validate_Lift_Command(float cmd, bool ispidcmd = false);

};

#endif /* SRC_ARM_H_ */

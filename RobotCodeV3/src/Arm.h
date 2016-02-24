/*
 * Arm.h
 *
 *  Created on: Jan 30, 2016
 *      Author: HighRollers
 */

#ifndef SRC_ARM_H_
#define SRC_ARM_H_

#include "WPILib.h"

class ArmClass {
public:
	Victor *ArmShooter;
	Victor *ArmShooter2;
	Victor *ArmLifter;
	Victor *ArmTurret;
	Encoder *TurretEncoder;
	Encoder *LifterEncoder;

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
	void SetTurret(int targ);
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
	void ResetEncoderTurret();
	float FSign(float a);
	float Clamp_Target(float tar, float lowerlim, float upperlim);
	bool TurretRoughlyCentered();
	ArmClass();
	~ArmClass();
	bool isauto;
};

#endif /* SRC_ARM_H_ */

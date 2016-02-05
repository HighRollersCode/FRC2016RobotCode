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
	Talon *ArmShooter;
	Talon *ArmShooter2;
	Talon *ArmLifter;
	Talon *ArmTurret;

	Encoder *TurretEncoder;
	Encoder *LifterEncoder;

	Solenoid *BallPusher;
	Solenoid *BallIn;

	bool CurrentBallTog;
	bool PrevBallTog;

	bool CurrentResetInput;
	bool PrevResetInput;


	int TurretEncoder_Cur;
	int TurretEncoder_Targ;

	float TurretCommand_Prev;
	float TurretCommand_Cur;
	float kpTurret;

	void Update(float Lift, float Arm, float Turret, bool Ball, bool Reset);
	int GetTurretEncoder();
	int GetLifterEncoder();
	void ResetEncoders_Timers2();
	void SetTurret(int targ);
	void ResetPostion();

	void SendData();
	ArmClass();
	~ArmClass();
};

#endif /* SRC_ARM_H_ */

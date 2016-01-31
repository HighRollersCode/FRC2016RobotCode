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

	Encoder *TurretEncoder;
	Encoder *LifterEncoder;

	Solenoid *BallPusher;
	Solenoid *BallIn;

	bool CurrentBallTog;
	bool PrevBallTog;

	void Motors(float Lift, float Arm);
	int GetTurretEncoder();
	int GetLifterEncoder();
	void ResetEncoders_Timers2();
	void Arm_Update(bool Ball);
	void SendData();
	ArmClass();
	~ArmClass();
};

#endif /* SRC_ARM_H_ */

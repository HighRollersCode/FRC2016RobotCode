/*
 * Arm.h
 *
 *  Created on: Feb 17, 2016
 *      Author: 987
 */

#ifndef SRC_ARM_H_
#define SRC_ARM_H_

#include "WPILib.h"

class ArmClass {
public:
	Talon *ArmLifter;
	Talon *ArmTurret;
	Talon *ArmShooter;
	Talon *ArmShooter2;

	Solenoid *BallPusher;
	Solenoid *BallIn;

	int ToggleState;

	bool CurrentBallTog;
	bool PrevBallTog;

	void Update();
	Arm();
	virtual ~Arm();
};

#endif /* SRC_ARM_H_ */

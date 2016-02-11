/*
 * CollisionManager.h
 *
 *  Created on: Feb 6, 2016
 *      Author: 987
 */

#ifndef SRC_COLLISIONMANAGER_H_
#define SRC_COLLISIONMANAGER_H_

#include "WPILib.h"
#include "Arm.h"
#include "Intake.h"
class CollisionManager {
public:

	enum RobotMode
	{
		Free = 0,
		Shooting,
		Intake,
		Defensive
	};
	IntakeClass *IntakeRef;
	ArmClass *ArmRef;

	bool ShooterState_Cur;
	bool ShooterState_Prev;
	bool IntakeState_Cur;
	bool IntakeState_Prev;
	bool DefensiveState_Cur;
	bool DefensiveState_Prev;

	int state;
	bool transitioning;
	RobotMode currentMode;
	CollisionManager(IntakeClass *IntakeRef, ArmClass *ArmRef );
	virtual ~CollisionManager();

	void EnterState(RobotMode mode);
	void Update(bool ShootingState, bool IntakeState, bool DefensiveState);
};

#endif /* SRC_COLLISIONMANAGER_H_ */

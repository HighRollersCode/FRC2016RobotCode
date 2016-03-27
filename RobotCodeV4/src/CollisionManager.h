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
#include "Defines.h"
class CollisionManager {
public:

	enum RobotMode
	{
		Free = 0,
		Shooting,
		Intake,
		Defensive,
		TowerShot,
		//EndGame
	};
	IntakeClass *IntakeRef;
	ArmClass *ArmRef;

	bool ShooterState_Cur;
	bool ShooterState_Prev;
	bool IntakeState_Cur;
	bool IntakeState_Prev;
	bool DefensiveState_Cur;
	bool DefensiveState_Prev;
	bool TowerShotState_Cur;
	bool TowerShotState_Prev;
	//bool EndGameState_Cur;
	//bool EndGameState_Prev;

	Timer *PresetTimer;

	int state;
	int counter;
	bool transitioning;
	RobotMode currentMode;
	CollisionManager(IntakeClass *IntakeRef, ArmClass *ArmRef );
	virtual ~CollisionManager();

	void EnterState(RobotMode mode);
	void Update(bool ShootingState, bool IntakeState, bool DefensiveState, bool TowerShotState);//, bool EndGameState);
	void SendData();
};

#endif /* SRC_COLLISIONMANAGER_H_ */

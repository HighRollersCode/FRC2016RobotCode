/*
 * CollisionManager.cpp
 *
 *  Created on: Feb 6, 2016
 *      Author: 987
 */

#include <CollisionManager.h>

CollisionManager::CollisionManager(IntakeClass *intake, ArmClass *arm ) {

	ShooterState_Cur = false;
	ShooterState_Prev = false;
	IntakeState_Cur = false;
	IntakeState_Prev = false;
	DefensiveState_Cur = false;
	DefensiveState_Prev = false;

	state = 0;
	transitioning = false;
	currentMode = RobotMode::Free;
	IntakeRef = intake;
	ArmRef = arm;
}

CollisionManager::~CollisionManager()
{
	// TODO Auto-generated destructor stub
}

void CollisionManager::EnterState(RobotMode mode)
{
	currentMode = mode;
	state = 0;
	transitioning = true;
}
void CollisionManager::Update(bool ShootingState, bool IntakeState, bool DefensiveState)
{
	int safezone = 4000;
	ShooterState_Prev = ShooterState_Cur;
	ShooterState_Cur = ShootingState;
	IntakeState_Prev = IntakeState_Cur;
	IntakeState_Cur = IntakeState;
	DefensiveState_Prev = DefensiveState_Cur;
	DefensiveState_Cur = DefensiveState;

	if(!ShooterState_Prev && ShooterState_Cur)
	{
		EnterState(RobotMode::Shooting);
	}

	if(!IntakeState_Prev && IntakeState_Cur)
	{
		EnterState(RobotMode::Intake);
	}

	if(!DefensiveState_Prev && DefensiveState_Cur)
	{
		EnterState(RobotMode::Defensive);
	}

	if(transitioning == true)
	{
		if(currentMode == RobotMode::Intake)
		{
			switch(state)
			{
			case 0 :
				//Reset Intske
				IntakeRef->GotoFloor();
				state = 1;
				break;
			case 1 :
				//Reset Turret and wait Intake
				if(fabs(IntakeRef->GetLiftEncoder()) <= 25)
				{
					state = 2;
					ArmRef->ResetTurret();
				}

				break;
			case 2 :
				if(fabs(ArmRef->GetTurretEncoder()) <= 20)
				{
					ArmRef->ResetArm();
					state = 3;

				}
				break;
			case 3:
				if(fabs(ArmRef->GetLifterEncoder() <= 25))
				{
					IntakeRef->GotoIntake();
					state = 4;
					transitioning = false;
				}
			}
		}
		else if(currentMode == RobotMode::Shooting)
		{
			switch(state)
			{
				case 0 :
					//Reset Intske
					IntakeRef->GotoFloor();
					state = 1;
					break;
				case 1 :
					//Reset Turret and wait Intake
					if(fabs(IntakeRef->GetLiftEncoder()) <= 20)
					{
						state = 2;
						ArmRef->GotoShooting();
						transitioning = false;

					}

					break;
				}
			}
		else if(currentMode == RobotMode::Defensive)
		{
			switch(state)
			{
				case 0 :
					//Reset Intske
					IntakeRef->GotoFloor();
					state = 1;
					break;
				case 1 :
					//Reset Turret and wait Intake
					if(fabs(IntakeRef->GetLiftEncoder()) <= 20)
					{
						state = 2;
						ArmRef->SetArm(5000);
					}

					break;
				case 2 :
					if(fabs(ArmRef->GetLifterEncoder()-5000) <= 70)
					{
						IntakeRef->GotoDefense();
						state = 3;
						transitioning = false;
					}
			}
		}
	}
}


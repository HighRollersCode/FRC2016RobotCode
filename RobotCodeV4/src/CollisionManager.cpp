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
	TowerShotState_Cur = false;
	TowerShotState_Prev = false;
	//EndGameState_Cur = false;
	//EndGameState_Prev = false;
	counter =0;
	PresetTimer = new Timer();
	PresetTimer->Reset();
	PresetTimer->Start();

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
void CollisionManager::Update(bool ShootingState, bool IntakeState, bool DefensiveState, bool TowerShotState)//, bool EndGameState)
{
	//int safezone = 4000;
	ShooterState_Prev = ShooterState_Cur;
	ShooterState_Cur = ShootingState;
	IntakeState_Prev = IntakeState_Cur;
	IntakeState_Cur = IntakeState;
	DefensiveState_Prev = DefensiveState_Cur;
	DefensiveState_Cur = DefensiveState;
	TowerShotState_Prev = TowerShotState_Cur;
	TowerShotState_Cur = TowerShotState;
	//EndGameState_Prev = EndGameState_Cur;
	//EndGameState_Cur = EndGameState;

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

	if(!TowerShotState_Prev && TowerShotState_Cur)
	{
		EnterState(RobotMode::TowerShot);
	}
	//if(!EndGameState_Prev && EndGameState_Cur)
	//{
		//EnterState(RobotMode::EndGame);
	//}

	if(transitioning == true)
	{
		if(currentMode == RobotMode::Intake)
		{
			switch(state)
			{
			case 0 :
				//Reset Intske
				IntakeRef->LiftPIDController->Enable();
				IntakeRef->GotoIntake();
				if (ArmRef->GetLifterEncoder() > 0)
				{
					ArmRef->ArmPIDController->Enable();
					ArmRef->SetArm(0);
				}

				state = 1;
				counter = 0;
				break;
			case 1 :
				//Reset Turret and wait Intake
				//REENABLE
				//ENABLE THIS WHEN THE INTAKE IS FUNCTIONAL
				counter = 3;
				/*
				if(fabs(IntakeRef->GetLiftEncoder()-Preset_Intake_Intake) <= 20)
				{
					counter++;
				}
				else
				{
					counter = 0;
				}
				*/
				if (counter > 2)
				{
					ArmRef->SetTurret(0);
					ArmRef->TurretPIDController->Enable();
					state = 2;
					counter = 0;
				}
				break;
			case 2 :
				if(fabs(ArmRef->GetTurretEncoder()) <= 15 * 60.0f/24.0f)
				{
					counter++;
				}
				else
				{
					counter= 0;
				}
				if(counter > 1)
				{
					ArmRef->ArmPIDController->Enable();
					ArmRef->ResetArm();
					state = 3;
				}
				break;
			case 3:
				if(fabs(ArmRef->GetLifterEncoder() - Preset_Arm_Floor <= 10))
				{
					ArmRef->ArmPIDController->Disable();
					ArmRef->TurretPIDController->Disable();
					IntakeRef->GotoIntake();
					Wait(.25f);
					ArmRef->GoToArm();
					state = 4;
					transitioning = false;
				}
				break;
			}
		}
		else if(currentMode == RobotMode::Shooting)
		{
			PresetTimer->Reset();
			PresetTimer->Start();
			switch(state)
			{
				case 0 :
					state = 1;
					IntakeRef->LiftPIDController->Enable();
					//Reset Intake
					ArmRef->ArmPIDController->Enable();
					ArmRef->TurretPIDController->Enable();
					IntakeRef->GotoIntake();
					break;
				case 1 :
					//Reset Turret and wait Intake
					//REENABLE
					if(fabs(IntakeRef->GetLiftEncoder()-Preset_Intake_Intake) <= 20)
					{
						state = 2;
						ArmRef->ResetTurret();
					}
					break;
				case 2 :
					if(fabs(ArmRef->GetTurretEncoder()) <= 5*60.0f/24.0f)
					{
						state = 3;
						ArmRef->GotoShooting();
					}
					break;
				case 3 :
					if(fabs(ArmRef->GetLifterEncoder() - Preset_Arm_Far_Shot) <= 20)
					{
						PresetTimer->Reset();
						PresetTimer->Start();
						state = 4;
					}
					break;
				case 4 :
					if(PresetTimer->Get() > 10)
					{
						state = 5;
						ArmRef->ArmPIDController->Disable();
						//	IntakeRef->LiftPIDController->Disable();
						ArmRef->TurretPIDController->Disable();
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
					IntakeRef->LiftPIDController->Enable();
					//Reset Intske
					IntakeRef->GotoIntake();
					state = 1;
					break;
				case 1 :
					//Reset Turret and wait Intake
					//REENABLE
					if(fabs(IntakeRef->GetLiftEncoder() - Preset_Intake_Intake) <= 20)
					{
						state = 2;
						ArmRef->SetArm(Preset_Arm_Defense);
						ArmRef->ResetTurret();
						ArmRef->ArmPIDController->Reset();
						ArmRef->ArmPIDController->Enable();
						ArmRef->TurretPIDController->Reset();
						ArmRef->TurretPIDController->Enable();
					}
					break;
				case 2 :
					if(fabs(ArmRef->GetLifterEncoder()-Preset_Arm_Defense) <= 200)
					{
						IntakeRef->GotoDefense();
						ArmRef->ArmPIDController->Disable();
						ArmRef->TurretPIDController->Disable();
						state = 3;
					}
					break;
				case 3:
					if(fabs(IntakeRef->GetLiftEncoder() - Preset_Intake_Defense) <= 20)
					{
						IntakeRef->LiftPIDController->Disable();
						state = 4;
						transitioning = false;
					}
					break;
			}
		}
		else if(currentMode == RobotMode::TowerShot)
		{
			switch(state)
			{
				case 0 :
					//Move Intake to Intake position
					IntakeRef->GotoIntake();
					IntakeRef->LiftPIDController->Enable();
					state = 1;
					break;
				case 1 :
					//wait for the intake to reach its target
					if(fabs(IntakeRef->GetLiftEncoder()-Preset_Intake_Intake) <= 20)
					{
						// center turret
						state = 2;
						ArmRef->ResetTurret();
						ArmRef->TurretPIDController->Enable();
					}
					break;
				case 2 :
					//wait for turret to reach zero
					if(fabs(ArmRef->GetTurretEncoder()) <= 20*60.0f/24.0f)
					{
						// move arm to tower shot height
						state = 3;
						ArmRef->GotoTowerShot();
						ArmRef->ArmPIDController->Enable();
					}
					break;
				case 3 :
					//Wait until arm is at the Tower Shot height
					if(fabs(ArmRef->GetLifterEncoder() - Preset_Arm_Tower_Shot) <= 50)
					{
						// raise the intake
						// gth - don't raise intake for tower shot, we want a better angle
						// IntakeRef->SetLift(Preset_Intake_Tower_Shot);

						ArmRef->ArmPIDController->Disable();
						ArmRef->TurretPIDController->Disable();
						transitioning = false;

						state = 4;
					}
					break;
				/* gth - don't raise intake for tower shot
				case 4 :
					//Wait until intake is raised
					if(fabs(IntakeRef->GetLiftEncoder()-Preset_Intake_Tower_Shot) <= 20)
					{
						state = 5;
						IntakeRef->LiftPIDController->Disable();
						ArmRef->ArmPIDController->Disable();
						ArmRef->TurretPIDController->Disable();
						transitioning = false;
					}
					break;
					*/
				}
			}
		/*else if(currentMode == RobotMode::EndGame)
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
						if(fabs(ArmRef->GetLifterEncoder()-5000) <= 200)
						{
							IntakeRef->GotoDefense();
							state = 3;
							transitioning = false;
						}
				}


			}
			*/


	}
}
void CollisionManager::SendData()
{
	SmartDashboard::PutNumber("CM State", state);
	SmartDashboard::PutBoolean("CM Trans", transitioning);
	SmartDashboard::PutNumber("CM Mode", currentMode);
}


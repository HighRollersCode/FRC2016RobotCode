/*
 * Auton.h
 *
 *  Created on: Jan 29, 2016
 *      Author: HighRollers
 */

#ifndef AUTON_H_
#define AUTON_H_

#include "WPILib.h"
#include "Drivetrain.h"
#include "Arm.h"
#include "CollisionManager.h"
#include "Intake.h"
#include "TargetingSystemClient.h"

class Auton
{
	public:
		Drivetrain *DriveTrain;
		ArmClass *Arm;
		IntakeClass *Intake;
		CollisionManager *Collision;
		DriverStation *ds;
		TargetingSystemClient *Targeting;
		Timer *AutonTimer;
		Timer *SendTimer;
		//float turningp;
		bool Abort;

		bool Running();
		void AutonWait(float Seconds);
		void AutonWait2(float Seconds, int brake);
		void AutonWaitForTarget(float Seconds);
		bool Auto_System_Update();

		void Auto_DriveTimer(float Forward, float Turn, float seconds);
		void Auto_Intake_On();
		void Auto_Intake_Off();
		void Auto_GYROTURN(float heading);
		void Auto_GYROTURN_TIMED(float heading, float seconds);
		void Auto_GYROSTRAIGHT(float forward, float ticks, float desheading);
		void Auto_DriveGyro_Encoder(float Forward, float Angle, float Ticks);

		void SendData();
		//void Auto_fullShot();

		void Auto_DriveEncoder(float Forward, float Turn, float Ticks);
		//void Auto_CameraAim();

		Auton
		(
				Drivetrain *D,
				ArmClass *A,
				DriverStation *Ds,
				IntakeClass *I,
				CollisionManager *C,
				TargetingSystemClient *T
		);
		~Auton();
		void Auto_Start();
		void Auto_End();
		void FireAll();
};

#endif /* AUTON_H_ */

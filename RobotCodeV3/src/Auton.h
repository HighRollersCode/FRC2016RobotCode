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

class Auton
{
	public:
		Drivetrain *DriveTrain;

		DriverStation *ds;
		Timer *AutonTimer;
		Timer *SendTimer;
		float turningp;
		bool Abort;

		bool Running();
		void AutonWait(float Seconds);
		void AutonWait2(float Seconds, int brake);
		bool Auto_System_Update();

		//void Auto_DriveTimer(float Forward, float Turn, float Ticks);

		//void Auto_GYROTURN(float heading);
		//void Auto_GYROTURN_TIMED(float heading, float seconds);
		//void Auto_GYROSTRAIGHT(float forward, float ticks, float desheading);
		//void Auto_DriveGyro_Encoder(float Forward, float Ticks);

		//void SendData();
		//void Auto_fullShot();

		//void Auto_DriveEncoder(float Forward, float Turn, float Ticks);
		//void Auto_CameraAim();

		Auton
		(
				Drivetrain *D,

				DriverStation *Ds
		);
		~Auton();
		void Auto_Start();
		void Auto_End();
		void FireAll();
};

#endif /* AUTON_H_ */

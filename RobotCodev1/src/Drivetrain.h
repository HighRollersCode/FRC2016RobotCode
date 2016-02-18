/*
 * Drivetrain.h
 *
 *  Created on: Jan 10, 2016
 *      Author: 987
 */
 #include "WPILib.h"
#ifndef SRC_DRIVETRAIN_H_
#define SRC_DRIVETRAIN_H_

class Drivetrain {


public:
	Talon *LeftDrive;
	Talon *RightDrive;
	Talon *LeftDrive2;
	Talon *RightDrive2;

	Solenoid *ShifterHigh;
	Solenoid *ShifterLow;

	bool CurrentShifterToggleTrig;
	bool PrevShifterToggleTrig;

	int ToggleState;
	bool Highgear;
	bool Lowgear;


	bool CurrentBallTog;
	bool PrevBallTog;
	Drivetrain();
	virtual ~Drivetrain();

	void StandardArcade(float Forward, float Turn, float Arm, float Lift, float Turret);
	void Shifter_Update(bool ShifterEnable);
	void Arm_Update(bool Ball);
};

#endif /* SRC_DRIVETRAIN_H_ */

/*
 * Drivetrain.h
 *
 *  Created on: Jan 10, 2016
 *      Author: 987
 */

#ifndef SRC_DRIVETRAIN_H_
#define SRC_DRIVETRAIN_H_

#include "WPILib.h"
#include "IMUProtocol.h"
#include "IMURegisters.h"
#include "AHRS.h"



class Drivetrain
{

public:
	AHRS *imu;
	SerialPort *serial_port;
	bool first_iteration;

	Talon *LeftDrive;
	Talon *RightDrive;
	Talon *LeftDrive2;
	Talon *RightDrive2;

	Encoder *LeftEncoder;
	Encoder *RightEncoder;

	Solenoid *ShifterHigh;
	Solenoid *ShifterLow;

	bool CurrentShifterToggleTrig;
	bool PrevShifterToggleTrig;

	int ToggleState;
	bool Highgear;
	bool Lowgear;

	Drivetrain();
	 ~Drivetrain();

	int GetLeftEncoder();
	int GetRightEncoder();
	void ResetEncoders_Timers();

	void IMUCalibration();
	void StandardArcade(float Forward, float Turn);
	void Shifter_Update(bool ShifterEnable);
	void Drive_Auton (float Forward, float Turn);
};

#endif /* SRC_DRIVETRAIN_H_ */

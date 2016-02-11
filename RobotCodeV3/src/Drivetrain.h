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

	//Gyro *gyro;

	float currentGyro;
	float targetGyro;

	Encoder *LeftEncoder;
	Encoder *RightEncoder;

	Solenoid *NormalShiftHigh;
	Solenoid *NormalShiftLow;
	Solenoid *NeutralEngaged;
	Solenoid *NeutralDisEngaged;

	Solenoid *PTO0;
	Solenoid *PTO1;

	bool CurrentPTOToggleTrig;
	bool PrevPTOToggleTrig;



	bool TransitionToPTO;
		bool TransitionToDriveTrain;
		int TransitionState;
		Timer *transitionwait;
		bool syncMotors;
	bool CurrentInnerShifterToggleTrig;
	bool PrevInnerShifterToggleTrig;

	int ToggleState;
	int ToggleStateNeutral;
	int ToggleStatePTO;
	int inPTO;


	bool CurrentOuterShifterToggleTrig;
	bool PrevOuterShifterToggleTrig;

	float mult;

	Drivetrain();
	~Drivetrain();

	int GetLeftEncoder();
	int GetRightEncoder();
	void ResetEncoders_Timers();

	void IMUCalibration();

	float ComputeAngleDelta(float t);

	void StandardArcade(float Forward, float Turn);
	void Shifter_Update(bool DriveTrainShift, bool PTOEnable,bool syncEnable);
	void PTO_Update(bool PTOEnable);
	void Drive_Auton (float Forward, float Turn);

	void Failsafe_Update();

	void SendData ();
};

#endif /* SRC_DRIVETRAIN_H_ */

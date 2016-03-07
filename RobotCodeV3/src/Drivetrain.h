/*
 * Drivetrain.h
 *
 *  Created on: Jan 10, 2016
 *      Author: 987
 */

#ifndef SRC_DRIVETRAIN_H_
#define SRC_DRIVETRAIN_H_

#include "WPILib.h"
#include "AHRS.h"
#include "AnalogGyro.h"


class Drivetrain
{
private:

	AHRS *imu;
	AnalogGyro *gyro;

public:


	bool first_iteration;

	Victor *LeftDrive;
	Victor *RightDrive;
	Victor *LeftDrive2;
	Victor *RightDrive2;

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

	bool disableInput;

	bool CurrentOuterShifterToggleTrig;
	bool PrevOuterShifterToggleTrig;

	float mult;

	float Ebrakemult;
	int PrevBrakeBTN;
	int CurrentBrakeBTN;

	Drivetrain();
	~Drivetrain();

	int GetLeftEncoder();
	int GetRightEncoder();
	void ResetEncoders_Timers();

	void Zero_Yaw();
	void IMUCalibration();
	float ComputeAngleDelta(float t);
	float GetHeading(void);

	void UpdateEBrake(int enable,int targ);

	void StandardTank(float Left, float Right);
	void StandardArcade(float fwd,float Turn);
	void Shifter_Update(bool DriveTrainShift, bool PTOEnable,bool syncEnable);
	void PTO_Update(bool PTOEnable);
	void Drive_Auton (float Left, float Right);

	void Failsafe_Update();

	void SendData ();
};

#endif /* SRC_DRIVETRAIN_H_ */

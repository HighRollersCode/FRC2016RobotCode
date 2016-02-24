/*
 * TargetingSystemClient.h
 *
 *  Created on: Feb 11, 2016
 *      Author: 987
 */

#ifndef SRC_TARGETINGSYSTEMCLIENT_H_
#define SRC_TARGETINGSYSTEMCLIENT_H_

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include "stdio.h"
#include "Timer.h"
#include "WPILib.h"

class TargetingSystemClient
{

public:
	TargetingSystemClient();
	~TargetingSystemClient();

	void Shutdown_Jetson();
	bool Is_Connected(void) { return m_Connected; }
	bool Connect(const char * server,unsigned short port);
	bool Update();
	bool Send(const char * data_out,int size);

	void Set_Turret_Angle(float a) { m_TurretAngle = a; }
	float Get_Target_Distance() { return m_TargetDistance; }
	float Get_Target_Angle() { return m_TargetAngle; }
	float Get_Target_Bank() { return m_BankAngle; }
	float Get_Cal_X(){return xCal;}
	float Get_Cal_Y(){return yCal;}
	float Get_Connected(){return m_Connected;}
	float Get_TargetArea(){return m_TargetArea;}
	void StartCalibrate();
	void FlipEnable();
	void FlipDisable();
	void EqualizeEnable();
	void EqualizeDisable();

protected:
	void Handle_Incoming_Data(char * data,int size);
	void Handle_Command(char *data);
	void Handle_Target(char *data);
	void Handle_Calibration(char *data);
	void Handle_CalibrationRefresh(char *data);
	float m_TurretAngle;
	float m_TargetDistance;
	float m_TargetAngle;
	float m_BankAngle;
	float m_TargetArea;
	bool m_Connected;
	// networking
	int m_SocketHandle;
	bool gotdata;
	float xCal;
	float yCal;
	Timer *m_CommTimer;
};

#endif //TARGETINGSYSTEMCLIENT_H

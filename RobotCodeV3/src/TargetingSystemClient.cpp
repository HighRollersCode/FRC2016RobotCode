/*
 * TargetingSystemClient.cpp
 *
 *  Created on: Feb 11, 2016
 *      Author: 987
 */

#include "TargetingSystemClient.h"
#include "stdio.h"
#include "stdlib.h"
#include <sys/socket.h>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
//#include "inetLib.h"
//#include "hostLib.h"
//#include "ioLib.h"
#include "string.h"

//#include "selectlib.h"


TargetingSystemClient::TargetingSystemClient() :
	m_TurretAngle(0.0f),
	m_TargetDistance(0.0f),
	m_TargetAngle(0.0f),
	m_BankAngle(0.0f),
	m_TargetArea(0.0f),
	m_Connected(false),
	m_SocketHandle(-1),
	gotdata(false),
	xCal(0.0f),
	yCal(0.0f)
{
	m_CommTimer = new Timer();
}

TargetingSystemClient::~TargetingSystemClient()
{

}
bool TargetingSystemClient::Connect(const char * server,unsigned short port)
{
	int status;
	sockaddr_in server_addr;
	//hostent *server_hostent;
	int addr_size;

	// int the select system
//	selectInit(2048);

	m_SocketHandle = socket(AF_INET,SOCK_STREAM,0);
	if (m_SocketHandle == -1)
	{
		printf("ERROR: Could not create socket!\n");
		return false;
	}

	addr_size = sizeof(server_addr);
	memset(&server_addr,0,sizeof(server_addr));

	server_addr.sin_family = AF_INET;
	//server_addr.sin_len = sizeof(server_addr);
	server_addr.sin_port = htons(port);

	server_addr.sin_addr.s_addr = inet_addr(server);
	/*reenable this
	if (server_addr.sin_addr.s_addr == (unsigned int)ERROR)
	{
		//hostent = getho(server);
		if (server_addr.sin_addr.s_addr == (unsigned int)ERROR)
		{
			printf("ERROR: could not resolve server address: %s\n",server);
			return false;
		}
	}
*/
	status = connect(m_SocketHandle,(struct sockaddr*)&server_addr,sizeof(server_addr));
	if (status == -1)
	{
		printf("ERROR: connect failed\n");
		close(m_SocketHandle);
		return false;
	}

	m_Connected = true;

	printf("Jetson Connected! server: %s port: %d\n",server,port);
	Send("0\r\n",4);
	Send("5\r\n",3);

	m_CommTimer->Reset();
	m_CommTimer->Start();
	return true;
}
void TargetingSystemClient::Shutdown_Jetson()
{
	Send("q\r\n",3);
}
void TargetingSystemClient::StartCalibrate()
{
	Send("4\r\n",3);
}
bool TargetingSystemClient::Update()
{
	if (m_Connected == false)
	{
		return false;
	}
	gotdata = false;
	fd_set read_set;
	FD_ZERO(&read_set);
	FD_SET(m_SocketHandle,&read_set);

	timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 0;

	int status = select(m_SocketHandle+1,&read_set,NULL,NULL,&timeout);
	if (status == -1)
	{
	//	printf("ERROR: select failed. socket: %d errno: %d",m_SocketHandle,errno);
		return false;
	}

	if (FD_ISSET(m_SocketHandle,&read_set))
	{
		char msg_buffer[4096];

		int bytes_in = recv(m_SocketHandle,msg_buffer,sizeof(msg_buffer),0);
		if (bytes_in == 0)
		{
			m_Connected = false;
			//printf("Disconnected!");
		}
		else
		{
			msg_buffer[512] = 0; // make sure msg_buffer can never overflow
			//printf("recv: %s\n",msg_buffer);
			Handle_Incoming_Data(msg_buffer,sizeof(msg_buffer));
		}
	}

	// Periodically make sure we ask for the current turret angle
	if (m_CommTimer->Get() > 0.0125f)
	{
		char buff[256];
		sprintf(buff,"2 %f\n",m_TurretAngle);
		//Send(buff,strlen(buff)+1);
		Send("0\r\n",3);
		Send("5\r\n",3);
		m_CommTimer->Reset();
	}

	return gotdata;
}

void TargetingSystemClient::Handle_Incoming_Data(char * data,int size)
{
	//printf("Handle_Incoming_Data: %s \n",data);

	char *cmd = strtok(data,"\n");
	while(cmd !=NULL)
	{
		Handle_Command(cmd);
		cmd = strtok(NULL,"\n");
	}

	//printf("done.\n");
}
void TargetingSystemClient::Handle_Command(char*data)
{
//	printf("Handle_Command: %s\n",data);

	switch(data[0])
	{
		case '0':
			Handle_Target(data);
			break;
		case '4':
			//Handle_Calibration(data);
			break;
		case '5':
			Handle_CalibrationRefresh(data);
			break;
	}
}
void TargetingSystemClient::FlipEnable()
{
	Send("2 1\r\n",5);
}
void TargetingSystemClient::FlipDisable()
{
	Send("2 0\r\n",5);
}
void TargetingSystemClient::EqualizeEnable()
{
	Send("6 1\r\n",5);
}
void TargetingSystemClient::EqualizeDisable()
{
	Send("6 0\r\n",5);
}
void TargetingSystemClient::Handle_Target(char *data)
{
	sscanf(data,"0 %f %f %f",&m_TargetDistance,&m_TargetAngle,&m_TargetArea);
	gotdata = true;
	SmartDashboard::PutString("Track Data", data);
//	printf("Handle_Target: %f\n",m_TargetDistance);
}
void TargetingSystemClient::Handle_Calibration(char *data)
{
	sscanf(data,"4 %f %f",&xCal,&yCal);
	gotdata = true;
	//SmartDashboard::PutString("Calibration", data);
}
void TargetingSystemClient::Handle_CalibrationRefresh(char *data)
{
	sscanf(data,"5 %f %f",&xCal,&yCal);
	gotdata = true;
	SmartDashboard::PutString("Calibration", data);
}
bool TargetingSystemClient::Send(const char * data_out,int size)
{
	int status;

	if ((m_Connected == true) && (size > 0))
	{
//		printf("send: %s\n",data_out);
		status = send(m_SocketHandle,data_out,size,0);
		if (status == -1)
		{
			//printf("ERROR: send failed. status: %d  errno: %d",status,errno);
			return false;
		}
		return true;
	}

	return false;
}

/*
 * HRLogger.cpp
 *
 *  Created on: Jan 30, 2016
 *      Author: HighRollers
 */

#include "HRLogger.h"
#include<stdio.h>
static bool HRLogEnabled = true;

char Filename[256];
int Logindex = 0;
bool Isfms = false;
bool Isinit = false;

void HRLogger::Init(bool fms)
{
	char rootname[256];

	Isfms = fms;
	if(Isfms == true)
	{
		sprintf(rootname, "FMS_LOG_");
	}
	else
	{
		sprintf(rootname,"LOG_");
	}
	Logindex = 0;

	FILE *f = fopen(Filename, "w");
	if(f)
	{
		fclose(f);
		Isinit = true;
	}
	else
	{
		Isinit = false;
	}
}

void HRLogger::Log(const char * format,...)
{

	printf(format);
	if(HRLogEnabled && Isinit)
	{

	};
}


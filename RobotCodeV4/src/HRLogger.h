/*
 * HRLogger.h
 *
 *  Created on: Jan 30, 2016
 *      Author: HighRollers
 */

#ifndef SRC_HRLOGGER_H_
#define SRC_HRLOGGER_H_

class HRLogger
{
public:
	static void Init(bool fms);
	static void Log(const char * format,...);
};

#endif // HRLOGGER_H

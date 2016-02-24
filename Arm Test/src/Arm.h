/*
 * Arm.h
 *
 *  Created on: Feb 17, 2016
 *      Author: 987
 */

#ifndef SRC_ARM_H_
#define SRC_ARM_H_

class Arm {
public:
	Talon *Armlifter;
	Talon *ArmTurret;
	
	Arm();
	~Arm();
	void Update();
};

#endif /* SRC_ARM_H_ */

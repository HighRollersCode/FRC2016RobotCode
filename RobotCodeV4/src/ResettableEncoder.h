/*
 * ResettableEncoderClass.h
 *
 *  Created on: Mar 1, 2016
 *      Author: 987
 */

#ifndef SRC_RESETTABLEENCODER_H_
#define SRC_RESETTABLEENCODER_H_

#include <Encoder.h>


/*
   ResettableEncoderClass
   This adds the ability to reset an encoder to a specific desired value.  For the 2016 game, we need to start
   our arm in the vertical position so that has to be our 'zero'.  However, during the match, if we need to reset,
   we want to reset when the arm is in the down position since that is quickly repeatable.  This is implemented
   by storing the reset amount as an offset.  The offset amout is added whenever the 'get' functions are called.
   At startup, the offset is zero and the true encoder value always matches the value return.  However, once we
   do a reset to a specific number, we have to reset the inherited state to 'zero' and store our desired value
   as the offset which is always added to the arm.
*/

class ResettableEncoderClass: public Encoder
{
public:

	ResettableEncoderClass(uint32_t aChannel, uint32_t bChannel, bool reverseDirection = false,EncodingType encodingType = k4X):
		Encoder(aChannel,bChannel,reverseDirection,encodingType),
		ResetOffset(0)
	{
	}
	virtual ~ResettableEncoderClass() {}

	int32_t Get() const override { return Encoder::Get() + ResetOffset; }
	int32_t GetRaw() const { return Encoder::GetRaw() + ResetOffset; }
	void Reset() { Encoder::Reset(); ResetOffset = 0; }
	double PIDGet() { return (double)Get(); }

	void Reset_To_Value(int count) { Encoder::Reset(); ResetOffset = count; }

private:

	int ResetOffset;	// offset from the encoder's internal counter.

};

#endif /* SRC_RESETTABLEENCODER_H_ */

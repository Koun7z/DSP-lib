/*
** DSP_FIR_RT.C
**
**  Created on: Feb 18, 2025
**      Author: Piotr Woliński
*/

#include "SignalFiltering.h"

void DSP_FIR_RT_Init_f32(DSP_FIR_RT_Instance_f32* filter,
                         size_t filterOrder,
                         const float* filterCoeffs,
                         float* pastSamplesBuff)
{
	filter->Coeffs           = filterCoeffs;
	filter->Order            = filterOrder;
	filter->_pastSamplesBuff = pastSamplesBuff;
	for(size_t i = 0; i < filterOrder; i++)
	{
		pastSamplesBuff[i] = 0.0f;
	}
	filter->_buffPointer = 0;
}

float DSP_FIR_RT_Update_f32(DSP_FIR_RT_Instance_f32* filter, float input)
{
	float out = 0.0f;

	for(size_t i = 0; i < filter->Order; i++)
	{
		const size_t n = (filter->_buffPointer + i) % filter->Order;

		out += filter->_pastSamplesBuff[n] * filter->Coeffs[i];
	}

	out += filter->Coeffs[filter->Order] * input;

	filter->_pastSamplesBuff[filter->_buffPointer] = input;
	filter->_buffPointer                           = (filter->_buffPointer + 1) % filter->Order;

	return out;
}
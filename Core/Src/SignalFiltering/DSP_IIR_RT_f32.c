/*
 * DSP_IIR_RT.C
 *
 *  Created on: Feb 18, 2025
 *      Author: Piotr Woliński
 */

#include "SignalFiltering.h"

void DSP_IIR_RT_Init_f32(DSP_IIR_RT_Instance_f32* filter,
                         size_t filterOrder,
                         const float* filterCoeffsNumerator,
                         const float* filterCoeffsDenominator,
                         float* pastSamplesBuff,
                         float* pastSamplesBuffFiltered)
{
	filter->Order                    = filterOrder;
	filter->CoeffsNumerator          = filterCoeffsNumerator;
	filter->CoeffsDenominator        = filterCoeffsDenominator;
	filter->_pastSamplesBuff         = pastSamplesBuff;
	filter->_pastSamplesBuffFiltered = pastSamplesBuffFiltered;

	for(size_t i = 0; i < filterOrder; i++)
	{
		filter->_pastSamplesBuff[i]         = 0.0f;
		filter->_pastSamplesBuffFiltered[i] = 0.0f;
	}

	filter->_buffPointer = 0;
}

float DSP_IIR_RT_Update_f32(DSP_IIR_RT_Instance_f32* filter, const float input)
{
	float out = 0.0f;

	size_t buff_ptr = filter->_buffPointer;
	for(size_t i = 0; i < filter->Order; i++)
	{
		out += filter->_pastSamplesBuff[buff_ptr] * filter->CoeffsNumerator[i];
		out -= filter->_pastSamplesBuffFiltered[buff_ptr] * filter->CoeffsDenominator[i];

		buff_ptr = buff_ptr + 1 >= filter->Order ? 0 : buff_ptr + 1;
	}

	out += input * filter->CoeffsNumerator[filter->Order];

	filter->_pastSamplesBuff[filter->_buffPointer]         = input;
	filter->_pastSamplesBuffFiltered[filter->_buffPointer] = out;
	filter->_buffPointer = filter->_buffPointer + 1 >= filter->Order ? 0 : filter->_buffPointer + 1;

	return out;
}

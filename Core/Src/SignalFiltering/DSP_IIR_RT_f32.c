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

float DSP_IIR_RT_Update_f32(DSP_IIR_RT_Instance_f32* filter, float input)
{
	float out = 0.0f;

	for(size_t i = 0; i < filter->Order; i++)
	{
		const size_t n = (filter->_buffPointer + i) % filter->Order;

		out += filter->_pastSamplesBuff[n] * filter->CoeffsNumerator[i];
		out -= filter->_pastSamplesBuffFiltered[n] * filter->CoeffsDenominator[i];
	}

	out += input * filter->CoeffsNumerator[filter->Order];

	filter->_pastSamplesBuff[filter->_buffPointer]         = input;
	filter->_pastSamplesBuffFiltered[filter->_buffPointer] = out;
	filter->_buffPointer                                   = (filter->_buffPointer + 1) % filter->Order;

	return out;
}

void DSP_ReverseArray_f32(float* arr, size_t size)
{
	for(size_t i = 0; i < size / 2; i++)
	{
		const float temp  = arr[i];
		arr[i]            = arr[size - i - 1];
		arr[size - i - 1] = temp;
	}
}
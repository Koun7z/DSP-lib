//
// Created by pwoli on 17.02.2025.
//

#include "SignalFiltering.h"

void DSP_FIR_RT_f32_Init(DSP_FIR_RT_f32_Instance* filter,
                         uint32_t filterOrder,
                         float* filterCoeffs,
                         float* pastSamplesBuff)
{
	filter->Coeffs           = filterCoeffs;
	filter->Order            = filterOrder;
	filter->_pastSamplesBuff = pastSamplesBuff;
	for(uint32_t i = 0; i < filterOrder; i++)
	{
		pastSamplesBuff[i] = 0.0f;
	}
	filter->_buffPointer = 0;
}

void DSP_FIR_RT_f64_Init(DSP_FIR_RT_f64_Instance* filter,
                         uint32_t filterOrder,
                         double* filterCoeffs,
                         double* pastSamplesBuff)
{
	filter->Coeffs           = filterCoeffs;
	filter->Order            = filterOrder;
	filter->_pastSamplesBuff = pastSamplesBuff;
	for(uint32_t i = 0; i < filterOrder; i++)
	{
		pastSamplesBuff[i] = 0.0f;
	}
	filter->_buffPointer = 0;
}

float DSP_FIR_RT_f32_Update(DSP_FIR_RT_f32_Instance* filter, float input)
{
	float out = 0.0f;

	for(uint32_t i = 0; i < filter->Order; i++)
	{
		const uint32_t n = (filter->_buffPointer + i) % filter->Order;

		out += filter->_pastSamplesBuff[n] * filter->Coeffs[i];
	}

	out += filter->Coeffs[filter->Order] * input;

	filter->_pastSamplesBuff[filter->_buffPointer] = input;
	filter->_buffPointer                           = (filter->_buffPointer + 1) % filter->Order;

	return out;
}

double DSP_FIR_RT_f64_Update(DSP_FIR_RT_f64_Instance* filter, double input)
{
	double out = 0.0;

	for(uint32_t i = 0; i < filter->Order; i++)
	{
		const uint32_t n = (filter->_buffPointer + i) % filter->Order;

		out += filter->_pastSamplesBuff[n] * filter->Coeffs[i];
	}

	out += filter->Coeffs[filter->Order] * input;

	filter->_pastSamplesBuff[filter->_buffPointer] = input;
	filter->_buffPointer                           = (filter->_buffPointer + 1) % filter->Order;

	return out;
}

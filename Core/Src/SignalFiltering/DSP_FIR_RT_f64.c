//
// Created by pwoli on 23.03.2025.
//

#include "SignalFiltering.h"

void DSP_FIR_RT_Init_f64(DSP_FIR_RT_Instance_f64* filter,
						 size_t filterOrder,
						 const double* filterCoeffs,
						 double* pastSamplesBuff)
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

double DSP_FIR_RT_Update_f64(DSP_FIR_RT_Instance_f64* filter, double input)
{
	double out = 0.0;

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
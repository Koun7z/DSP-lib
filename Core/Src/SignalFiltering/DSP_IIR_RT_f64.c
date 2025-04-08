//
// Created by pwoli on 23.03.2025.
//

#include "SignalFiltering.h"

void DSP_IIR_RT_Init_f64(DSP_IIR_RT_Instance_f64* filter,
						 size_t filterOrder,
						 const double* filterCoeffsNumerator,
						 const double* filterCoeffsDenominator,
						 double* pastSamplesBuff,
						 double* pastSamplesBuffFiltered)
{
	filter->Order                    = filterOrder;
	filter->CoeffsNumerator          = filterCoeffsNumerator;
	filter->CoeffsDenominator        = filterCoeffsDenominator;
	filter->_pastSamplesBuff         = pastSamplesBuff;
	filter->_pastSamplesBuffFiltered = pastSamplesBuffFiltered;

	for(size_t i = 0; i < filterOrder; i++)
	{
		filter->_pastSamplesBuff[i]         = 0.0;
		filter->_pastSamplesBuffFiltered[i] = 0.0;
	}

	filter->_buffPointer = 0;
}

double DSP_IIR_RT_Update_f64(DSP_IIR_RT_Instance_f64* filter, double input)
{
	double out = 0.0;

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

void DSP_ReverseArray_f64(double* arr, size_t size)
{
	for(size_t i = 0; i < size / 2; i++)
	{
		const double temp = arr[i];
		arr[i]            = arr[size - i - 1];
		arr[size - i - 1] = temp;
	}
}
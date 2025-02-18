//
// Created by pwoli on 17.02.2025.
//

#include "SignalFiltering.h"


void DSP_IIR_RT_f32_Init(DSP_IIR_RT_f32_Instance* filter,
                         size_t filterOrder,
                         float* filterCoeffsNumerator,
                         float* filterCoeffsDenominator,
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

void DSP_IIR_RT_f64_Init(DSP_IIR_RT_f64_Instance* filter,
                         size_t filterOrder,
                         double* filterCoeffsNumerator,
                         double* filterCoeffsDenominator,
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

float DSP_IIR_RT_f32_Update(DSP_IIR_RT_f32_Instance* filter, float input)
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

double DSP_IIR_RT_f64_Update(DSP_IIR_RT_f64_Instance* filter, double input)
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

void DSP_ReverseArrayf32(float* arr, size_t size)
{
	for(size_t i = 0; i < size / 2; i++)
	{
		const float temp  = arr[i];
		arr[i]            = arr[size - i - 1];
		arr[size - i - 1] = temp;
	}
}

void DSP_ReverseArrayf64(double* arr, size_t size)
{
	for(size_t i = 0; i < size / 2; i++)
	{
		const double temp = arr[i];
		arr[i]            = arr[size - i - 1];
		arr[size - i - 1] = temp;
	}
}

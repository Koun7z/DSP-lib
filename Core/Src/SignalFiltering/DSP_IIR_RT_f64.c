//
// Created by pwoli on 23.03.2025.
//

#include "DSP_SignalFiltering.h"

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

double DSP_IIR_RT_Update_f64(DSP_IIR_RT_Instance_f64* filter, const double input)
{
    double out = 0.0f;

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

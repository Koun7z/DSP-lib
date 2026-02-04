/*
** DSP_FIR_RT.C
**
**  Created on: Feb 18, 2025
**      Author: Piotr Woliński
*/

#include "DSP_SignalFiltering.h"

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

float DSP_FIR_RT_Update_f32(DSP_FIR_RT_Instance_f32* __restrict filter, const float input)
{
    float out = 0.0f;

    size_t buff_ptr = filter->_buffPointer;
    for(size_t i = 0; i < filter->Order; i++)
    {
        out     += filter->_pastSamplesBuff[buff_ptr] * filter->Coeffs[i];
        buff_ptr = buff_ptr + 1 >= filter->Order ? 0 : buff_ptr + 1;
    }

    out += filter->Coeffs[filter->Order] * input;

    filter->_pastSamplesBuff[filter->_buffPointer] = input;
    filter->_buffPointer = filter->_buffPointer + 1 >= filter->Order ? 0 : filter->_buffPointer + 1;

    return out;
}

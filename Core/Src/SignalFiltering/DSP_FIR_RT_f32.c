/*
** DSP_FIR_RT.C
**
**  Created on: Feb 18, 2025
**      Author: Piotr Woliński
*/

#include "DSP_FIR.h"

#include "DSP_Utils.h"
#include "DSP_Assert.h"

void DSP_FIR_RT_Init_f32(DSP_FIR_RT_Instance_f32* filter, size_t order, const float* b, float* buff)
{
    DSP_ASSERT(filter && b && buff);

    filter->b     = b;
    filter->Order = order;
    filter->_buff = buff;
    for(size_t i = 0; i < order; i++)
    {
        buff[i] = 0.0f;
    }
    filter->_buffIdx = 0;
}

float DSP_FIR_RT_Update_f32(DSP_FIR_RT_Instance_f32* filter, const float x)
{
    DSP_ASSERT(filter);
    DSP_ASSERT_MSG(filter->b && filter->_buff, "Uninitialized filter instance?");

    float y = 0.0f;

    size_t idx = filter->_buffIdx;
    for(size_t i = 0; i < filter->Order; i++)
    {
        y  += filter->_buff[idx] * filter->b[i];
        idx = DSP_IncrementIndex(idx, filter->Order);
    }

    y += filter->b[filter->Order] * x;

    filter->_buff[filter->_buffIdx] = x;
    filter->_buffIdx                = DSP_IncrementIndex(filter->_buffIdx, filter->Order);

    return y;
}

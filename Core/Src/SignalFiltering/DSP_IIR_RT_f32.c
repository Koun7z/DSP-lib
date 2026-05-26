/*
 * DSP_IIR_RT.C
 *
 *  Created on: Feb 18, 2025
 *      Author: Piotr Woliński
 */

#include "DSP_IIR.h"
#include "DSP_Assert.h"

void DSP_IIR_RT_Init_f32(DSP_IIR_RT_Instance_f32* filter, size_t order, const float* b, const float* a, float* buff)
{
    DSP_ASSERT(filter && b && a && buff);

    filter->Order = order;
    filter->b     = b;
    filter->a     = a;
    filter->_buff = buff;

    for(size_t i = 0; i < filter->Order; i++)
    {
        filter->_buff[i] = 0.0f;
    }
}

float DSP_IIR_RT_Update_f32(DSP_IIR_RT_Instance_f32* filter, float x)
{
    DSP_ASSERT(filter);
    DSP_ASSERT_MSG(filter->a && filter->b && filter->_buff, "Uninitialized filter instance?");

    const float* b = filter->b;
    const float* a = filter->a;
    float* d       = filter->_buff;
    const size_t N = filter->Order;

    float y = b[0] * x + d[0];

    for(size_t i = 0; i < N - 1; i++)
    {
        d[i] = b[i + 1] * x + d[i + 1] - a[i + 1] * y;
    }
    d[N - 1] = b[N] * x - a[N] * y;

    return y;
}

//
// Created by pwoli on 23.03.2025.
//

#include "DSP_FIR.h"

#include "DSP_Utils.h"
#include "DSP_Assert.h"

void DSP_FIR_RT_Init_f64(DSP_FIR_RT_Instance_f64* filter, size_t order, const double* b, double* buff)
{
    DSP_ASSERT(filter && b && buff);

    filter->b     = b;
    filter->Order = order;
    filter->_buff = buff;
    for(size_t i = 0; i < order; i++)
    {
        buff[i] = 0.0;
    }
    filter->_buffIdx = 0;
}

double DSP_FIR_RT_Update_f64(DSP_FIR_RT_Instance_f64* filter, const double x)
{
    DSP_ASSERT(filter);
    DSP_ASSERT_MSG(filter->b && filter->_buff, "Uninitialized filter instance?");

    double y = 0.0;

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

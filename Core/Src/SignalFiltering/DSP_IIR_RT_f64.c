//
// Created by pwoli on 23.03.2025.
//

#include "DSP_IIR.h"
#include "DSP_Assert.h"

void DSP_IIR_RT_Init_f64(DSP_IIR_RT_Instance_f64* filter, size_t order, const double* b, const double* a, double* buff)
{
    DSP_ASSERT(filter && b && a && buff);

    filter->Order = order;
    filter->b     = b;
    filter->a     = a;
    filter->_buff = buff;

    for(size_t i = 0; i < order; i++)
    {
        filter->_buff[i] = 0.0;
    }
}

double DSP_IIR_RT_Update_f64(DSP_IIR_RT_Instance_f64* filter, double x)
{
    DSP_ASSERT(filter);
    DSP_ASSERT_MSG(filter->a && filter->b && filter->_buff, "Uninitialized filter instance?");

    const double* b = filter->b;
    const double* a = filter->a;
    double* d       = filter->_buff;
    const size_t N  = filter->Order;

    double y = b[0] * x + d[0];

    for(size_t i = 0; i < N - 1; i++)
    {
        d[i] = b[i + 1] * x + d[i + 1] - a[i + 1] * y;
    }
    d[N - 1] = b[N] * x - a[N] * y;

    return y;
}

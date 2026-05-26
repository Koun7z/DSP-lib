#ifndef DSP_FIR_H__
#define DSP_FIR_H__

#include <stddef.h>

/**
 * @brief  Finite impulse response real time filter instance
 *		   storing z-transform coefficients and past samples circular buffer
 *		   as single precision floating point values
 */
typedef struct
{
    size_t Order;
    const float* b;
    float* _buff;
    size_t _buffIdx;
} DSP_FIR_RT_Instance_f32;

/**
 * @brief  Finite impulse response real time filter instance
 *		   storing z-transform coefficients and past samples circular buffer
 *		   as double precision floating point values
 */
typedef struct
{
    size_t Order;
    const double* b;
    double* _buff;
    size_t _buffIdx;
} DSP_FIR_RT_Instance_f64;

/**
 * @brief		  Initializes FIR filter instance structure.
 *				  - Each filtered signal needs a unique filter instance.
 * @param[out]   *filter           Instance to initialize.
 * @param[in]     order            Order of the filter (number of taps).
 * @param[in]    *b                Array of filter z-transform coefficients in REVERSE order.
 *							       - Can be shared between different filter instances.
 *							       - Array size = order + 1.
 * @param[inout] *pastSamplesBuff  Buffer to store n past samples needed for filtering.
 *								   - Must be unique to this filter instance.
 *								   - Buffer size = order.
 */
void DSP_FIR_RT_Init_f32(DSP_FIR_RT_Instance_f32* filter, size_t order, const float* b, float* buff);

/**
 * @brief		  Initializes FIR filter instance structure.
 *				  - Each filtered signal needs a unique filter instance.
 * @param[out]   *filter           Instance to initialize.
 * @param[in]     order            Order of the filter (number of taps).
 * @param[in]    *b                Array of filter z-transform coefficients in REVERSE order.
 *							       - Can be shared between different filter instances.
 *							       - Array size = order + 1.
 * @param[inout] *pastSamplesBuff  Buffer to store n past samples needed for filtering.
 *								   - Must be unique to this filter instance.
 *								   - Buffer size = order.
 */
void DSP_FIR_RT_Init_f64(DSP_FIR_RT_Instance_f64* filter, size_t order, const double* b, double* buff);

/**
 * @brief		  Filters signal using a FIR filter, one sample at a time
 * @param[inout] *filter  Initialized filter instance
 * @param[in]     input	  Current raw signal sample
 * @return		  float   Filtered sample
 */
float DSP_FIR_RT_Update_f32(DSP_FIR_RT_Instance_f32* filter, float x);

/**
 * @brief		  Filters signal using a FIR filter, one sample at a time
 * @param[inout] *filter  Initialized filter instance
 * @param[in]	  input	  Current raw signal sample
 * @return        float	  Filtered sample
 */
double DSP_FIR_RT_Update_f64(DSP_FIR_RT_Instance_f64* filter, double x);

#endif  // DSP_FIR_H__
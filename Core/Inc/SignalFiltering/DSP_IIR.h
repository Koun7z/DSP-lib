#ifndef DSP_IIR_H__
#define DSP_IIR_H__

#include <stddef.h>

/**
 * @brief  Infinite impulse response real time filter instance
 *		   storing z-transform coefficients and past samples circular buffer
 *		   as single precision floating point values
 */
typedef struct
{
    size_t Order;
    const float* b;
    const float* a;
    float* _buff;
} DSP_IIR_RT_Instance_f32;

/**
 * @brief  Infinite impulse response real time filter instance
 *		   storing z-transform coefficients and past samples circular buffer
 *		   as double precision floating point values
 */
typedef struct
{
    size_t Order;
    const double* b;
    const double* a;
    double* _buff;
} DSP_IIR_RT_Instance_f64;


/**
 * @brief		Initializes IIR filter instance structure.
 *				- Each filtered signal needs a unique filter instance.
 * @param[out]  *filter			  	       Filter instance to initialize.
 * @param[in]    order			           Order of the filter (number of taps).
 * @param[in]   *b                   	   Array of filter z-transform numerator coefficients in forward order.
 *										   - Can be shared between different filter instances.
 *										   - Array size = filterOrder + 1.
 * @param[in]	*a                         Array of filter z-transform denominator coefficients in forward order.
 *										   - Can be shared between different filter instances.
 *										   - Array size = filterOrder.
 * @param[inout] *buff                     Buffer to store n past input samples needed for filtering.
 *										   - Must be unique to this filter instance.
 *										   - Buffer size = order.
 */
void DSP_IIR_RT_Init_f32(DSP_IIR_RT_Instance_f32* filter, size_t order, const float* b, const float* a, float* buff);

/**
 * @brief		 Initializes IIR filter instance structure.
 *				 - Each filtered signal needs a unique filter instance.
 * @param[out]   *filter  Filter instance to initialize.
 * @param[in]     order   Order of the filter (number of taps).
 * @param[in]    *b	      Array of filter z-transform numerator coefficients in forward order.
 *						  - Can be shared between different filter instances.
 *						  - Array size = order + 1.
 * @param[in]	 *a       Array of filter z-transform denominator coefficients in forward order.
 *						  - Can be shared between different filter instances.
 *						  - Array size = order + 1.
 * @param[inout] *buff    Buffer to store n past input samples needed for filtering.
 *						  - Must be unique to this filter instance.
 *						  - Buffer size = order.
 */
void DSP_IIR_RT_Init_f64(DSP_IIR_RT_Instance_f64* filter, size_t order, const double* b, const double* a, double* buff);

/**
 * @brief		  Performs a single update step of the discreet filter using transposed direct form II structure.
 * @param[inout] *filter  Initialized filter instance
 * @param[in]	  x       Input sample
 * @return        float	  Filtered sample
 */
float DSP_IIR_RT_Update_f32(DSP_IIR_RT_Instance_f32* filter, float x);

/**
 * @brief		  Performs a single update step of the discreet filter using transposed direct form II structure.
 * @param[inout] *filter  Initialized filter instance
 * @param[in]	  x       Input sample
 * @return        double  Filtered sample
 */
double DSP_IIR_RT_Update_f64(DSP_IIR_RT_Instance_f64* filter, double x);

#endif  // DSP_IIR_H__
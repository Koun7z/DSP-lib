/*
 * SignalFiltering.h
 *
 *  Created on: Feb 18, 2025
 *      Author: Piotr Woliński
 */

#ifndef SIGNALFILTERING_H
#define SIGNALFILTERING_H

#include <stddef.h>

/**
 * @brief  Finite impulse response real time filter instance
 *		   storing z-transform coefficients and past samples circular buffer
 *		   as single precision floating point values
 */
typedef struct
{
    size_t Order;
    const float* Coeffs;
    float* _pastSamplesBuff;
    size_t _buffPointer;
} DSP_FIR_RT_Instance_f32;

/**
 * @brief  Finite impulse response real time filter instance
 *		   storing z-transform coefficients and past samples circular buffer
 *		   as double precision floating point values
 */
typedef struct
{
    size_t Order;
    const double* Coeffs;
    double* _pastSamplesBuff;
    size_t _buffPointer;
} DSP_FIR_RT_Instance_f64;

/**
 * @brief  Infinite impulse response real time filter instance
 *		   storing z-transform coefficients and past samples circular buffer
 *		   as single precision floating point values
 */
typedef struct
{
    size_t Order;
    const float* CoeffsNumerator;
    const float* CoeffsDenominator;
    float* _pastSamplesBuff;
    float* _pastSamplesBuffFiltered;
    size_t _buffPointer;
} DSP_IIR_RT_Instance_f32;

/**
 * @brief  Infinite impulse response real time filter instance
 *		   storing z-transform coefficients and past samples circular buffer
 *		   as double precision floating point values
 */
typedef struct
{
    size_t Order;
    const double* CoeffsNumerator;
    const double* CoeffsDenominator;
    double* _pastSamplesBuff;
    double* _pastSamplesBuffFiltered;
    size_t _buffPointer;
} DSP_IIR_RT_Instance_f64;

/**
 * @brief		  Initializes FIR filter instance structure.
 *				  - Each filtered signal needs a unique filter instance.
 * @param[out]   *filter           Instance to initialize.
 * @param[in]     filterOrder      Order of the filter (number of taps).
 * @param[in]    *filterCoeffs     Array of filter z-transform coefficients in REVERSE order.
 *							       - Can be shared between different filter instances.
 *							       - Array size = filterOrder + 1.
 * @param[inout] *pastSamplesBuff  Buffer to store n past samples needed for filtering.
 *								   - Must be unique to this filter instance.
 *								   - Buffer size = filterOrder.
 */
void DSP_FIR_RT_Init_f32(DSP_FIR_RT_Instance_f32* filter,
                         size_t filterOrder,
                         const float* filterCoeffs,
                         float* pastSamplesBuff);

/**
 * @brief		  Initializes FIR filter instance structure.
 *				  - Each filtered signal needs a unique filter instance.
 * @param[out]   *filter           Instance to initialize.
 * @param[in]     filterOrder      Order of the filter (number of taps).
 * @param[in]    *filterCoeffs     Array of filter z-transform coefficients in REVERSE order.
 *							       - Can be shared between different filter instances.
 *							       - Array size = filterOrder + 1.
 * @param[inout] *pastSamplesBuff  Buffer to store n past samples needed for filtering.
 *								   - Must be unique to this filter instance.
 *								   - Buffer size = filterOrder.
 */
void DSP_FIR_RT_Init_f64(DSP_FIR_RT_Instance_f64* filter,
                         size_t filterOrder,
                         const double* filterCoeffs,
                         double* pastSamplesBuff);

/**
 * @brief		  Filters signal using a FIR filter, one sample at a time
 * @param[inout] *filter  Initialized filter instance
 * @param[in]     input	  Current raw signal sample
 * @return		  float   Filtered sample
 */
float DSP_FIR_RT_Update_f32(DSP_FIR_RT_Instance_f32* filter, float input);

/**
 * @brief		  Filters signal using a FIR filter, one sample at a time
 * @param[inout] *filter  Initialized filter instance
 * @param[in]	  input	  Current raw signal sample
 * @return        float	  Filtered sample
 */
double DSP_FIR_RT_Update_f64(DSP_FIR_RT_Instance_f64* filter, double input);


/**
 * @brief		 Initializes IIR filter instance structure.
 *				 - Each filtered signal needs a unique filter instance.
 * @param[out]   *filter				   Filter instance to initialize.
 * @param[in]     filterOrder			   Order of the filter (number of taps).
 * @param[in]    *filterCoeffsNumerator	   Array of filter z-transform numerator coefficients in REVERSE order.
 *										   - Can be shared between different filter instances.
 *										   - Array size = filterOrder + 1.
 * @param[in]	*filterCoeffsDenominator  Array of filter z-transform numerator coefficients in REVERSE order.
 *										   - Can be shared between different filter instances.
 *										   - Array size = filterOrder.
 * @param[inout] *pastSamplesBuff          Buffer to store n past input samples needed for filtering.
 *										   - Must be unique to this filter instance.
 *										   - Buffer size = filterOrder.
 * @param[inout] *pastSamplesBuffFiltered  Buffer to store n past output samples needed for filtering.
 *										   - Must be unique to this filter instance.
 *										   - Buffer size = filterOrder.
 */
void DSP_IIR_RT_Init_f32(DSP_IIR_RT_Instance_f32* filter,
                         size_t filterOrder,
                         const float* filterCoeffsNumerator,
                         const float* filterCoeffsDenominator,
                         float* pastSamplesBuff,
                         float* pastSamplesBuffFiltered);

/**
 * @brief		 Initializes IIR filter instance structure.
 *				 - Each filtered signal needs a unique filter instance.
 * @param[out]   *filter				   Filter instance to initialize.
 * @param[in]     filterOrder			   Order of the filter (number of taps).
 * @param[in]    *filterCoeffsNumerator	   Array of filter z-transform numerator coefficients in REVERSE order.
 *										   - Can be shared between different filter instances.
 *										   - Array size = filterOrder + 1.
 * @param[in]	*filterCoeffsDenominator  Array of filter z-transform numerator coefficients in REVERSE order.
 *										   - Can be shared between different filter instances.
 *										   - Array size = filterOrder.
 * @param[inout] *pastSamplesBuff          Buffer to store n past input samples needed for filtering.
 *										   - Must be unique to this filter instance.
 *										   - Buffer size = filterOrder.
 * @param[inout] *pastSamplesBuffFiltered  Buffer to store n past output samples needed for filtering.
 *										   - Must be unique to this filter instance.
 *										   - Buffer size = filterOrder.
 */
void DSP_IIR_RT_Init_f64(DSP_IIR_RT_Instance_f64* filter,
                         size_t filterOrder,
                         const double* filterCoeffsNumerator,
                         const double* filterCoeffsDenominator,
                         double* pastSamplesBuff,
                         double* pastSamplesBuffFiltered);

/**
 * @brief		  Filters signal using a IIR filter, one sample at a time
 * @param[inout] *filter  Initialized filter instance
 * @param[in]	  input   Current raw signal sample
 * @return        float	  Filtered sample
 */
float DSP_IIR_RT_Update_f32(DSP_IIR_RT_Instance_f32* filter, float input);

/**
 * @brief		  Filters signal using a IIR filter, one sample at a time
 * @param[inout] *filter  Initialized filter instance
 * @param[in]	  input   Current raw signal sample
 * @return        float	  Filtered sample
 */
double DSP_IIR_RT_Update_f64(DSP_IIR_RT_Instance_f64* filter, double input);

#endif  // SIGNALFILTERING_H

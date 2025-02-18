//
// Created by pwoli on 17.02.2025.
//

#ifndef SIGNALFILTERING_H
#define SIGNALFILTERING_H

#include <stddef.h>

// Finite impulse response filter instance working on float data type
typedef struct
{
	size_t Order;
	float* Coeffs;
	float* _pastSamplesBuff;
	size_t _buffPointer;
} DSP_FIR_RT_f32_Instance;

// Finite impulse response filter instance working on double data type
typedef struct
{
	size_t Order;
	double* Coeffs;
	double* _pastSamplesBuff;
	size_t _buffPointer;
} DSP_FIR_RT_f64_Instance;

// Infinite impulse response filter instance working on float data type
typedef struct
{
	size_t Order;
	float* CoeffsNumerator;
	float* CoeffsDenominator;
	float* _pastSamplesBuff;
	float* _pastSamplesBuffFiltered;
	size_t _buffPointer;
} DSP_IIR_RT_f32_Instance;

// Infinite impulse response filter instance working on double data type
typedef struct
{
	size_t Order;
	double* CoeffsNumerator;
	double* CoeffsDenominator;
	double* _pastSamplesBuff;
	double* _pastSamplesBuffFiltered;
	size_t _buffPointer;
} DSP_IIR_RT_f64_Instance;

/**
 * @brief					       Initializes FIR filter instance structure.
 *							       Each filtered signal needs a unique filter instance.
 * @param *filter[out]	           Instance to initialize.
 * @param filterOrder[in]          Order of the filter (number of taps).
 * @param *filterCoeffs[in]        Array of filter z-transform coefficients in REVERSE order.
 *							       Can be shared between different filter instances.
 *							       Array size = filterOrder + 1.
 * @param *pastSamplesBuff[inout]  Buffer to store n past samples needed for filtering.
 *								   Must be unique to this filter instance.
 *							       Buffer size = filterOrder.
 */
void DSP_FIR_RT_f32_Init(DSP_FIR_RT_f32_Instance* filter,
                         size_t filterOrder,
                         float* filterCoeffs,
                         float* pastSamplesBuff);

/**
 * @brief					       Initializes FIR filter instance structure.
 *							       Each filtered signal needs a unique filter instance.
 * @param *filter[out]	           Instance to initialize.
 * @param filterOrder[in]          Order of the filter (number of taps).
 * @param *filterCoeffs[in]        Array of filter z-transform coefficients in REVERSE order.
 *							       Can be shared between different filter instances.
 *							       Array size = filterOrder + 1.
 * @param *pastSamplesBuff[inout]  Buffer to store n past samples needed for filtering.
 *								   Must be unique to this filter instance.
 *							       Buffer size = filterOrder.
 */
void DSP_FIR_RT_f64_Init(DSP_FIR_RT_f64_Instance* filter,
                         size_t filterOrder,
                         double* filterCoeffs,
                         double* pastSamplesBuff);

/**
 * @brief				  Filters signal using a FIR filter, one sample at a time
 * @param *filter[inout]  Initialized filter instance
 * @param input[in]		  Current raw signal sample
 * @return float		  Filtered sample
 */
float DSP_FIR_RT_f32_Update(DSP_FIR_RT_f32_Instance* filter, float input);

/**
 * @brief				  Filters signal using a FIR filter, one sample at a time
 * @param *filter[inout]  Initialized filter instance
 * @param input[in]		  Current raw signal sample
 * @return float		  Filtered sample
 */
double DSP_FIR_RT_f64_Update(DSP_FIR_RT_f64_Instance* filter, double input);


/**
 * @brief							       Initializes IIR filter instance structure.
 *									       Each filtered signal needs a unique filter instance.
 * @param filter[out]				       Instance to initialize.
 * @param filterOrder[in]			       Order of the filter (number of taps).
 * @param *filterCoeffsNumerator[in]	   Array of filter z-transform numerator coefficients in REVERSE order.
 *									       Can be shared between different filter instances.
 *							               Array size = filterOrder + 1.
 * @param *filterCoeffsDenominator[in]     Array of filter z-transform numerator coefficients in REVERSE order.
 *									       Can be shared between different filter instances.
 *							               Array size = filterOrder.
 * @param *pastSamplesBuff[inout]          Buffer to store n past input samples needed for filtering.
 *										   Must be unique to this filter instance.
 *								           Buffer size = filterOrder.
 * @param *pastSamplesBuffFiltered[inout]  Buffer to store n past output samples needed for filtering.
 *										   Must be unique to this filter instance.
 *								           Buffer size = filterOrder.
 */
void DSP_IIR_RT_f32_Init(DSP_IIR_RT_f32_Instance* filter,
                         size_t filterOrder,
                         float* filterCoeffsNumerator,
                         float* filterCoeffsDenominator,
                         float* pastSamplesBuff,
                         float* pastSamplesBuffFiltered);
/**
 * @brief							       Initializes IIR filter instance structure.
 *									       Each filtered signal needs a unique filter instance.
 * @param filter[out]				       Instance to initialize.
 * @param filterOrder[in]			       Order of the filter (number of taps).
 * @param *filterCoeffsNumerator[in]	   Array of filter z-transform numerator coefficients in REVERSE order.
 *									       Can be shared between different filter instances.
 *							               Array size = filterOrder + 1.
 * @param *filterCoeffsDenominator[in]     Array of filter z-transform numerator coefficients in REVERSE order.
 *									       Can be shared between different filter instances.
 *							               Array size = filterOrder.
 * @param *pastSamplesBuff[inout]          Buffer to store n past input samples needed for filtering.
 *										   Must be unique to this filter instance.
 *								           Buffer size = filterOrder.
 * @param *pastSamplesBuffFiltered[inout]  Buffer to store n past output samples needed for filtering.
 *										   Must be unique to this filter instance.
 *								           Buffer size = filterOrder.
 */
void DSP_IIR_RT_f64_Init(DSP_IIR_RT_f64_Instance* filter,
                         size_t filterOrder,
                         double* filterCoeffsNumerator,
                         double* filterCoeffsDenominator,
                         double* pastSamplesBuff,
                         double* pastSamplesBuffFiltered);

/**
 * @brief				  Filters signal using a IIR filter, one sample at a time
 * @param *filter[inout]  Initialized filter instance
 * @param input[in]	 	  Current raw signal sample
 * @return float		  Filtered sample
 */
float DSP_IIR_RT_f32_Update(DSP_IIR_RT_f32_Instance* filter, float input);

/**
 * @brief				  Filters signal using a IIR filter, one sample at a time
 * @param *filter[inout]  Initialized filter instance
 * @param input[in]		  Current raw signal sample
 * @return float		  Filtered sample
 */
double DSP_IIR_RT_f64_Update(DSP_IIR_RT_f64_Instance* filter, double input);

/**
 * brief			   Reverses elements inside float array in place
 * @param *arr[inout]  Array to be reversed
 * @param size[in]     Array size
 */
void DSP_ReverseArrayf32(float* arr, size_t size);

/**
 * brief			   Reverses elements inside double array in place
 * @param *arr[inout]  Array to be reversed
 * @param size[in]     Array size
 */
void DSP_ReverseArrayf64(double* arr, size_t size);

#endif  // SIGNALFILTERING_H

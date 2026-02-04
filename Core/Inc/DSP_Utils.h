//
// Created by pwoli on 04.05.2025.
//

#ifndef DSP_UTILS_H
#define DSP_UTILS_H
#include <math.h>

static inline float DSP_Clamp_f32(const float x, const float min, const float max)
{
    const float t = x < min ? min : x;
    return t > max ? max : t;
}

static inline double DSP_Clamp_f64(const double x, const double min, const double max)
{
    const double t = x < min ? min : x;
    return t > max ? max : t;
}

static inline float DSP_DeadZone_f32(const float x, const float deadZone)
{
    return fabsf(x) < deadZone ? 0.0f : x;
}

static inline double DSP_DeadZone_f64(const double x, const double deadZone)
{
    return fabs(x) < deadZone ? 0.0 : x;
}

/**
 * brief		  Reverses elements inside float array (in place)
 * @param[inout] *arr   Array to be reversed
 * @param[in]     size  Array size
 */
static inline void DSP_ReverseArray_f32(float* arr, const size_t size)
{
    for(size_t i = 0; i < size / 2; i++)
    {
        const float temp  = arr[i];
        arr[i]            = arr[size - i - 1];
        arr[size - i - 1] = temp;
    }
}

/**
 * brief		  Reverses elements inside double array (in place)
 * @param[inout] *arr   Array to be reversed
 * @param[in]     size  Array size
 */
static inline void DSP_ReverseArray_f64(double* arr, const size_t size)
{
    for(size_t i = 0; i < size / 2; i++)
    {
        const double temp = arr[i];
        arr[i]            = arr[size - i - 1];
        arr[size - i - 1] = temp;
    }
}

#endif  // DSP_UTILS_H

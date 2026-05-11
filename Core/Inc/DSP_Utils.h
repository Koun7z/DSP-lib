//
// Created by pwoli on 04.05.2025.
//

#ifndef DSP_UTILS_H
#define DSP_UTILS_H

#include "DSP_Quaternion.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

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

/*
** ToString functions for quick and dirty debugging. Remember to free the returned string after use.
*/

#define DSP_QT_ToString(_q)                                                                              \
    _Generic((_q), DSP_Quaternion_f32: DSP_QT_ToString_f32, DSP_Quaternion_f64: DSP_QT_ToString_f64)(_q)

#define DSP_Vec3_ToString(_vec) _Generic((_vec), float*: DSP_Vec3_ToString_f32, double*: DSP_Vec3_ToString_f64)(_vec)

static inline char* DSP_QT_ToString_f32(DSP_Quaternion_f32 q)
{
    int len   = snprintf(NULL, 0, "[%.4f, %.4f, %.4f, %.4f]", q.r, q.i, q.j, q.k);
    char* buf = malloc(len + 1);
    snprintf(buf, len + 1, "[%.4f, %.4f, %.4f, %.4f]", q.r, q.i, q.j, q.k);
    return buf;
}

static inline char* DSP_QT_ToString_f64(DSP_Quaternion_f64 q)
{
    int len   = snprintf(NULL, 0, "[%.8f, %.8f, %.8f, %.8f]", q.r, q.i, q.j, q.k);
    char* buf = malloc(len + 1);
    snprintf(buf, len + 1, "[%.8f, %.8f, %.8f, %.8f]", q.r, q.i, q.j, q.k);
    return buf;
}

static inline char* DSP_Vec3_ToString_f32(const float* vec)
{
    int len   = snprintf(NULL, 0, "[%.4f, %.4f, %.4f]", vec[0], vec[1], vec[2]);
    char* buf = malloc(len + 1);
    snprintf(buf, len + 1, "[%.4f, %.4f, %.4f]", vec[0], vec[1], vec[2]);
    return buf;
}

static inline char* DSP_Vec3_ToString_f64(const double* vec)
{
    int len   = snprintf(NULL, 0, "[%.8f, %.8f, %.8f]", vec[0], vec[1], vec[2]);
    char* buf = malloc(len + 1);
    snprintf(buf, len + 1, "[%.8f, %.8f, %.8f]", vec[0], vec[1], vec[2]);
    return buf;
}

#endif  // DSP_UTILS_H

#ifndef DSP_VECTOR_H__
#define DSP_VECTOR_H__

#include "DSP_Constants.h"

#include <math.h>
#include <stddef.h>

static inline float DSP_Vector_Norm_f32(const float* vec, size_t len)
{
    float sum = 0.0f;
    for(size_t i = 0; i < len; i++)
    {
        sum += vec[i] * vec[i];
    }
    return sqrtf(sum);
}

static inline float DSP_Vector_SquaredNorm_f32(const float* vec, size_t len)
{
    float sum = 0.0f;
    for(size_t i = 0; i < len; i++)
    {
        sum += vec[i] * vec[i];
    }
    return sum;
}

static inline float DSP_Vector_Normalize_f32(float* vec, size_t len)
{
    const float sq_norm = DSP_Vector_SquaredNorm_f32(vec, len);

    if(sq_norm < DSP_NORM_EPSILON_F32 * DSP_NORM_EPSILON_F32)
    {
        for(size_t i = 0; i < len; i++)
        {
            vec[i] = 0.0f;
        }

        return 0.0f;
    }

    const float norm     = sqrtf(sq_norm);
    const float inv_norm = 1.0f / norm;
    for(size_t i = 0; i < len; i++)
    {
        vec[i] = vec[i] * inv_norm;
    }

    return norm;
}

static inline void DSP_Vector_Scale_f32(float* vec, float s, size_t len)
{
    for(size_t i = 0; i < len; i++)
    {
        vec[i] *= s;
    }
}

static inline float DSP_Vector_Dot_f32(const float* a, const float* b, size_t len)
{
    float sum = 0.0f;
    for(size_t i = 0; i < len; i++)
    {
        sum += a[i] * b[i];
    }
    return sum;
}

static inline void DSP_Vec3_Cross_f32(float* __restrict result, const float* a, const float* b)
{
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

/*
** Double precision implementations
*/

static inline double DSP_Vector_Norm_f64(const double* vec, size_t len)
{
    double sum = 0.0;
    for(size_t i = 0; i < len; i++)
    {
        sum += vec[i] * vec[i];
    }
    return sqrt(sum);
}

static inline double DSP_Vector_SquaredNorm_f64(const double* vec, size_t len)
{
    double sum = 0.0;
    for(size_t i = 0; i < len; i++)
    {
        sum += vec[i] * vec[i];
    }
    return sum;
}

static inline double DSP_Vector_Normalize_f64(double* vec, size_t len)
{
    const double sq_norm = DSP_Vector_SquaredNorm_f64(vec, len);

    if(sq_norm < DSP_NORM_EPSILON_F64 * DSP_NORM_EPSILON_F64)
    {
        for(size_t i = 0; i < len; i++)
        {
            vec[i] = 0.0;
        }

        return 0.0;
    }

    const double norm     = sqrt(sq_norm);
    const double inv_norm = 1.0 / norm;
    for(size_t i = 0; i < len; i++)
    {
        vec[i] = vec[i] * inv_norm;
    }

    return norm;
}

static inline void DSP_Vector_Scale_f64(double* vec, double s, size_t len)
{
    for(size_t i = 0; i < len; i++)
    {
        vec[i] *= s;
    }
}

static inline double DSP_Vector_Dot_f64(const double* a, const double* b, size_t len)
{
    double sum = 0.0;
    for(size_t i = 0; i < len; i++)
    {
        sum += a[i] * b[i];
    }
    return sum;
}

static inline void DSP_Vec3_Cross_f64(double* __restrict result, const double* a, const double* b)
{
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

#endif /* DSP_VECTOR_H__ */
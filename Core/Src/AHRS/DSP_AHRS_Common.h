#include <math.h>

static inline float DSP_AHRS_GainFactor_f32(float accNorm, float cutoffLow, float cutoffHigh)
{
    const float acc_err = fabsf(accNorm - 1.0f);
    if(acc_err > cutoffHigh)
    {
        return 0.0f;
    }

    if(acc_err > cutoffLow)
    {
        return (cutoffHigh - acc_err) / (cutoffHigh - cutoffLow);
    }

    return 1.0f;
}
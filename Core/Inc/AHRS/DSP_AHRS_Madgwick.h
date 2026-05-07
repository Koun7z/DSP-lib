#ifndef DSP_AHRS_MADGWICK_H__
#define DSP_AHRS_MADGWICK_H__

#include "DSP_AHRS_Data.h"

#include <stdbool.h>

typedef struct
{
    float beta;
    bool use_mag;
} DSP_AHRS_Madgwick_Instance_f32;

int DSP_AHRS_Madgwick_Init_f32(DSP_AHRS_Madgwick_Instance_f32* filter, float beta, bool use_mag);

void DSP_AHRS_Madgwick_FilterUpdate_f32(DSP_AHRS_Madgwick_Instance_f32* filter,
                                        DSP_AHRS_DataInstance_f32* data,
                                        float dt);

#endif /* DSP_AHRS_MADGWICK_H__ */
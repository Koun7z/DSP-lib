#ifndef DSP_AHRS_MADGWICK_H__
#define DSP_AHRS_MADGWICK_H__

#include "DSP_AHRS_Data.h"

#include <stdbool.h>

typedef struct
{
    float _beta;
    float _zeta;
    float _b[3];
} DSP_AHRS_Madgwick_Instance_f32;

int DSP_AHRS_Madgwick_InitIMU_f32(DSP_AHRS_Madgwick_Instance_f32* filter, float beta);

int DSP_AHRS_Madgwick_InitMARG_f32(DSP_AHRS_Madgwick_Instance_f32* filter, float beta, float zeta);

void DSP_AHRS_Madgwick_SetGain_f32(DSP_AHRS_Madgwick_Instance_f32* filter, float beta, float zeta);

void DSP_AHRS_Madgwick_UpdateIMU_f32(DSP_AHRS_Madgwick_Instance_f32* filter, DSP_AHRS_DataInstance_f32* data, float dt);

void DSP_AHRS_Madgwick_UpdateMARG_f32(DSP_AHRS_Madgwick_Instance_f32* filter,
                                      DSP_AHRS_DataInstance_f32* data,
                                      float dt);

#endif /* DSP_AHRS_MADGWICK_H__ */
#ifndef DSP_AHRS_MAHONY_H__
#define DSP_AHRS_MAHONY_H__

#include "DSP_AHRS_Data.h"

typedef struct
{
    float Kp;
    float Ki;

    float _k_a;
    float _k_m;

    float _b[3];
} DSP_AHRS_Mahony_Instance_f32;

int DSP_AHRS_Mahony_Init_f32(DSP_AHRS_Mahony_Instance_f32* filter, float kp, float ki);

void DSP_AHRS_Mahony_SetGain_f32(DSP_AHRS_Mahony_Instance_f32* filter, float kp, float ki);

void DSP_AHRS_Mahony_SetMagAccRatio_f32(DSP_AHRS_Mahony_Instance_f32* filter, float magAccRatio);

void DSP_AHRS_Mahony_UpdateIMU_f32(DSP_AHRS_Mahony_Instance_f32* filter, DSP_AHRS_DataInstance_f32* data, float dt);

void DSP_AHRS_Mahony_UpdateMARG_f32(DSP_AHRS_Mahony_Instance_f32* filter, DSP_AHRS_DataInstance_f32* data, float dt);

#endif /* DSP_AHRS_MAHONY_H__ */

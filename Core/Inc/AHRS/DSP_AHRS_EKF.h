#ifndef AHRS_DSP_EKF_H__
#define AHRS_DSP_EKF_H__

#include "DSP_AHRS_Data.h"

#include <stdbool.h>

typedef struct
{
    float GyroNoise[3 * 3];
    float AccNoise[3 * 3];
    float MagNoise[3 * 3];

    float _P[4 * 4];
} DSP_AHRS_EKF_Instance_f32;

int DSP_AHRS_EKF_InitIMU_f32(DSP_AHRS_EKF_Instance_f32* filter,
                             const float gyroNoise[3 * 3],
                             const float accNoise[3 * 3]);

int DSP_AHRS_EKF_InitMARG_f32(DSP_AHRS_EKF_Instance_f32* filter,
                              const float gyroNoise[3 * 3],
                              const float accNoise[3 * 3],
                              const float magNoise[3 * 3]);

int DSP_AHRS_EKF_SetGyroCov_f32(DSP_AHRS_EKF_Instance_f32* filter, const float gyroNoise[3 * 3]);

int DSP_AHRS_EKF_SetAccCov_f32(DSP_AHRS_EKF_Instance_f32* filter, const float accNoise[3 * 3]);

int DSP_AHRS_EKF_SetMagCov_f32(DSP_AHRS_EKF_Instance_f32* filter, const float magNoise[3 * 3]);

void DSP_AHRS_EKF_UpdateIMU_f32(DSP_AHRS_EKF_Instance_f32* filter, DSP_AHRS_DataInstance_f32* data, float dt);

void DSP_AHRS_EKF_UpdateMARG_f32(DSP_AHRS_EKF_Instance_f32* filter, DSP_AHRS_DataInstance_f32* data, float dt);

#endif
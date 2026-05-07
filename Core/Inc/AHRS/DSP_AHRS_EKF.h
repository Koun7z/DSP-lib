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
    bool _useMagnetometer;
} DSP_AHRS_EKF_Instance_f32;

int DSP_AHRS_EKF_Init_f32(DSP_AHRS_EKF_Instance_f32* filter,
                          const float gyroNoise[3 * 3],
                          const float accNoise[3 * 3],
                          const float magNoise[3 * 3]);

void DSP_AHRS_EKF_FilterUpdate_f32(DSP_AHRS_EKF_Instance_f32* filter, DSP_AHRS_DataInstance_f32* data, float dt);

#endif
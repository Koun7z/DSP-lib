//
// Created by pwoli on 18.03.2025.
//

#ifndef DSP_NC_FILTER_H
#define DSP_NC_FILTER_H

#include "DSP_AHRS_Data.h"

#include <stdint.h>

typedef struct
{
    float AccGain;
    float MagGain;
    float SLERP_Threshold;
    float AccCutoffLow;   // Start decreasing acc gain from this point
    float AccCutoffHigh;  // Zero acc gain from this point
} DSP_AHRS_AQUA_Instance_f32;

int DSP_AHRS_AQUA_InitIMU_f32(DSP_AHRS_AQUA_Instance_f32* filter, float AccGain);

int DSP_AHRS_AQUA_InitMARG_f32(DSP_AHRS_AQUA_Instance_f32* filter, float AccGain, float MagGain);

void DSP_AHRS_AQUA_SetAccGain_f32(DSP_AHRS_AQUA_Instance_f32* filter, float AccGain);

void DSP_AHRS_AQUA_SetMagGain_f32(DSP_AHRS_AQUA_Instance_f32* filter, float MagGain);

void DSP_AHRS_AQUA_SetSLERPThreshold_f32(DSP_AHRS_AQUA_Instance_f32* filter, float SLERP_Threshold);

void DSP_AHRS_AQUA_SetAccCutoff_f32(DSP_AHRS_AQUA_Instance_f32* filter, float AccCutoffLow, float AccCutoffHigh);

void DSP_AHRS_AQUA_UpdateIMU_f32(const DSP_AHRS_AQUA_Instance_f32* filter, DSP_AHRS_DataInstance_f32* data, float dt);

void DSP_AHRS_AQUA_UpdateMARG_f32(const DSP_AHRS_AQUA_Instance_f32* filter, DSP_AHRS_DataInstance_f32* data, float dt);

#endif  // DSP_NC_FILTER_H

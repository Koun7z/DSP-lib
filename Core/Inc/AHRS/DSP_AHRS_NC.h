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
} DSP_AHRS_NC_Instance_f32;

uint8_t DSP_AHRS_NC_Init_f32(DSP_AHRS_NC_Instance_f32* instance,
                             float AccGain,
                             float MagGain,
                             float SLERP_Threshold,
                             float AccCutoffLow,
                             float AccCutoffHigh);

void DSP_AHRS_NC_FilterUpdate_f32(const DSP_AHRS_NC_Instance_f32* filter, DSP_AHRS_DataInstance_f32* data, float dt);

#endif  // DSP_NC_FILTER_H

#ifndef DSP_ATTITUDE_COMMON_H
#define DSP_ATTITUDE_COMMON_H

#include "DSP_Quaternion.h"
#include "DSP_SignalFiltering.h"

#include <assert.h>

typedef struct DSP_AHRS_DataInstance_f32
{
	float GyroData[3];
	float AccData[3];
	float MagData[3];

	DSP_Quaternion_f32 AttitudeEstimate;

	DSP_IIR_RT_Instance_f32* _gyroFilters;
	DSP_IIR_RT_Instance_f32* _accFilters;
	DSP_IIR_RT_Instance_f32* _magFilters;
} DSP_AHRS_DataInstance_f32;

void DSP_AHRS_DataInit_f32(DSP_AHRS_DataInstance_f32* instance);

/**
 * @brief Add filter instances for gyroscope data prefiltering
 *
 * @param instance   AHRS data instance
 * @param gyroFilter Array of 3 separate and initialized IIR filter instances for X, Y, Z gyroscope data
 */
void DSP_AHRS_AddGyroFilters_f32(DSP_AHRS_DataInstance_f32* instance, DSP_IIR_RT_Instance_f32 gyroFilter[3]);

/**
 * @brief Add filter instances for accelerometer data prefiltering
 *
 * @param instance   AHRS data instance
 * @param accFilter Array of 3 separate and initialized IIR filter instances for X, Y, Z accelerometer data
 */
void DSP_AHRS_AddAccFilters_f32(DSP_AHRS_DataInstance_f32* instance, DSP_IIR_RT_Instance_f32 accFilter[3]);

/**
 * @brief Add filter instances for magnetometer data prefiltering
 *
 * @param instance   AHRS data instance
 * @param magFilter Array of 3 separate and initialized IIR filter instances for X, Y, Z magnetometer data
 */
void DSP_AHRS_AddMagFilters_f32(DSP_AHRS_DataInstance_f32* instance, DSP_IIR_RT_Instance_f32 magFilter[3]);

static inline void DSP_AHRS_UpdateGyroData_f32(DSP_AHRS_DataInstance_f32* instance, const float gyroData[3])
{
	instance->GyroData[0] = gyroData[0];
	instance->GyroData[1] = gyroData[1];
	instance->GyroData[2] = gyroData[2];
}

static inline void DSP_AHRS_UpdateAccData_f32(DSP_AHRS_DataInstance_f32* instance, const float accData[3])
{
	instance->AccData[0] = accData[0];
	instance->AccData[1] = accData[1];
	instance->AccData[2] = accData[2];
}

static inline void DSP_AHRS_UpdateMagData_f32(DSP_AHRS_DataInstance_f32* instance, const float magData[3])
{
	instance->MagData[0] = magData[0];
	instance->MagData[1] = magData[1];
	instance->MagData[2] = magData[2];
}

void DSP_AHRS_UpdateFilterGyroData_f32(DSP_AHRS_DataInstance_f32* instance, const float gyroData[3]);

void DSP_AHRS_UpdateFilterAccData_f32(DSP_AHRS_DataInstance_f32* instance, const float accData[3]);

void DSP_AHRS_UpdateFilterMagData_f32(DSP_AHRS_DataInstance_f32* instance, const float magData[3]);
#endif  // DSP_ATTITUDE_COMMON_H

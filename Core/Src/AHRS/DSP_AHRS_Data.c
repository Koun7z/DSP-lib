#include "DSP_AHRS_Data.h"
#include "DSP_SignalFiltering.h"

void DSP_AHRS_DataInit_f32(DSP_AHRS_DataInstance_f32* instance)
{
    instance->GyroData[0] = 0.0f;
    instance->GyroData[1] = 0.0f;
    instance->GyroData[2] = 0.0f;

    instance->AccData[0] = 0.0f;
    instance->AccData[1] = 0.0f;
    instance->AccData[2] = 1.0f;  // Initial gravity vector (z-axis)

    instance->MagData[0] = 1.0f;  // Initial north heading (x-axis)
    instance->MagData[1] = 0.0f;
    instance->MagData[2] = 0.0f;

    instance->AttitudeEstimate = (DSP_Quaternion_f32) {1, 0, 0, 0};  // Identity quaternion

    instance->_gyroFilters = NULL;
    instance->_accFilters  = NULL;
    instance->_magFilters  = NULL;
}

void DSP_AHRS_AddGyroFilters_f32(DSP_AHRS_DataInstance_f32* instance, DSP_IIR_RT_Instance_f32 gyroFilter[3])
{
    instance->_gyroFilters = gyroFilter;
}

void DSP_AHRS_AddAccFilters_f32(DSP_AHRS_DataInstance_f32* instance, DSP_IIR_RT_Instance_f32 accFilter[3])
{
    instance->_accFilters = accFilter;
}

void DSP_AHRS_AddMagFilters_f32(DSP_AHRS_DataInstance_f32* instance, DSP_IIR_RT_Instance_f32 magFilter[3])
{
    instance->_magFilters = magFilter;
}

void DSP_AHRS_UpdateFilterGyroData_f32(DSP_AHRS_DataInstance_f32* instance, const float gyroData[3])
{
    assert(instance->_gyroFilters);

    for(size_t i = 0; i < 3; i++)
    {
        instance->GyroData[i] = DSP_IIR_RT_Update_f32(&instance->_gyroFilters[i], gyroData[i]);
    }
}

void DSP_AHRS_UpdateFilterAccData_f32(DSP_AHRS_DataInstance_f32* instance, const float accData[3])
{
    assert(instance->_accFilters);

    for(size_t i = 0; i < 3; i++)
    {
        instance->AccData[i] = DSP_IIR_RT_Update_f32(&instance->_accFilters[i], accData[i]);
    }
}

void DSP_AHRS_UpdateFilterMagData_f32(DSP_AHRS_DataInstance_f32* instance, const float magData[3])
{
    assert(instance->_magFilters);

    for(size_t i = 0; i < 3; i++)
    {
        instance->MagData[i] = DSP_IIR_RT_Update_f32(&instance->_magFilters[i], magData[i]);
    }
}
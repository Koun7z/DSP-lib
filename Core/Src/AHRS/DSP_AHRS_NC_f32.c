//
// Created by pwoli on 18.03.2025.
//
#include "DSP_AHRS_NC.h"

#include <math.h>

uint8_t DSP_AHRS_NC_Init_f32(DSP_AHRS_NC_Instance_f32* instance,
                             const float AccGain,
                             const float MagGain,
                             const float SLERP_Threshold,
                             const float AccCutoffLow,
                             const float AccCutoffHigh)
{
    instance->AccGain         = AccGain;
    instance->MagGain         = MagGain;
    instance->SLERP_Threshold = SLERP_Threshold;
    instance->AccCutoffLow    = AccCutoffLow;
    instance->AccCutoffHigh   = AccCutoffHigh;

    return 0;
}

void DSP_AHRS_NC_FilterUpdate_f32(const DSP_AHRS_NC_Instance_f32* filter,
                                  DSP_AHRS_DataInstance_f32* data,
                                  const float dt)
{
    const float p = data->GyroData[0];
    const float q = data->GyroData[1];
    const float r = data->GyroData[2];

    float a_i = data->AccData[0];
    float a_j = data->AccData[1];
    float a_k = data->AccData[2];

    // Integrating local angular velocity
    const DSP_Quaternion_f32 omega_l = {.r = 0.0f, .i = p, .j = q, .k = r};

    DSP_Quaternion_f32 delta_q_gyro;

    DSP_QT_Multiply_f32(&delta_q_gyro, &data->AttitudeEstimate, &omega_l);
    DSP_QT_Scale_f32(&delta_q_gyro, &delta_q_gyro, 0.5f * dt);

    DSP_Quaternion_f32 q_gyro;

    DSP_QT_Add_f32(&q_gyro, &data->AttitudeEstimate, &delta_q_gyro);
    DSP_QT_Normalize_f32(&q_gyro, &q_gyro);


    // Normalizing local acceleration vector
    const float acc_norm = sqrtf(a_i * a_i + a_j * a_j + a_k * a_k);

    a_i = a_i / acc_norm;
    a_j = a_j / acc_norm;
    a_k = a_k / acc_norm;

    // Calculating predicted gravity vector
    float g_pred[3] = {a_i, a_j, a_k};

    DSP_QT_RotateVector_f32(g_pred, g_pred, &q_gyro);

    // Calculating accelerometer correction
    DSP_Quaternion_f32 delta_q_acc;
    if(g_pred[2] >= 0.0f)
    {
        const float sq_gk = sqrtf(2.0f * (g_pred[2] + 1.0f));

        delta_q_acc.r = sqrtf((g_pred[2] + 1.0f) / 2.0f);
        delta_q_acc.i = g_pred[1] / sq_gk;
        delta_q_acc.j = -g_pred[0] / sq_gk;
        delta_q_acc.k = 0.0f;
    }
    else
    {
        const float sq_gk = sqrtf(2.0f * (1.0f - g_pred[2]));

        delta_q_acc.r = -g_pred[1] / sq_gk;
        delta_q_acc.i = -sqrtf((1.0f - g_pred[2]) / 2.0f);
        delta_q_acc.j = 0.0f;
        delta_q_acc.k = -g_pred[0] / sq_gk;
    }


    // Simple adaptive gain
    float acc_gain      = filter->AccGain;
    const float acc_err = fabsf(acc_norm - 1.0f);

    if(acc_err > filter->AccCutoffHigh)
    {
        acc_gain = 0.0f;
    }
    else if(acc_err > filter->AccCutoffLow)
    {
        acc_gain *= (filter->AccCutoffHigh - acc_err) / (filter->AccCutoffHigh - filter->AccCutoffLow);
    }

    // Applying gain to accelerometer correction
    if(delta_q_acc.r > filter->SLERP_Threshold)
    {
        DSP_QT_Scale_f32(&delta_q_acc, &delta_q_acc, acc_gain);
        delta_q_acc.r += 1.0f - acc_gain;
    }
    else
    {
        const float omega_slerp = acosf(delta_q_acc.r);
        const float sin_om      = sinf(omega_slerp);

        const float scale = sinf(acc_gain * omega_slerp) / sin_om;
        DSP_QT_Scale_f32(&delta_q_acc, &delta_q_acc, scale);

        delta_q_acc.r += sinf((1.0f - acc_gain) * omega_slerp) / sin_om;
    }

    // Applying accelerometer correction
    DSP_QT_Multiply_f32(&data->AttitudeEstimate, &delta_q_acc, &q_gyro);
    DSP_QT_Normalize_f32(&data->AttitudeEstimate, &data->AttitudeEstimate);

    /*
    ** Magnetometer yaw correction
    */

    if(filter->MagGain == 0.0f)
    {
        return;
    }

    float l[3] = {data->MagData[0], data->MagData[1], data->MagData[2]};
    DSP_QT_RotateVector_f32(l, l, &data->AttitudeEstimate);

    DSP_Quaternion_f32 delta_q_mag;
    const float gamma    = sqrtf(l[0] * l[0] + l[1] * l[1]);
    const float sq_gamma = sqrtf(gamma);

    if(l[0] >= 0)
    {
        delta_q_mag.r = sqrtf(gamma + l[0] * sq_gamma) / (sqrtf(2.0f) * sq_gamma);
        delta_q_mag.i = 0.0f;
        delta_q_mag.j = 0.0f;
        delta_q_mag.k = -l[1] / sqrtf(2.0f * (gamma + l[0] * sq_gamma));
    }
    else
    {
        delta_q_mag.r = l[1] / sqrtf(2.0f * (gamma - l[0] * sq_gamma));
        delta_q_mag.i = 0.0f;
        delta_q_mag.j = 0.0f;
        delta_q_mag.k = -sqrtf(gamma - l[0] * sq_gamma) / (sqrtf(2.0f) * sq_gamma);
    }

    if(delta_q_mag.r > filter->SLERP_Threshold)
    {
        DSP_QT_Scale_f32(&delta_q_mag, &delta_q_mag, filter->MagGain);
        delta_q_mag.r += 1.0f - filter->MagGain;
    }
    else
    {
        const float omega_slerp = acosf(delta_q_mag.r);
        const float sin_om      = sinf(omega_slerp);

        const float scale = sinf(filter->MagGain * omega_slerp) / sin_om;
        DSP_QT_Scale_f32(&delta_q_mag, &delta_q_mag, scale);

        delta_q_mag.r += sinf((1.0f - filter->MagGain) * omega_slerp) / sin_om;
    }


    // Applying magnetometer correction
    DSP_QT_Multiply_f32(&data->AttitudeEstimate, &delta_q_mag, &data->AttitudeEstimate);
    DSP_QT_Normalize_f32(&data->AttitudeEstimate, &data->AttitudeEstimate);
}

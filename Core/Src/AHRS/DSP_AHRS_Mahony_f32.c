#include "DSP_AHRS_Mahony.h"

#include <math.h>

int DSP_AHRS_Mahony_Init_f32(DSP_AHRS_Mahony_Instance_f32* filter, float kp, float ki, float magAccRatio)
{
    filter->Kp = kp;
    filter->Ki = ki;

    if(magAccRatio < 1e-6)
    {
        filter->k_a = 1.0f;
        filter->k_m = 0.0f;
    }
    else if(magAccRatio > 1)
    {
        filter->k_a = 0.0f;
        filter->k_m = 1.0f;
    }
    else
    {
        filter->k_a = 1.0f - magAccRatio;
        filter->k_m = magAccRatio;
    }

    filter->_b[0] = 0.0f;
    filter->_b[1] = 0.0f;
    filter->_b[2] = 0.0f;

    return 0;
}

void DSP_AHRS_Mahony_FilterUpdate_f32(DSP_AHRS_Mahony_Instance_f32* filter, DSP_AHRS_DataInstance_f32* data, float dt)
{
    const float p = data->GyroData[0];
    const float q = data->GyroData[1];
    const float r = data->GyroData[2];

    float a_i = data->AccData[0];
    float a_j = data->AccData[1];
    float a_k = data->AccData[2];

    float m_i = data->MagData[0];
    float m_j = data->MagData[1];
    float m_k = data->MagData[2];

    float omega_m[3] = {0.0f, 0.0f, 0.0f};

    if(filter->k_a != 0.0f)
    {
        float a_norm = sqrtf(a_i * a_i + a_j * a_j + a_k * a_k);
        a_i          = a_i / a_norm;
        a_j          = a_j / a_norm;
        a_k          = a_k / a_norm;

        float g_pred[3] = {0.0f, 0.0f, 1.0f};
        DSP_QT_RotateVectorInv_f32(g_pred, g_pred, &data->AttitudeEstimate);

        // Cross product between measured and predicted gravity vector
        omega_m[0] += filter->k_a * (a_j * g_pred[2] - a_k * g_pred[1]);
        omega_m[1] += filter->k_a * (a_k * g_pred[0] - a_i * g_pred[2]);
        omega_m[2] += filter->k_a * (a_i * g_pred[1] - a_j * g_pred[0]);
    }

    if(filter->k_m != 0.0f)
    {
        float m_norm = sqrtf(m_i * m_i + m_j * m_j + m_k * m_k);
        m_i          = m_i / m_norm;
        m_j          = m_j / m_norm;
        m_k          = m_k / m_norm;

        // TODO: Project the magnetic vector to the XY frame, maybe?

        float m_pred[3] = {1.0f, 0.0f, 0.0f};
        DSP_QT_RotateVectorInv_f32(m_pred, m_pred, &data->AttitudeEstimate);

        // Cross product between measured and predicted magnetic vector
        omega_m[0] += filter->k_m * (m_j * m_pred[2] - m_k * m_pred[1]);
        omega_m[1] += filter->k_m * (m_k * m_pred[0] - m_i * m_pred[2]);
        omega_m[2] += filter->k_m * (m_i * m_pred[1] - m_j * m_pred[0]);
    }

    DSP_Quaternion_f32 omega = {.r = 0.0f,
                                .i = p + filter->Kp * omega_m[0] + filter->_b[0],
                                .j = q + filter->Kp * omega_m[1] + filter->_b[1],
                                .k = r + filter->Kp * omega_m[2] + filter->_b[2]};

    // q_{k + 1} = q{k} + 0.5*q{k}*omega*dt
    DSP_Quaternion_f32 delta_q;
    DSP_QT_Multiply_f32(&delta_q, &data->AttitudeEstimate, &omega);
    DSP_QT_Scale_f32(&delta_q, &delta_q, 0.5f * dt);

    DSP_QT_Add_f32(&data->AttitudeEstimate, &data->AttitudeEstimate, &delta_q);
    DSP_QT_Normalize_f32(&data->AttitudeEstimate, &data->AttitudeEstimate);

    filter->_b[0] += filter->Ki * omega_m[0] * dt;
    filter->_b[1] += filter->Ki * omega_m[1] * dt;
    filter->_b[2] += filter->Ki * omega_m[2] * dt;
}
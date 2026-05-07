#include "DSP_AHRS_Madgwick.h"
#include "DSP_Matrix.h"

#include <math.h>

int DSP_AHRS_Madgwick_Init_f32(DSP_AHRS_Madgwick_Instance_f32* filter, float beta, bool use_mag)
{
    filter->beta    = beta;
    filter->use_mag = use_mag;
    return 0;
}

void DSP_AHRS_Madgwick_FilterUpdate_f32(DSP_AHRS_Madgwick_Instance_f32* filter,
                                        DSP_AHRS_DataInstance_f32* data,
                                        float dt)
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

    float a_norm = sqrtf(a_i * a_i + a_j * a_j + a_k * a_k);
    a_i          = a_i / a_norm;
    a_j          = a_j / a_norm;
    a_k          = a_k / a_norm;

    // Integrating local angular velocity
    const DSP_Quaternion_f32 omega_l = {.r = 0.0f, .i = p, .j = q, .k = r};

    DSP_Quaternion_f32 delta_q_gyro;
    DSP_QT_Multiply_f32(&delta_q_gyro, &data->AttitudeEstimate, &omega_l);
    DSP_QT_Scale_f32(&delta_q_gyro, &delta_q_gyro, 0.5f);

    // I wanted to use the i,j,k notation here but name conflicts :(
    const float w = data->AttitudeEstimate.r;
    const float x = data->AttitudeEstimate.i;
    const float y = data->AttitudeEstimate.j;
    const float z = data->AttitudeEstimate.k;

    float grad[4];
    if(filter->use_mag)
    {
        float m_norm = sqrtf(m_i * m_i + m_j * m_j + m_k * m_k);
        m_i          = m_i / m_norm;
        m_j          = m_j / m_norm;
        m_k          = m_k / m_norm;

        // TODO:

        float f_gm[6] = {
            2 * (x * z - w * y) - a_i,
            2 * (w * x + y * z) - a_j,
            2 * (0.5f - x * x - y * y) - a_k,
            //
            //
            //
        };


        // clang-format off
        float J_gm[6 * 4] = {
            -2*y,  2*z, -2*w,  2*x,
             2*x,  2*w,  2*z,  2*y,
            0.0f, -4*x, -4*y, 0.0f,
            //  
            //
            //

        };
        // clang-format on

        // DSP_Matrix_Multiply_f32(grad, J_gmT, 4, 6, f_gm, 1);
    }
    else
    {
        float f_g[3] = {
            2 * (x * z - w * y) - a_i,
            2 * (w * x + y * z) - a_j,
            2 * (0.5f - x * x - y * y) - a_k,
        };

        // float J_g[3 * 4] = {
        //     -2*y,  2*z, -2*w,  2*x,
        //      2*x,  2*w,  2*z,  2*y,
        //     0.0f, -4*x, -4*y,  0.0f
        // };

        // clang-format off
        float J_gT[4 * 3] = {
            -2*y,  2*x, 0.0f,
             2*z,  2*w, -4*x,
            -2*w,  2*z, -4*y,
             2*x,  2*y, 0.0f
        };
        // clang-format on

        DSP_Matrix_Multiply_f32(grad, J_gT, 4, 3, f_g, 1);
    }


    float grad_norm = sqrtf(grad[0] * grad[0] + grad[1] * grad[1] + grad[2] * grad[2] + grad[3] * grad[3]);
    grad[0]         = grad[0] / grad_norm;
    grad[1]         = grad[1] / grad_norm;
    grad[2]         = grad[2] / grad_norm;
    grad[3]         = grad[3] / grad_norm;

    data->AttitudeEstimate.r += (delta_q_gyro.r - filter->beta * grad[0]) * dt;
    data->AttitudeEstimate.i += (delta_q_gyro.i - filter->beta * grad[1]) * dt;
    data->AttitudeEstimate.j += (delta_q_gyro.j - filter->beta * grad[2]) * dt;
    data->AttitudeEstimate.k += (delta_q_gyro.k - filter->beta * grad[3]) * dt;

    DSP_QT_Normalize_f32(&data->AttitudeEstimate, &data->AttitudeEstimate);
}
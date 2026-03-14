#include "DSP_AHRS_EKF.h"

#include "DSP_Matrix.h"

#include <string.h>

int DSP_AHRS_EKF_Init_f32(DSP_AHRS_EKF_Instance_f32* filter,
                          const float GyroNoise[3],
                          const float AccNoise[3],
                          const float MagNoise[3])
{
    for(size_t i = 0; i < 3; i++)
    {
        filter->GyroNoise[i] = GyroNoise[i];
        filter->AccNoise[i]  = AccNoise[i];
        filter->MagNoise[i]  = MagNoise[i];
    }

    memset(filter->_covariance, 0, 16 * sizeof(float));
    filter->_covariance[0]  = 1.0f;
    filter->_covariance[5]  = 1.0f;
    filter->_covariance[10] = 1.0f;
    filter->_covariance[15] = 1.0f;

    return 0;
}

void DSP_AHRS_EKF_FilterUpdate_f32(DSP_AHRS_EKF_Instance_f32* filter, DSP_AHRS_DataInstance_f32* data, float dt)
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
    // DSP_QT_Normalize_f32(&q_gyro, &q_gyro);

    /*
    ** Jacobian F
    */

    // clang-format off
    const float dt_2 = 0.5f * dt;
    float F[16] = {
            1.0f, -dt_2 * p, -dt_2 * q, -dt_2 * r,
        dt_2 * p,      1.0f,  dt_2 * r, -dt_2 * q,
        dt_2 * q, -dt_2 * r,      1.0f,  dt_2 * p,
        dt_2 * r,  dt_2 * q, -dt_2 * p,      1.0f,     
    };
    // clang-format on

    /*
    ** Process noise covariance Q = W * Sigma * W^T
    ** W = df / dw
    */

    const float w     = data->AttitudeEstimate.r;
    const float x     = data->AttitudeEstimate.i;
    const float y     = data->AttitudeEstimate.j;
    const float z     = data->AttitudeEstimate.k;
    const float sx    = filter->GyroNoise[0];
    const float sy    = filter->GyroNoise[1];
    const float sz    = filter->GyroNoise[2];  // variances
    const float dt2_4 = 0.25f * dt * dt;

    const float ww = w * w;
    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;

    // Off-diagonal terms
    const float sigma1 = -dt2_4 * (w * x * sx - y * z * sy + y * z * sz);
    const float sigma2 = -dt2_4 * (x * z * sx + w * y * sy - x * z * sz);
    const float sigma3 = -dt2_4 * (-x * y * sx + x * y * sy + w * z * sz);
    const float sigma4 = -dt2_4 * (-w * z * sx + w * z * sy + x * y * sz);
    const float sigma5 = -dt2_4 * (w * y * sx + x * z * sy - w * y * sz);
    const float sigma6 = -dt2_4 * (y * z * sx - w * x * sy + w * x * sz);

    // Diagonal terms
    const float d1 = dt2_4 * (sx * xx + sy * yy + sz * zz);  // (0,0)
    const float d2 = dt2_4 * (sx * ww + sy * zz + sz * yy);  // (1,1)
    const float d3 = dt2_4 * (sx * zz + sy * ww + sz * xx);  // (2,2)
    const float d4 = dt2_4 * (sx * yy + sy * xx + sz * ww);  // (3,3)

    // clang-format off
    float Q[16] = {
            d1, sigma1, sigma2, sigma3,
        sigma1,     d2, sigma4, sigma5,
        sigma2, sigma4,     d3, sigma6,
        sigma3, sigma5, sigma6,     d4,    
    };
    // clang-format on

    // P_t = F * P * F^T + Q

    float P_t[16];
    DSP_Matrix_Multiply_f32(P_t, F, 4, 4, filter->_covariance, 4);
    DSP_Matrix_TransposeInline_f32(F, 4);
    DSP_Matrix_Multiply_f32(filter->_covariance, P_t, 4, 4, F, 4);
    DSP_Matrix_AddInline_f32(filter->_covariance, Q, 4, 4);

    /*
    ** Correction step (TODO)
    */
}

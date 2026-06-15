#include "DSP_AHRS_Madgwick.h"

#include "DSP_Matrix.h"
#include "DSP_Vector.h"

#include "DSP_Assert.h"

int DSP_AHRS_Madgwick_InitIMU_f32(DSP_AHRS_Madgwick_Instance_f32* filter, float beta)
{
    filter->_beta = beta;
    filter->_zeta = 0.0f;
    filter->_b[0] = 0.0f;
    filter->_b[1] = 0.0f;
    filter->_b[2] = 0.0f;
    return 0;
}

int DSP_AHRS_Madgwick_InitMARG_f32(DSP_AHRS_Madgwick_Instance_f32* filter, float beta, float zeta)
{
    filter->_beta = beta;
    filter->_zeta = zeta;
    filter->_b[0] = 0.0f;
    filter->_b[1] = 0.0f;
    filter->_b[2] = 0.0f;
    return 0;
}

void DSP_AHRS_Madgwick_SetGain_f32(DSP_AHRS_Madgwick_Instance_f32* filter, float beta, float zeta)
{
    filter->_beta = beta;
    filter->_zeta = zeta;
}

void DSP_AHRS_Madgwick_UpdateIMU_f32(DSP_AHRS_Madgwick_Instance_f32* filter, DSP_AHRS_DataInstance_f32* data, float dt)
{
    DSP_ASSERT(filter && data);
    DSP_ASSERT(dt > 0.0f);


    const float p = data->GyroData[0];
    const float q = data->GyroData[1];
    const float r = data->GyroData[2];

    float acc_data[3] = {data->AccData[0], data->AccData[1], data->AccData[2]};
    DSP_Vector_Normalize_f32(acc_data, 3);
    const float a_i = acc_data[0];
    const float a_j = acc_data[1];
    const float a_k = acc_data[2];


    // Integrating local angular velocity
    const DSP_Quaternion_f32 omega_l = {.r = 0.0f, .i = p, .j = q, .k = r};

    DSP_Quaternion_f32 delta_q_gyro;
    DSP_QT_Multiply_f32(&delta_q_gyro, &data->AttitudeEstimate, &omega_l);
    DSP_QT_Scale_f32(&delta_q_gyro, &delta_q_gyro, 0.5f);

    // I wanted to use the i,j,k notation here but name conflicts :(
    const float q_r = data->AttitudeEstimate.r;
    const float q_i = data->AttitudeEstimate.i;
    const float q_j = data->AttitudeEstimate.j;
    const float q_k = data->AttitudeEstimate.k;

    const float q_ii = q_i * q_i;
    const float q_jj = q_j * q_j;

    // clang-format off
    const float  f_g[3] = {
        2 * (q_i  * q_k  - q_r  * q_j) - a_i,
        2 * (q_r  * q_i  + q_j  * q_k) - a_j,
        2 * (0.5f - q_ii - q_jj)    - a_k,
    };

    // float J_g[3 * 4] = {
    //     -2*y,  2*z, -2*w,  2*x,
    //      2*x,  2*w,  2*z,  2*y,
    //     0.0f, -4*x, -4*y,  0.0f
    // };

    // const float J_gT[4 * 3] = {
    //     -2*q_j,  2*q_i,   0.0f,
    //      2*q_k,  2*q_r, -4*q_i,
    //     -2*q_r,  2*q_k, -4*q_j,
    //      2*q_i,  2*q_j,   0.0f
    // };

    DSP_Quaternion_f32 grad = {
        .r = -2 * q_j * f_g[0] + 2 * q_i * f_g[1] +     0.0f * f_g[2],
        .i =  2 * q_k * f_g[0] + 2 * q_r * f_g[1] + -4 * q_i * f_g[2],
        .j = -2 * q_r * f_g[0] + 2 * q_k * f_g[1] + -4 * q_j * f_g[2],
        .k =  2 * q_i * f_g[0] + 2 * q_j * f_g[1] +     0.0f * f_g[2],
    };
    // clang-format on

    DSP_QT_Normalize_f32(&grad, &grad);
    data->AttitudeEstimate.r += (delta_q_gyro.r - filter->_beta * grad.r) * dt;
    data->AttitudeEstimate.i += (delta_q_gyro.i - filter->_beta * grad.i) * dt;
    data->AttitudeEstimate.j += (delta_q_gyro.j - filter->_beta * grad.j) * dt;
    data->AttitudeEstimate.k += (delta_q_gyro.k - filter->_beta * grad.k) * dt;

    DSP_QT_Normalize_f32(&data->AttitudeEstimate, &data->AttitudeEstimate);
}

void DSP_AHRS_Madgwick_UpdateMARG_f32(DSP_AHRS_Madgwick_Instance_f32* filter, DSP_AHRS_DataInstance_f32* data, float dt)
{
    DSP_ASSERT(filter && data);
    DSP_ASSERT(dt > 0.0f);

    const float p = data->GyroData[0];
    const float q = data->GyroData[1];
    const float r = data->GyroData[2];

    /* Integrating local angular velocity */
    const DSP_Quaternion_f32 omega_l = {
        .r = 0.0f,
        .i = p - filter->_b[0],
        .j = q - filter->_b[1],
        .k = r - filter->_b[2],
    };

    DSP_Quaternion_f32 delta_q_gyro;
    DSP_QT_Multiply_f32(&delta_q_gyro, &data->AttitudeEstimate, &omega_l);
    DSP_QT_Scale_f32(&delta_q_gyro, &delta_q_gyro, 0.5f);

    float acc_data[3] = {data->AccData[0], data->AccData[1], data->AccData[2]};
    DSP_Vector_Normalize_f32(acc_data, 3);
    const float a_i = acc_data[0];
    const float a_j = acc_data[1];
    const float a_k = acc_data[2];

    const float q_r = data->AttitudeEstimate.r;
    const float q_i = data->AttitudeEstimate.i;
    const float q_j = data->AttitudeEstimate.j;
    const float q_k = data->AttitudeEstimate.k;

    const float q_ii = q_i * q_i;
    const float q_jj = q_j * q_j;
    const float q_kk = q_k * q_k;

    float mag_data[3] = {data->MagData[0], data->MagData[1], data->MagData[2]};
    DSP_Vector_Normalize_f32(mag_data, 3);
    const float m_i = mag_data[0];
    const float m_j = mag_data[1];
    const float m_k = mag_data[2];

    DSP_QT_RotateVector_f32(mag_data, mag_data, &data->AttitudeEstimate);
    const float h_i = mag_data[0];
    const float h_j = mag_data[1];
    const float h_k = mag_data[2];

    const float b_i = sqrtf(h_i * h_i + h_j * h_j);
    const float b_k = h_k;

    // clang-format off
    float f_gb[6] = {
        /* f_g */
        2 * (q_i * q_k  - q_r  * q_j )  - a_i,
        2 * (q_r * q_i  + q_j  * q_k )  - a_j,
        2 * (      0.5f - q_ii - q_jj)  - a_k,
        /* f_b */
        2 * b_i * (      0.5f - q_jj - q_kk) + 2 * b_k * (q_i * q_k  - q_r  * q_j ) - m_i,
        2 * b_i * (q_i * q_j  - q_r  * q_k ) + 2 * b_k * (q_r * q_i  + q_j  * q_k ) - m_j,
        2 * b_i * (q_r * q_j  + q_i  * q_k ) + 2 * b_k * (      0.5f - q_ii - q_jj) - m_k,
    };
    // clang-format on

    // clang-format off
    // const float J_gb[6 * 4] = {
    //                     -2*q_j,                 2*q_k,                 -2*q_r,                  2*q_i,
    //                      2*q_i,                 2*q_r,                  2*q_k,                  2*q_j,
    //                       0.0f,                -4*q_i,                 -4*q_j,                   0.0f,
    //                 -2*b_k*q_j,             2*b_k*q_k, -4*b_i*q_j - 2*b_k*q_r, -4*b_i*q_k + 2*b_k*q_i,
    //     -2*b_i*q_k + 2*b_k*q_i, 2*b_i*q_j + 2*b_k*q_r,  2*b_i*q_i + 2*b_k*q_k, -2*b_i*q_r + 2*b_k*q_j,
    //                  2*b_i*q_j, 2*b_i*q_k - 4*b_k*q_i,  2*b_i*q_r - 4*b_k*q_j,              2*b_i*q_i,
    // };
    // clang-format on

    // clang-format off
    const float J_gbT[4 * 6] = {
        -2*q_j,  2*q_i,   0.0f,              -2*b_k*q_j,  -2*b_i*q_k + 2*b_k*q_i,              2*b_i*q_j,
         2*q_k,  2*q_r, -4*q_i,               2*b_k*q_k,   2*b_i*q_j + 2*b_k*q_r,  2*b_i*q_k - 4*b_k*q_i,
        -2*q_r,  2*q_k, -4*q_j,  -4*b_i*q_j - 2*b_k*q_r,   2*b_i*q_i + 2*b_k*q_k,  2*b_i*q_r - 4*b_k*q_j,
         2*q_i,  2*q_j,   0.0f,  -4*b_i*q_k + 2*b_k*q_i,  -2*b_i*q_r + 2*b_k*q_j,              2*b_i*q_i,
    };
    // clang-format on

    DSP_Quaternion_f32 grad;
    DSP_Matrix_Multiply_f32(grad.q, J_gbT, 4, 6, f_gb, 1);
    DSP_QT_Normalize_f32(&grad, &grad);

    DSP_Quaternion_f32 omega_eps;
    DSP_QT_Multiply_f32(&omega_eps, &data->AttitudeEstimate, &grad);
    filter->_b[0] += 2.0f * filter->_zeta * omega_eps.i * dt;
    filter->_b[1] += 2.0f * filter->_zeta * omega_eps.j * dt;
    filter->_b[2] += 2.0f * filter->_zeta * omega_eps.k * dt;


    data->AttitudeEstimate.r += (delta_q_gyro.r - filter->_beta * grad.r) * dt;
    data->AttitudeEstimate.i += (delta_q_gyro.i - filter->_beta * grad.i) * dt;
    data->AttitudeEstimate.j += (delta_q_gyro.j - filter->_beta * grad.j) * dt;
    data->AttitudeEstimate.k += (delta_q_gyro.k - filter->_beta * grad.k) * dt;

    DSP_QT_Normalize_f32(&data->AttitudeEstimate, &data->AttitudeEstimate);
}
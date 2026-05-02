#include "DSP_AHRS_EKF.h"

#include "DSP_Matrix.h"

#include <math.h>
#include <string.h>

int DSP_AHRS_EKF_Init_f32(DSP_AHRS_EKF_Instance_f32* filter,
                          const float gyroNoise[3],
                          const float accNoise[3],
                          const float magNoise[3])
{
    filter->_useMagnetometer = false;
    for(size_t i = 0; i < 3; i++)
    {
        filter->GyroNoise[i] = gyroNoise[i];
        filter->AccNoise[i]  = accNoise[i];

        if(magNoise != NULL)
        {
            filter->MagNoise[i]      = magNoise[i];
            filter->_useMagnetometer = true;
        }
    }

    memset(filter->_P, 0, 16 * sizeof(float));
    filter->_P[0]  = 1.0f;
    filter->_P[5]  = 1.0f;
    filter->_P[10] = 1.0f;
    filter->_P[15] = 1.0f;

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

    float norm_acc = sqrtf(a_i * a_i + a_j * a_j + a_k * a_k);

    a_i /= norm_acc;
    a_j /= norm_acc;
    a_k /= norm_acc;

    // Integrating local angular velocity
    const DSP_Quaternion_f32 omega_l = {.r = 0.0f, .i = p, .j = q, .k = r};

    DSP_Quaternion_f32 delta_q_gyro;
    DSP_QT_Multiply_f32(&delta_q_gyro, &data->AttitudeEstimate, &omega_l);
    DSP_QT_Scale_f32(&delta_q_gyro, &delta_q_gyro, 0.5f * dt);

    DSP_Quaternion_f32 q_gyro;
    DSP_QT_Add_f32(&q_gyro, &data->AttitudeEstimate, &delta_q_gyro);
    DSP_QT_Normalize_f32(&q_gyro, &q_gyro);

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

    const float w     = q_gyro.r;
    const float x     = q_gyro.i;
    const float y     = q_gyro.j;
    const float z     = q_gyro.k;
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

    float P_hat[16];
    DSP_Matrix_SandwichMultiply_f32(P_hat, F, filter->_P, 4, 4);
    DSP_Matrix_AddInline_f32(P_hat, Q, 4, 4);

    /*
    ** Correction step
    */

    const float g[3] = {0.0f, 0.0f, 1.0f};

    if(filter->_useMagnetometer)
    {
        // float h_q[6];
    }
    else
    {
        // clang-format off
        
        // float h_q[3] = {
        //     2 * q_gyro.i * q_gyro.k - 2 * q_gyro.r * q_gyro.j,                                      
        //     2 * q_gyro.j * q_gyro.k + 2 * q_gyro.r * q_gyro.i,                                      
        //     -q_gyro.i * q_gyro.i - q_gyro.j * q_gyro.j + q_gyro.k * q_gyro.k + q_gyro.r * q_gyro.r
        // };

        // y = z - h(x)
        float y[3] = {
            a_i - 2 * q_gyro.i * q_gyro.k + 2 * q_gyro.r * q_gyro.j,                                      
            a_j - 2 * q_gyro.j * q_gyro.k - 2 * q_gyro.r * q_gyro.i,                                      
            a_k + q_gyro.i * q_gyro.i + q_gyro.j * q_gyro.j - q_gyro.k * q_gyro.k - q_gyro.r * q_gyro.r
        };

        float H[12] = {
                -2 * q_gyro.j,  2 * q_gyro.k, -2 * q_gyro.r, 2 * q_gyro.i,
                2 * q_gyro.i,  2 * q_gyro.r,  2 * q_gyro.k, 2 * q_gyro.j,
                2 * q_gyro.r, -2 * q_gyro.i, -2 * q_gyro.j, 2 * q_gyro.k
        };

        float H_t[12] = {
            -2 * q_gyro.j,  2 * q_gyro.i,  2 * q_gyro.r,
             2 * q_gyro.k,  2 * q_gyro.r, -2 * q_gyro.i,
            -2 * q_gyro.r,  2 * q_gyro.k, -2 * q_gyro.j,
             2 * q_gyro.i,  2 * q_gyro.j,  2 * q_gyro.k
        };
        // clang-format on

        // S[3x3]
        float S[9];
        DSP_Matrix_SandwichMultiply_f32(S, H, P_hat, 3, 4);
        S[0] += filter->AccNoise[0];
        S[4] += filter->AccNoise[1];
        S[8] += filter->AccNoise[2];

        // K[4x3]
        // K = P * H^T * S^-1
        // KS = P * H^T
        float K[12];

        // PH_T[4x3]
        float PH_t[12];
        DSP_Matrix_Multiply_f32(PH_t, P_hat, 4, 4, H_t, 3);

        size_t LU_P[4];
        DSP_Matrix_LUPDecompose_f32(S, 3, 1e-6f, LU_P);

        // Right solve for every column of K
        for(size_t i = 0; i < 4; i++)
        {
            DSP_Matrix_LUPRightSolve_f32(S, LU_P, K + i * 3, PH_t + i * 3, 3);
        }

        // q = q + K * y
        DSP_Matrix_Multiply_f32((float*)&data->AttitudeEstimate, K, 4, 3, y, 1);
        DSP_Matrix_AddInline_f32((float*)&data->AttitudeEstimate, (float*)&q_gyro, 4, 1);
        DSP_QT_Normalize_f32(&data->AttitudeEstimate, &data->AttitudeEstimate);

        // P = (I - K * H) * P
        float KH[16];
        DSP_Matrix_Multiply_f32(KH, K, 4, 3, H, 4);
        DSP_Matrix_Scale_f32(KH, KH, -1.0f, 4, 4);
        KH[0]  += 1.0f;
        KH[5]  += 1.0f;
        KH[10] += 1.0f;
        KH[15] += 1.0f;

        DSP_Matrix_Multiply_f32(filter->_P, KH, 4, 4, P_hat, 4);
    }
}

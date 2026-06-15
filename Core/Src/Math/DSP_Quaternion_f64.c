#include "DSP_Constants.h"
#include "DSP_Quaternion.h"

#include <math.h>

#define QT_90DEG_SINGULARITY_THRESHOLD 0.2

double DSP_QT_Norm_f64(const DSP_Quaternion_f64* q)
{
    return sqrt(q->r * q->r + q->i * q->i + q->j * q->j + q->k * q->k);
}

double DSP_QT_Normalize_f64(DSP_Quaternion_f64* dst, const DSP_Quaternion_f64* q)
{
    const double sq_norm = q->r * q->r + q->i * q->i + q->j * q->j + q->k * q->k;

    if(sq_norm < (DSP_NORM_EPSILON_F64 * DSP_NORM_EPSILON_F64))
    {
        dst->r = 0.0;
        dst->i = 0.0;
        dst->j = 0.0;
        dst->k = 0.0;
        return 0.0;
    }
    const double norm     = sqrt(sq_norm);
    const double inv_norm = 1.0 / norm;

    dst->r = q->r * inv_norm;
    dst->i = q->i * inv_norm;
    dst->j = q->j * inv_norm;
    dst->k = q->k * inv_norm;

    return norm;
}

void DSP_QT_RotateVector_f64(double* dst, const double* v, const DSP_Quaternion_f64* q)
{
    const double x = v[0];
    const double y = v[1];
    const double z = v[2];

    const double r = q->r;
    const double i = q->i;
    const double j = q->j;
    const double k = q->k;

    const double rr = r * r;
    const double ii = i * i;
    const double jj = j * j;
    const double kk = k * k;

    dst[0] = x * ii + 2.0 * y * i * j + 2.0 * z * i * k - x * jj + 2.0 * z * j * r - x * kk - 2.0 * y * k * r + x * rr;
    dst[1] = -y * ii + 2.0 * x * i * j - 2.0 * z * i * r + y * jj + 2.0 * z * j * k - y * kk + 2 * x * k * r + y * rr;
    dst[2] = -z * ii + 2.0 * x * i * k + 2 * y * i * r - z * jj + 2.0 * y * j * k - 2 * x * j * r + z * kk + z * rr;
}

void DSP_QT_RotateVectorInv_f64(double* dst, const double* v, const DSP_Quaternion_f64* q)
{
    const double x = v[0];
    const double y = v[1];
    const double z = v[2];

    const double r = q->r;
    const double i = -q->i;
    const double j = -q->j;
    const double k = -q->k;

    const double rr = r * r;
    const double ii = i * i;
    const double jj = j * j;
    const double kk = k * k;

    dst[0] = x * ii + 2.0 * y * i * j + 2.0 * z * i * k - x * jj + 2.0 * z * j * r - x * kk - 2.0 * y * k * r + x * rr;
    dst[1] = -y * ii + 2.0 * x * i * j - 2.0 * z * i * r + y * jj + 2.0 * z * j * k - y * kk + 2 * x * k * r + y * rr;
    dst[2] = -z * ii + 2.0 * x * i * k + 2 * y * i * r - z * jj + 2.0 * y * j * k - 2 * x * j * r + z * kk + z * rr;
}

void DSP_QT_EulerAngles_f64(double* eulerAngles, const DSP_Quaternion_f64* q)
{
    const double sing = q->i * q->j + q->k * q->r;

    if(sing > QT_90DEG_SINGULARITY_THRESHOLD)
    {
        eulerAngles[2] = 2.0 * atan2(q->i, q->r);
        eulerAngles[1] = PI_F64 / 2.0;
        eulerAngles[0] = 0.0;
        return;
    }
    if(sing < -QT_90DEG_SINGULARITY_THRESHOLD)
    {
        eulerAngles[2] = -2.0 * atan2(q->i, q->r);
        eulerAngles[1] = PI_F64 / 2.0;
        eulerAngles[0] = 0.0;
        return;
    }

    const double sq_qt = 2.0 * (q->r * q->j - q->i * q->k);

    eulerAngles[0] = atan2(2.0 * (q->r * q->i + q->j * q->k), 1 - 2 * (q->i * q->i + q->j * q->j));
    eulerAngles[1] = -PI_F64 / 2.0 + 2.0 * atan2(sqrt(1.0 + sq_qt), sqrt(1.0 - sq_qt));
    eulerAngles[2] = atan2(2.0 * (q->r * q->k + q->i * q->j), 1.0 - 2.0 * (q->j * q->j + q->k * q->k));
}

void DSP_QT_EulerToQuaternion_f64(DSP_Quaternion_f64* q, const double* eulerAngles)
{
    const double cosX = cos(eulerAngles[0] * 0.5);
    const double sinX = sin(eulerAngles[0] * 0.5);
    const double cosY = cos(eulerAngles[1] * 0.5);
    const double sinY = sin(eulerAngles[1] * 0.5);
    const double cosZ = cos(eulerAngles[2] * 0.5);
    const double sinZ = sin(eulerAngles[2] * 0.5);

    q->r = cosX * cosY * cosZ + sinX * sinY * sinZ;
    q->i = sinX * cosY * cosZ - cosX * sinY * sinZ;
    q->j = cosX * sinY * cosZ + sinX * cosY * sinZ;
    q->k = cosX * cosY * sinZ - sinX * sinY * cosZ;
}

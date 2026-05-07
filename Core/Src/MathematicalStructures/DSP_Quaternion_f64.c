#include "DSP_Constants.h"
#include "DSP_Quaternion.h"

#include <math.h>

#define QT_90DEG_SINGULARITY_THRESHOLD 0.2

void DSP_QT_Add_f64(DSP_Quaternion_f64* dst, const DSP_Quaternion_f64* q1, const DSP_Quaternion_f64* q2)
{
    dst->r = q1->r + q2->r;
    dst->i = q1->i + q2->i;
    dst->j = q1->j + q2->j;
    dst->k = q1->k + q2->k;
}

void DSP_QT_Subtract_f64(DSP_Quaternion_f64* dst, const DSP_Quaternion_f64* q1, const DSP_Quaternion_f64* q2)
{
    dst->r = q1->r - q2->r;
    dst->i = q1->i - q2->i;
    dst->j = q1->j - q2->j;
    dst->k = q1->k - q2->k;
}

void DSP_QT_Multiply_f64(DSP_Quaternion_f64* dst, const DSP_Quaternion_f64* q1, const DSP_Quaternion_f64* q2)
{
    const double a1 = q1->r;
    const double b1 = q1->i;
    const double c1 = q1->j;
    const double d1 = q1->k;

    const double a2 = q2->r;
    const double b2 = q2->i;
    const double c2 = q2->j;
    const double d2 = q2->k;

    dst->r = a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2;
    dst->i = a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2;
    dst->j = a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2;
    dst->k = a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2;
}

void DSP_QT_Scale_f64(DSP_Quaternion_f64* dst, const DSP_Quaternion_f64* q, double s)
{
    dst->r = s * q->r;
    dst->i = s * q->i;
    dst->j = s * q->j;
    dst->k = s * q->k;
}

void DSP_QT_Conjugate_f64(DSP_Quaternion_f64* dst, const DSP_Quaternion_f64* q)
{
    dst->r = q->r;
    dst->i = -q->i;
    dst->j = -q->j;
    dst->k = -q->k;
}

void DSP_QT_Normalize_f64(DSP_Quaternion_f64* dst, const DSP_Quaternion_f64* q)
{
    const double norm = sqrt(q->r * q->r + q->i * q->i + q->j * q->j + q->k * q->k);

    dst->r = q->r / norm;
    dst->i = q->i / norm;
    dst->j = q->j / norm;
    dst->k = q->k / norm;
}

double DSP_QT_Norm_f64(const DSP_Quaternion_f64* q)
{
    return sqrt(q->r * q->r + q->i * q->i + q->j * q->j + q->k * q->k);
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

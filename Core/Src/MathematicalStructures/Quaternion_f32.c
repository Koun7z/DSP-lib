//
// Created by pwoli on 18.03.2025.
//

#include "DSP_Constants.h"
#include "Quaternion.h"

#include <math.h>

#define QT_90DEG_SINGULARITY_THRESHOLD 0.2f

void DSP_QT_Add_f32(DSP_Quaternion_f32* dst, const DSP_Quaternion_f32* q1, const DSP_Quaternion_f32* q2)
{
	dst->r = q1->r + q2->r;
	dst->i = q1->i + q2->i;
	dst->j = q1->j + q2->j;
	dst->k = q1->k + q2->k;
}

void DSP_QT_Subtract_f32(DSP_Quaternion_f32* dst, const DSP_Quaternion_f32* q1, const DSP_Quaternion_f32* q2)
{
	dst->r = q1->r - q2->r;
	dst->i = q1->i - q2->i;
	dst->j = q1->j - q2->j;
	dst->k = q1->k - q2->k;
}

void DSP_QT_Multiply_f32(DSP_Quaternion_f32* dst, const DSP_Quaternion_f32* q1, const DSP_Quaternion_f32* q2)
{
	const float a1 = q1->r;
	const float b1 = q1->i;
	const float c1 = q1->j;
	const float d1 = q1->k;

	const float a2 = q2->r;
	const float b2 = q2->i;
	const float c2 = q2->j;
	const float d2 = q2->k;

	dst->r = a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2;
	dst->i = a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2;
	dst->j = a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2;
	dst->k = a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2;
}

void DSP_QT_Scale_f32(DSP_Quaternion_f32* dst, const DSP_Quaternion_f32* q, float s)
{
	dst->r = s * q->r;
	dst->i = s * q->i;
	dst->j = s * q->j;
	dst->k = s * q->k;
}

void DSP_QT_Conjugate_f32(DSP_Quaternion_f32* dst, const DSP_Quaternion_f32* q)
{
	dst->r = q->r;
	dst->i = -q->i;
	dst->j = -q->j;
	dst->k = -q->k;
}

void DSP_QT_Normalize_f32(DSP_Quaternion_f32* dst, const DSP_Quaternion_f32* q)
{
	const float norm = sqrtf(q->r * q->r + q->i * q->i + q->j * q->j + q->k * q->k);

	dst->r = q->r / norm;
	dst->i = q->i / norm;
	dst->j = q->j / norm;
	dst->k = q->k / norm;
}

float DSP_QT_Norm_f32(const DSP_Quaternion_f32* q)
{
	return sqrtf(q->r * q->r + q->i * q->i + q->j * q->j + q->k * q->k);
}

void DSP_QT_RotateVector_f32(float* dst, const float* v, const DSP_Quaternion_f32* q)
{
	const float x = v[0];
	const float y = v[1];
	const float z = v[2];

	const float r = q->r;
	const float i = q->i;
	const float j = q->j;
	const float k = q->k;

	const float rr = q->r * q->r;
	const float ii = q->i * q->i;
	const float jj = q->j * q->j;
	const float kk = q->k * q->k;

	dst[0] =
	  x * ii + 2.0f * y * i * j + 2.0f * z * i * k - x * jj + 2.0f * z * j * r - x * kk - 2.0f * y * k * r + x * rr;
	dst[1] =
	  -y * ii + 2.0f * x * i * j - 2.0f * z * i * r + y * jj + 2.0f * z * j * k - y * kk + 2 * x * k * r + y * rr;
	dst[2] = -z * ii + 2.0f * x * i * k + 2 * y * i * r - z * jj + 2.0f * y * j * k - 2 * x * j * r + z * kk + z * rr;
}

void DSP_QT_EulerAngles_f32(float* eulerAngles, const DSP_Quaternion_f32* q)
{
	const float sing = q->i * q->j + q->k * q->r;

	if(sing > QT_90DEG_SINGULARITY_THRESHOLD)
	{
		eulerAngles[2] = 2.0f * atan2f(q->i, q->r);
		eulerAngles[1] = PI_F32 / 2.0f;
		eulerAngles[0] = 0.0f;
		return;
	}
	if(sing < -QT_90DEG_SINGULARITY_THRESHOLD)
	{
		eulerAngles[2] = -2.0f * atan2f(q->i, q->r);
		eulerAngles[1] = PI_F32 / 2.0f;
		eulerAngles[0] = 0.0f;
		return;
	}

	const float sq_qt = 2.0f * (q->r * q->j - q->i * q->k);

	eulerAngles[0] = atan2f(2.0f * (q->r * q->i + q->j * q->k), 1 - 2 * (q->i * q->i + q->j * q->j));
	eulerAngles[1] = -PI_F32 / 2.0f + 2.0f * atan2f(sqrtf(1.0f + sq_qt), sqrtf(1.0f - sq_qt));
	eulerAngles[2] = atan2f(2.0f * (q->r * q->k + q->i * q->j), 1.0f - 2.0f * (q->j * q->j + q->k * q->k));
}

void DSP_QT_EulerToQuaternion_f32(DSP_Quaternion_f32* q, const float* eulerAngles)
{
	const float cosX = cosf(eulerAngles[0] * 0.5f);
	const float sinX = sinf(eulerAngles[0] * 0.5f);
	const float cosY = cosf(eulerAngles[1] * 0.5f);
	const float sinY = sinf(eulerAngles[1] * 0.5f);
	const float cosZ = cosf(eulerAngles[2] * 0.5f);
	const float sinZ = sinf(eulerAngles[2] * 0.5f);

	q->r = cosX * cosY * cosZ + sinX * sinY * sinZ;
	q->i = sinX * cosY * cosZ - cosX * sinY * sinZ;
	q->j = cosX * sinY * cosZ + sinX * cosY * sinZ;
	q->k = cosX * cosY * sinZ - sinX * sinY * cosZ;
}

//
// Created by pwoli on 18.03.2025.
//

#include "DSP_Constants.h"
#include "Quaternion.h"

#include <math.h>

#define QT_SINGULARITY_THRESHOLD 0.2f

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

void DSP_QT_Scale_f32(DSP_Quaternion_f32* dst, const DSP_Quaternion_f32* q, float scale)
{
	dst->r = scale * q->r;
	dst->i = scale * q->i;
	dst->j = scale * q->j;
	dst->k = scale * q->k;
}

void DSP_QT_Conjugate_f32(DSP_Quaternion_f32* dst, const DSP_Quaternion_f32* q)
{
	dst->r = q->r;
	dst->i = -q->i;
	dst->j = -q->j;
	dst->k = -q->k;
}

void DSP_QT_Normalized_f32(DSP_Quaternion_f32* dst, const DSP_Quaternion_f32* q)
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

void DSP_QT_EulerAngles_f32(float* eulerAngles, const DSP_Quaternion_f32* q)
{
	const float sing = q->i * q->j + q->k * q->r;

	if (sing > QT_SINGULARITY_THRESHOLD)
	{
		eulerAngles[2] = 2 * atan2f(q->i, q->r);
		eulerAngles[1] = PI_F32 / 2;
		eulerAngles[0] = 0;
	}
	else if (sing < -QT_SINGULARITY_THRESHOLD)
	{
		eulerAngles[2] = -2 * atan2f(q->i, q->r);
		eulerAngles[1] = PI_F32 / 2;
		eulerAngles[0] = 0;
	}

	eulerAngles[0] = atan2f(2 * (q->r * q->i + q->j * q->k), 1 - 2 * (q->i * q->i + q->j * q->j));
	eulerAngles[1] =
	  -PI_F32 / 2 + 2 * atan2(sqrtf(1 + 2 * (q->r * q->j - q->i * q->k)), sqrtf(1 - 2 * (q->r * q->j + q->i * q->k)));
	eulerAngles[2] = atan2(2 * (q->r * q->k + q->i * q->j), 1 - 2 * (q->j * q->j + q->k * q->k));
}

void DSP_QT_EulerToQuaternion(DSP_Quaternion_f32* q, const float* eulerAngles)
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

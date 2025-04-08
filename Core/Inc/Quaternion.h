//
// Created by pwoli on 18.03.2025.
//

#ifndef QUATERNION_H
#define QUATERNION_H

typedef struct
{
	float r;
	float i;
	float j;
	float k;
} DSP_Quaternion_f32;

void DSP_QT_Add_f32(DSP_Quaternion_f32* dst, const DSP_Quaternion_f32* q1, const DSP_Quaternion_f32* q2);

void DSP_QT_Subtract_f32(DSP_Quaternion_f32* dst,const DSP_Quaternion_f32* q1,const DSP_Quaternion_f32* q2);

void DSP_QT_Multiply_f32(DSP_Quaternion_f32* dst,const DSP_Quaternion_f32* q1,const DSP_Quaternion_f32* q2);

void DSP_QT_Scale_f32(DSP_Quaternion_f32* dst,const DSP_Quaternion_f32* q,float scale);

void DSP_QT_Conjugate_f32(DSP_Quaternion_f32* dst,const DSP_Quaternion_f32* q);

void DSP_QT_Normalized_f32(DSP_Quaternion_f32* dst,const DSP_Quaternion_f32* q);

float DSP_QT_Norm_f32(const DSP_Quaternion_f32* q);


void DSP_QT_EulerAngles_f32(float* eulerAngles, const DSP_Quaternion_f32* q);

void DSP_QT_EulerToQuaternion(DSP_Quaternion_f32* q, const float* eulerAngles);
#endif //QUATERNION_H

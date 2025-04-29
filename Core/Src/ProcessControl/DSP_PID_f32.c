//
// Created by pwoli on 28.04.2025.
//
#include <float.h>
#include <ProcessControl.h>

static float clamp(const float x, const float min, const float max)
{
	const float t = x < min ? min : x;
	return t > max ? max : t;
}


void DSP_PID_Init_f32(DSP_PID_Instance_f32* regulator,
                      const float Kp,
                      const float Ti,
                      const float Td,
                      const float N,
                      const float Ts)
{
	regulator->K = Kp;

	regulator->Kd[0] = Td / (Td + N * Ts);
	regulator->Kd[1] = Kp * N * regulator->Kd[0];

	regulator->Ki = Kp * Ts / Ti;

	regulator->K_SetPointRatio = 1.0f;
	regulator->D_SetPointRatio = 0.0f;
	regulator->Int_Rst_T       = 0.0f;
	regulator->OutMax          = FLT_MAX;
	regulator->OutMin          = -FLT_MAX;
}

float DSP_PID_Update_f32(DSP_PID_Instance_f32* regulator, const float sp, const float pv)
{
	// P
	float out = regulator->K * (regulator->K_SetPointRatio * sp - pv);

	// D
	const float diff =
	  regulator->Kd[0] * regulator->_prevDiff
	  - regulator->Kd[1] * (pv - regulator->_prevPV - regulator->D_SetPointRatio * (sp - regulator->_prevSP));

	out                += diff + regulator->_integralState;
	const float out_lim = clamp(out, regulator->OutMin, regulator->OutMax);

	// I
	regulator->_integralState += regulator->Ki * (sp - pv) + regulator->Int_Rst_T * (out_lim - out);

	regulator->_prevPV   = pv;
	regulator->_prevSP   = sp;
	regulator->_prevDiff = diff;

	return out_lim;
}

void DSP_PID_SetGains_f32(DSP_PID_Instance_f32* regulator,
                          const float Kp,
                          const float Ti,
                          const float Td,
                          const float N,
                          const float Ts)
{
	regulator->K = Kp;

	regulator->Kd[0] = Td / (Td + N * Ts);
	regulator->Kd[1] = Kp * N * regulator->Kd[0];

	regulator->Ki = Kp * Ts / Ti;
}

void DSP_PID_SetSaturation_f32(DSP_PID_Instance_f32* regulator,
                               const float min,
                               const float max,
                               const float Tt,
                               const float Ts)
{
	regulator->Int_Rst_T       = Ts / Tt;
	regulator->OutMax          = max;
	regulator->OutMin          = min;
}

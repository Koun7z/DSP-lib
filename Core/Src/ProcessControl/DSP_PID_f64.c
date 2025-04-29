//
// Created by pwoli on 29.04.2025.
//
#include <float.h>
#include <ProcessControl.h>

static double clamp(const double x, const double min, const double max)
{
	const double t = x < min ? min : x;
	return t > max ? max : t;
}


void DSP_PID_Init_f64(DSP_PID_Instance_f32* regulator,
                      const double Kp,
                      const double Ti,
                      const double Td,
                      const double N,
                      const double Ts)
{
	regulator->K = Kp;

	regulator->Kd[0] = Td / (Td + N * Ts);
	regulator->Kd[1] = Kp * N * regulator->Kd[0];

	regulator->Ki = Kp * Ts / Ti;

	regulator->K_SetPointRatio = 1.0;
	regulator->D_SetPointRatio = 0.0;
	regulator->Int_Rst_T       = 0.0;
	regulator->OutMax          = DBL_MAX;
	regulator->OutMin          = -DBL_MAX;
}

double DSP_PID_Update_f64(DSP_PID_Instance_f32* regulator, const double sp, const double pv)
{
	// P
	double out = regulator->K * (regulator->K_SetPointRatio * sp - pv);

	// D
	const double diff =
	  regulator->Kd[0] * regulator->_prevDiff
	  - regulator->Kd[1] * (pv - regulator->_prevPV - regulator->D_SetPointRatio * (sp - regulator->_prevSP));

	out                += diff + regulator->_integralState;
	const double out_lim = clamp(out, regulator->OutMin, regulator->OutMax);

	// I
	regulator->_integralState += regulator->Ki * (sp - pv) + regulator->Int_Rst_T * (out_lim - out);

	regulator->_prevPV   = pv;
	regulator->_prevSP   = sp;
	regulator->_prevDiff = diff;

	return out_lim;
}

void DSP_PID_SetGains_f64(DSP_PID_Instance_f32* regulator,
                          const double Kp,
                          const double Ti,
                          const double Td,
                          const double N,
                          const double Ts)
{
	regulator->K = Kp;

	regulator->Kd[0] = Td / (Td + N * Ts);
	regulator->Kd[1] = Kp * N * regulator->Kd[0];

	regulator->Ki = Kp * Ts / Ti;
}


void DSP_PID_SetSaturation_f64(DSP_PID_Instance_f32* regulator,
                               const double min,
                               const double max,
                               const double Tt,
                               const double Ts)
{
	regulator->Int_Rst_T       = Ts / Tt;
	regulator->OutMax          = max;
	regulator->OutMin          = min;
}
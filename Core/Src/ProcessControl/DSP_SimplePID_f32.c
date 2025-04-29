//
// Created by pwoli on 28.04.2025.
//

#include "ProcessControl.h"

void DSP_SimplePID_Init_f32(DSP_SimplePID_Instance_f32* regulator,
                            const float Kp,
                            const float Ti,
                            const float Td,
                            const float Ts)
{
	const float Ki = (Ti != 0.0f) ? Kp / Ti : 0.0f;
	const float Kd = Kp * Td;

	// PID coeffs
	regulator->A[0] = Kp + Ki * Ts + Kd / Ts;
	regulator->A[1] = -Kp - 2*Kd/Ts;
	regulator->A[2] = Kd / Ts;

	// Zero out the past samples

	regulator->_prevErr[0] = 0.0f;
	regulator->_prevErr[1] = 0.0f;
}

float DPS_SimplePID_Update_f32(DSP_SimplePID_Instance_f32* regulator, const float err)
{
	// Update PID
	regulator->output = err * regulator->A[0] + regulator->_prevErr[0] * regulator->A[1] + regulator->_prevErr[1] * regulator->A[2];

	// Shift past samples
	regulator->_prevErr[1] = regulator->_prevErr[0];
	regulator->_prevErr[0] = err;

	return regulator->output;
}

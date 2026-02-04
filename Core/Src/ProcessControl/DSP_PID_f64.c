//
// Created by pwoli on 29.04.2025.
//

#include "DSP_ProcessControl.h"
#include "DSP_Utils.h"


#include <float.h>


void DSP_PID_Init_f64(DSP_PID_Instance_f64* regulator,
                      const double Kp,
                      const double Ti,
                      const double Td,
                      const double N,
                      const double Ts)
{
    regulator->K = Kp;

    regulator->Kd[0] = Td / (Td + N * Ts);
    regulator->Kd[1] = Kp * N * regulator->Kd[0];

    regulator->Ki = Ti != 0.0 ? Kp * Ts / Ti : 0.0;

    regulator->K_SP_Ratio = 1.0;
    regulator->D_SP_Ratio = 0.0;
    regulator->Int_Rst_T  = 0.0;
    regulator->OutMax     = DBL_MAX;
    regulator->OutMin     = -DBL_MAX;
}

double DSP_PID_Update_f64(DSP_PID_Instance_f64* regulator, const double sp, const double pv)
{
    // P
    double out = regulator->K * (regulator->K_SP_Ratio * sp - pv);

    // D
    const double diff = regulator->Kd[0] * regulator->_prevDiff
                      - regulator->Kd[1]
                          * (pv - regulator->_prevPV - regulator->D_SP_Ratio * (sp - regulator->_prevSP));

    out                 += diff + regulator->_integralState;
    const double out_lim = DSP_Clamp_f64(out, regulator->OutMin, regulator->OutMax);

    // I
    regulator->_integralState += regulator->Ki * (sp - pv) + regulator->Int_Rst_T * (out_lim - out);

    regulator->_prevPV   = pv;
    regulator->_prevSP   = sp;
    regulator->_prevDiff = diff;

    return out_lim;
}

void DSP_PID_SetGains_f64(DSP_PID_Instance_f64* regulator,
                          const double Kp,
                          const double Ti,
                          const double Td,
                          const double N,
                          const double Ts)
{
    regulator->K = Kp;

    regulator->Kd[0] = Td / (Td + N * Ts);
    regulator->Kd[1] = Kp * N * regulator->Kd[0];

    regulator->Ki = Ti != 0.0 ? Kp * Ts / Ti : 0.0;
}


void DSP_PID_SetSaturation_f64(DSP_PID_Instance_f64* regulator,
                               const double min,
                               const double max,
                               const double Tt,
                               const double Ts)
{
    regulator->Int_Rst_T = Tt == 0.0 ? 0.0 : Ts / Tt;
    regulator->OutMax    = max;
    regulator->OutMin    = min;
}

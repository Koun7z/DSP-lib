/*
 * ProcessControl.h
 *
 *  Created on: Jan 25, 2025
 *      Author: Piotr Woliński
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdbool.h>

#include "SignalFiltering.h"

#define ANTI_WINDUP_ENABLE 1

/**
 * @brief PID instance holding parameters and internal regulator state as single precision floating point values
 */
typedef struct
{
	// PID Config
	float K_Gain;
	float I_Gain;
	float D_Gain;
	float MaxWindup;
	float MinWindup;

	// PID State
	float _integralState;
	float _previousInput;
	float _previousInputFiltered;
} PID_Instance_f32;

/**
 * @brief PID instance holding parameters and internal regulator state as double precision floating point values
 */
typedef struct
{
	// PID Config
	double K_Gain;
	double I_Gain;
	double D_Gain;
	double MaxWindup;
	double MinWindup;

	// PID State
	double _integralState;
	double _previousInput;
	double _previousInputFiltered;
} PID_Instance_f64;

/**
 * @brief	    Initializes PID internal structure.
 *				The Kp, Ti and Td are parameters of the PID controller in the Standard Form.
 * @param[out] *regulator PID Instance to initialize
 * @param[in]   Kp		  proportional gain
 * @param[in]   Ti		  integration time constant
 * @param[in]   Td		  differentiation time constant
 */
void PID_Init_f32(PID_Instance_f32* regulator, float Kp, float Ti, float Td);

/**
 * @brief       Sets minimum and maximum values of the internal integral state
 * @param[out]  regulator  PID Instance
 * @param[in]   minWindup  minimal allowed integral value
 * @param[in]   maxWindup  maximal allowed integral value
 */
void PID_SetAntiWindup_f32(PID_Instance_f32* regulator, float minWindup, float maxWindup);

/**
 * @brief		  Calculates PID response based on given input and updates internal regulator state
 * @param[inout] *regulator  Initialized PID instance
 * @param[in]     input      control error
 * @param[in]     dt         sampling period (time between two update calls)
 * @return 	      float      PID output
 */
float PID_Update_f32(PID_Instance_f32* regulator, float input, float dt);

/**
 * @brief		  Calculates PID response based on given input and updates internal regulator state.
 *				  Performs additional input filtering (using FIR filter) before calculating differential response,
 *				  to reduce the effect of high frequency noise amplification.
 * @param[inout] *regulator  Initialized PID instance
 * @param[inout] *filter	 Initialized FIR filter instance
 * @param[in]     input      control error
 * @param[in]     dt         sampling period (time between two update calls)
 * @return 	      float      PID output
 */
float PID_DTermFIR_Update_f32(PID_Instance_f32* regulator, DSP_FIR_RT_Instance_f32* filter, float input, float dt);

/**
 * @brief		  Calculates PID response based on given input and updates internal regulator state.
 *				  Performs additional input filtering (using IIR filter) before calculating differential response,
 *				  to reduce the effect of high frequency noise amplification.
 * @param[inout] *regulator  Initialized PID instance
 * @param[inout] *filter	 Initialized IIR filter instance
 * @param[in]     input      control error
 * @param[in]     dt         sampling period (time between two update calls)
 * @return 	      float      PID output
 */
float PID_DTermIIR_Update_f32(PID_Instance_f32* regulator, DSP_IIR_RT_Instance_f32* filter, float input, float dt);


/**
 * @brief	    Initializes PID internal structure.
 *				The Kp, Ti and Td are parameters of the PID controller in the Standard Form.
 * @param[out] *regulator PID Instance to initialize
 * @param[in]   Kp		  proportional gain
 * @param[in]   Ti		  integration time constant
 * @param[in]   Td		  differentiation time constant
 */
void PID_Init_f64(PID_Instance_f64* regulator, double Kp, double Ti, double Td);

/**
 * @brief       Sets minimum and maximum values of the internal integral state
 * @param[out]  regulator  PID Instance
 * @param[in]   minWindup  minimal allowed integral value
 * @param[in]   maxWindup  maximal allowed integral value
 */
void PID_SetAntiWindup_f64(PID_Instance_f64* regulator, double minWindup, double maxWindup);

/**
 * @brief		  Calculates PID response based on given input and updates internal regulator state
 * @param[inout] *regulator  Initialized PID instance
 * @param[in]     input      control error
 * @param[in]     dt         sampling period (time between two update calls)
 * @return 	      float      PID output
 */
double PID_Update_f64(PID_Instance_f64* regulator, double input, double dt);

/**
 * @brief		  Calculates PID response based on given input and updates internal regulator state.
 *				  Performs additional input filtering (using FIR filter) before calculating differential response,
 *				  to reduce the effect of high frequency noise amplification.
 * @param[inout] *regulator  Initialized PID instance
 * @param[inout] *filter	 Initialized FIR filter instance
 * @param[in]     input      control error
 * @param[in]     dt         sampling period (time between two update calls)
 * @return 	      float      PID output
 */
double PID_DTermFIR_Update_f64(PID_Instance_f64* regulator, DSP_FIR_RT_Instance_f64* filter, double input, double dt);

/**
 * @brief		  Calculates PID response based on given input and updates internal regulator state.
 *				  Performs additional input filtering (using IIR filter) before calculating differential response,
 *				  to reduce the effect of high frequency noise amplification.
 * @param[inout] *regulator  Initialized PID instance
 * @param[inout] *filter	 Initialized IIR filter instance
 * @param[in]     input      control error
 * @param[in]     dt         sampling period (time between two update calls)
 * @return 	      float      PID output
 */
double PID_DTermIIR_Update_f64(PID_Instance_f64* regulator, DSP_IIR_RT_Instance_f64* filter, double input, double dt);

#endif /* INC_PID_H_ */
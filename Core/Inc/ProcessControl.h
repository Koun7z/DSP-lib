/*
** ProcessControl.h
**
**  Created on: Jan 25, 2025
**      Author: Piotr Woliński
*/

#ifndef INC_PID_H_
#define INC_PID_H_

/**
 * @brief A simple, IIR based PID instance
 */
typedef struct
{
	// PID Config
	float A[3];

	// PID State
	float _prevErr[2];
	float output;
} DSP_SimplePID_Instance_f32;

/**
 * @breif  PID instance holding its parameters and internal state.
 *		   [K/D]_SetPointRatio are the only parameters that should be modified directly.
 *		   All other parameters should be modified through provided Set functions.
 */
typedef struct
{
	// PID Parameters
	float K;                // Proportional gain
	float Ki;               // Integral gain
	float Kd[2];            // Differential gain
	float K_SetPointRatio;  // Proportional SP ratio (default = 1)
	float D_SetPointRatio;  // Differential SP ratio (default = 0)
	float OutMax;           // Maximum output
	float OutMin;           // Minimum output
	float Int_Rst_T;        // Integral windup reset time

	// PID State
	float _integralState;  // Current integral value
	float _prevDiff;       // Previous differential output
	float _prevPV;         // Previous process value
	float _prevSP;         // Previous process value
} DSP_PID_Instance_f32;

/**
 * @breif  Double precision PID instance holding its parameters and internal state.
 *		   [K/D]_SetPointRatio are the only parameters that should be modified directly.
 *		   All other parameters should be modified through provided Set functions.
 */
typedef struct
{
	// PID Parameters
	double K;                // Proportional gain
	double Ki;               // Integral gain
	double Kd[2];            // Differential gain
	double K_SetPointRatio;  // Proportional SP ratio (default = 1)
	double D_SetPointRatio;  // Differential SP ratio (default = 0)
	double OutMax;           // Maximum output
	double OutMin;           // Minimum output
	double Int_Rst_T;        // Integral windup reset time

	// PID State
	double _integralState;  // Current integral value
	double _prevDiff;       // Previous differential output
	double _prevPV;         // Previous process value
	double _prevSP;         // Previous process value
} DSP_PID_Instance_f64;


/**
 * @brief	    Initializes a simple, barebone PID internal structure.
 *				The Kp, Ti and Td are parameters of the PID controller in the Standard Form.
 * @param[out] *regulator PID Instance to initialize
 * @param[in]   Kp		  Proportional gain
 * @param[in]   Ti		  Integration time constant
 * @param[in]   Td		  Differentiation time constant
 * @param[in]   Ts		  Fixed sample time
 */
void DSP_SimplePID_Init_f32(DSP_SimplePID_Instance_f32* regulator, float Kp, float Ti, float Td, float Ts);

/**
 * @brief		  Calculates PID response treating the regulator as a 2-nd order IIR filter
 * @param[inout] *regulator  Initialized PID instance
 * @param[in]     input      control error
 * @param[in]     dt         sampling period (time between two update calls)
 * @return 	      float      PID output
 */
float DSP_SimplePID_Update_f32(DSP_SimplePID_Instance_f32* regulator, float err);

/**
 * @brief	    Initializes internal PID structure.
 *				The Kp, Ti and Td are PID parameters in the Standard Form.
 * @param[out] *regulator PID Instance to initialize
 * @param[in]   Kp		  Proportional gain
 * @param[in]   Ti		  Integration time constant
 * @param[in]   Td		  Differentiation time constant
 * @param[in]	N		  D-term filter constant
 * @param[in]   Ts		  Fixed sampling time
 */
void DSP_PID_Init_f32(DSP_PID_Instance_f32* regulator, float Kp, float Ti, float Td, float N, float Ts);

/**
 * @brief		  Calculates PID response based on given input and updates internal regulator state
 * @param[inout] *regulator     Initialized PID instance
 * @param[in]     setPoint      Process set point
 * @param[in]     processValue  Value of controlled process output
 * @return 	      float         PID output
 */
float DSP_PID_Update_f32(DSP_PID_Instance_f32* regulator, float setPoint, float processValue);

/**
 * @brief	    Updates PID parameters without resetting the internal state.
 *				The Kp, Ti and Td are PID parameters in the Standard Form.
 * @param[out] *regulator  Initialized PID instance
 * @param[in]   Kp		   Proportional gain
 * @param[in]   Ti		   Integration time constant
 * @param[in]   Td		   Differentiation time constant
 * @param[in]	N		   D-term filter constant
 * @param[in]   Ts		   Fixed sampling time
 */
void DSP_PID_SetGains_f32(DSP_PID_Instance_f32* regulator, float Kp, float Ti, float Td, float N, float Ts);

/**
 * @brief       Sets the output and integral term saturation
 * @param[out] *regulator  Initialized PID instance
 * @param[in]   min		   Minimum PID output
 * @param[in]   max        Maximum PID output
 * @param[in]   Tt		   Integral windup reset time constant
 * @param[in]   Ts		   Fixed sampling time
 */
void DSP_PID_SetSaturation_f32(DSP_PID_Instance_f32* regulator, float min, float max, float Tt, float Ts);


/*
** Double implementation
*/

/**
 * @brief	    Initializes internal PID structure.
 *				The Kp, Ti and Td are PID parameters in the Standard Form.
 * @param[out] *regulator PID Instance to initialize
 * @param[in]   Kp		  Proportional gain
 * @param[in]   Ti		  Integration time constant
 * @param[in]   Td		  Differentiation time constant
 * @param[in]	N		  D-term filter constant
 * @param[in]   Ts		  Fixed sampling time
 */
void DSP_PID_Init_f64(DSP_PID_Instance_f64* regulator, double Kp, double Ti, double Td, double N, double Ts);

/**
 * @brief		  Calculates PID response based on given input and updates internal regulator state
 * @param[inout] *regulator     Initialized PID instance
 * @param[in]     setPoint      Process set point
 * @param[in]     processValue  Value of controlled process output
 * @return 	      float         PID output
 */
double DSP_PID_Update_f64(DSP_PID_Instance_f64* regulator, double setPoint, double processValue);

/**
 * @brief	    Updates PID parameters without resetting the internal state.
 *				The Kp, Ti and Td are PID parameters in the Standard Form.
 * @param[out] *regulator  Initialized PID instance
 * @param[in]   Kp		   Proportional gain
 * @param[in]   Ti		   Integration time constant
 * @param[in]   Td		   Differentiation time constant
 * @param[in]	N		   D-term filter constant
 * @param[in]   Ts		   Fixed sampling time
 */
void DSP_PID_SetGains_f64(DSP_PID_Instance_f64* regulator, double Kp, double, double Td, double N, double Ts);

/**
 * @brief       Sets the output and integral term saturation
 * @param[out] *regulator  Initialized PID instance
 * @param[in]   min		   Minimum PID output
 * @param[in]   max        Maximum PID output
 * @param[in]   Tt		   Integral windup reset time constant
 * @param[in]   Ts		   Fixed sampling time
 */
void DSP_PID_SetSaturation_f64(DSP_PID_Instance_f64* regulator, double min, double max, double Tt, double Ts);
#endif /* INC_PID_H_ */

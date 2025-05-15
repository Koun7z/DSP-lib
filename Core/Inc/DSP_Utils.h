//
// Created by pwoli on 04.05.2025.
//

#ifndef DSP_UTILS_H
#define DSP_UTILS_H
#include <math.h>

static inline float DSP_Clamp_f32(const float x, const float min, const float max)
{
	const float t = x < min ? min : x;
	return t > max ? max : t;
}

static inline double DSP_Clamp_f64(const double x , const double min, const double max)
{
	const double t = x < min ? min : x;
	return t > max ? max : t;
}

static inline float DSP_DeadZone_f32(const float x,  const float deadZone)
{
	return fabsf(x) < deadZone ? 0.0f : x;
}

static inline double DSP_DeadZone_f64(const double x,  const double deadZone)
{
	return fabs(x) < deadZone ? 0.0 : x;
}

#endif //DSP_UTILS_H

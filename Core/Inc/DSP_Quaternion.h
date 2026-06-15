//
// Created by pwoli on 18.03.2025.
//

#ifndef QUATERNION_H
#define QUATERNION_H

/**
 * @brief Structure representing a quaternion q = (r, i, j, k):
 *		  - r       field denotes the real part
 *		  - i, j, k fields denote the corresponding imaginary parts
 */
typedef union
{
    struct
    {
        float r;
        float i;
        float j;
        float k;
    };
    float q[4];
} DSP_Quaternion_f32;

/**
 * @brief		Performs quaternion addition operation and puts the result into dst output parameter.
 *
 *				dst = q1 + q2
 *
 * @param[out] *dst  Operation output
 * @param[in]  *q1   First input quaternion
 * @param[in]  *q2   Second input quaternion
 */
static inline void DSP_QT_Add_f32(DSP_Quaternion_f32* dst, const DSP_Quaternion_f32* q1, const DSP_Quaternion_f32* q2)
{
    dst->r = q1->r + q2->r;
    dst->i = q1->i + q2->i;
    dst->j = q1->j + q2->j;
    dst->k = q1->k + q2->k;
}

/**
 * @brief		Performs quaternion subtraction operation and puts the result into dst output parameter.
 *
 *				dst = q1 - q2
 *
 * @param[out] *dst  Operation output
 * @param[in]  *q1   First input quaternion
 * @param[in]  *q2   Second input quaternion
 */
static inline void DSP_QT_Subtract_f32(DSP_Quaternion_f32* dst,
                                       const DSP_Quaternion_f32* q1,
                                       const DSP_Quaternion_f32* q2)
{
    dst->r = q1->r - q2->r;
    dst->i = q1->i - q2->i;
    dst->j = q1->j - q2->j;
    dst->k = q1->k - q2->k;
}

/**
 * @brief		Performs quaternion multiplication (Hamilton product)
 *				operation and puts the result into dst output parameter.
 *
 *				dst = q1 * q2
 *
 * @param[out] *dst  Operation output
 * @param[in]  *q1   First input quaternion
 * @param[in]  *q2   Second input quaternion
 */
static inline void DSP_QT_Multiply_f32(DSP_Quaternion_f32* dst,
                                       const DSP_Quaternion_f32* q1,
                                       const DSP_Quaternion_f32* q2)
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

/**
 * @brief		Performs multiplication of given quaternion by given scalar value
 *				and puts the result in dst output parameter.
 *
 *				dst = s * q
 *
 * @param[out] *dst  Operation output
 * @param[in]  *q    Input quaternion
 * @param[in]   s    Scalar value
 */
static inline void DSP_QT_Scale_f32(DSP_Quaternion_f32* dst, const DSP_Quaternion_f32* q, float s)
{
    dst->r = s * q->r;
    dst->i = s * q->i;
    dst->j = s * q->j;
    dst->k = s * q->k;
}

/**
 * @brief		Performs quaternion conjugation operation and puts the result into dst output parameter.
 *
 *				dst = conj(q)
 *
 * @param[out] *dst  Operation output
 * @param[in]  *q    Input quaternion
 */
static inline void DSP_QT_Conjugate_f32(DSP_Quaternion_f32* dst, const DSP_Quaternion_f32* q)
{
    dst->r = q->r;
    dst->i = -q->i;
    dst->j = -q->j;
    dst->k = -q->k;
}

/**
 * @brief	   Calculates the quaternion norm as the square root of the product of a quaternion with its conjugate.
 *
 *			   dst = ||q|| = sqrt(q * conj(q))
 *
 * @param[in] *q      Input quaternion
 * @retval     float  Quaternion norm
 */
float DSP_QT_Norm_f32(const DSP_Quaternion_f32* q);

/**
 * @brief		Performs quaternion normalization and puts the resulting unit quaternion into dst output parameter.
 *
 *				dst = q / ||q||
 *
 * @param[out] *dst  Operation output
 * @param[in]  *q    Input quaternion
 * @retval     float  Quaternion norm
 */
float DSP_QT_Normalize_f32(DSP_Quaternion_f32* dst, const DSP_Quaternion_f32* q);

/**
 * @brief		Rotates a 3d vector v by a unit quaternion q.
 *
 *			    - Both dst and v are 3 element float arrays
 *
 * @param[out] *dst  Rotated output vector
 * @param[in]  *v    Input vector
 * @param[in]  *q	 Input unit quaternion
 */
void DSP_QT_RotateVector_f32(float* dst, const float* v, const DSP_Quaternion_f32* q);

/**
 * @brief		Rotates a 3d vector v by the inverse of a unit quaternion q.
 *
 *			    - Both dst and v are 3 element float arrays
 *
 * @param[out] *dst  Rotated output vector
 * @param[in]  *v    Input vector
 * @param[in]  *q	 Input unit quaternion
 */
void DSP_QT_RotateVectorInv_f32(float* dst, const float* v, const DSP_Quaternion_f32* q);

/**
 * @brief	    Converts a unit quaternion, representing a rotation in 3d space
 *				into Euler angles representation in Tait-Bryan convention,
 *				using the body 3-2-1 sequence <=> lab 1-2-3 sequence.
 *
 *				-eulerAngles is a 3 element float array
 *				-eulerAngles = [Roll (x), Pitch (y), Yaw (z)]
 *
 * @param[out] *eulerAngles  Roll, Pitch and Yaw as Euler angles in radians
 * @param[in]  *q			 Input unit quaternion representing the rotation
 */
void DSP_QT_EulerAngles_f32(float* eulerAngles, const DSP_Quaternion_f32* q);

/**
 * @brief		Converts a rotation described in Euler angels, in Tait-Bryan convention
 *				into a unit quatetnion representation using
 *				Body 3-2-1 sequence <=> Lab 1-2-3 sequence
 *
 *				-eulerAngles is a 3 element float array
 *				-eulerAngles = [Roll (x), Pitch (y), Yaw (z)]
 *
 * @param[out] *q            Rotation representation in unit quaternion
 * @param[in]  *eulerAngles  Roll, Pitch and Yaw as float array
 */
void DSP_QT_EulerToQuaternion_f32(DSP_Quaternion_f32* q, const float* eulerAngles);

/*
** Double precision versions of the above functions
*/

/**
 * @brief Structure representing a quaternion q = (r, i, j, k):
 *		  - r       field denotes the real part
 *		  - i, j, k fields denote the corresponding imaginary parts
 */
typedef union
{
    struct
    {
        double r;
        double i;
        double j;
        double k;
    };
    double q[4];
} DSP_Quaternion_f64;

/**
 * @brief		Performs quaternion addition operation and puts the result into dst output parameter.
 *
 *				dst = q1 + q2
 *
 * @param[out] *dst  Operation output
 * @param[in]  *q1   First input quaternion
 * @param[in]  *q2   Second input quaternion
 */
static inline void DSP_QT_Add_f64(DSP_Quaternion_f64* dst, const DSP_Quaternion_f64* q1, const DSP_Quaternion_f64* q2)
{
    dst->r = q1->r + q2->r;
    dst->i = q1->i + q2->i;
    dst->j = q1->j + q2->j;
    dst->k = q1->k + q2->k;
}

/**
 * @brief		Performs quaternion subtraction operation and puts the result into dst output parameter.
 *
 *				dst = q1 - q2
 *
 * @param[out] *dst  Operation output
 * @param[in]  *q1   First input quaternion
 * @param[in]  *q2   Second input quaternion
 */
static inline void DSP_QT_Subtract_f64(DSP_Quaternion_f64* dst,
                                       const DSP_Quaternion_f64* q1,
                                       const DSP_Quaternion_f64* q2)
{
    dst->r = q1->r - q2->r;
    dst->i = q1->i - q2->i;
    dst->j = q1->j - q2->j;
    dst->k = q1->k - q2->k;
}

/**
 * @brief		Performs quaternion multiplication (Hamilton product)
 *				operation and puts the result into dst output parameter.
 *
 *				dst = q1 * q2
 *
 * @param[out] *dst  Operation output
 * @param[in]  *q1   First input quaternion
 * @param[in]  *q2   Second input quaternion
 */
static inline void DSP_QT_Multiply_f64(DSP_Quaternion_f64* dst,
                                       const DSP_Quaternion_f64* q1,
                                       const DSP_Quaternion_f64* q2)
{
    const double a1 = q1->r;
    const double b1 = q1->i;
    const double c1 = q1->j;
    const double d1 = q1->k;

    const double a2 = q2->r;
    const double b2 = q2->i;
    const double c2 = q2->j;
    const double d2 = q2->k;

    dst->r = a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2;
    dst->i = a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2;
    dst->j = a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2;
    dst->k = a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2;
}

/**
 * @brief		Performs multiplication of given quaternion by given scalar value
 *				and puts the result in dst output parameter.
 *
 *				dst = s * q
 *
 * @param[out] *dst  Operation output
 * @param[in]  *q    Input quaternion
 * @param[in]   s    Scalar value
 */
static inline void DSP_QT_Scale_f64(DSP_Quaternion_f64* dst, const DSP_Quaternion_f64* q, double s)
{
    dst->r = s * q->r;
    dst->i = s * q->i;
    dst->j = s * q->j;
    dst->k = s * q->k;
}

/**
 * @brief		Performs quaternion conjugation operation and puts the result into dst output parameter.
 *
 *				dst = conj(q)
 *
 * @param[out] *dst  Operation output
 * @param[in]  *q    Input quaternion
 */
static inline void DSP_QT_Conjugate_f64(DSP_Quaternion_f64* dst, const DSP_Quaternion_f64* q)
{
    dst->r = q->r;
    dst->i = -q->i;
    dst->j = -q->j;
    dst->k = -q->k;
}

/**
 * @brief		Performs quaternion normalization and puts the resulting unit quaternion into dst output parameter.
 *
 *				dst = q / ||q||
 *
 * @param[out] *dst   Operation output
 * @param[in]  *q     Input quaternion
 * @retval     float  Quaternion norm
 */
double DSP_QT_Normalize_f64(DSP_Quaternion_f64* dst, const DSP_Quaternion_f64* q);

/**
 * @brief	   Calculates the quaternion norm as the square root of the product of a quaternion with its conjugate.
 *
 *			   dst = ||q|| = sqrt(q * conj(q))
 *
 * @param[in] *q      Input quaternion
 * @retval     float  Quaternion norm
 */
double DSP_QT_Norm_f64(const DSP_Quaternion_f64* q);

/**
 * @brief		Rotates a 3d vector v by a unit quaternion q.
 *
 *			    - Both dst and v are 3 element float arrays
 *
 * @param[out] *dst  Rotated output vector
 * @param[in]  *v    Input vector
 * @param[in]  *q	 Input unit quaternion
 */
void DSP_QT_RotateVector_f64(double* dst, const double* v, const DSP_Quaternion_f64* q);

/**
 * @brief		Rotates a 3d vector v by the inverse of a unit quaternion q.
 *
 *			    - Both dst and v are 3 element float arrays
 *
 * @param[out] *dst  Rotated output vector
 * @param[in]  *v    Input vector
 * @param[in]  *q	 Input unit quaternion
 */
void DSP_QT_RotateVectorInv_f64(double* dst, const double* v, const DSP_Quaternion_f64* q);

/**
 * @brief	    Converts a unit quaternion, representing a rotation in 3d space
 *				into Euler angles representation in Tait-Bryan convention,
 *				using the body 3-2-1 sequence <=> lab 1-2-3 sequence.
 *
 *				-eulerAngles is a 3 element float array
 *				-eulerAngles = [Roll (x), Pitch (y), Yaw (z)]
 *
 * @param[out] *eulerAngles  Roll, Pitch and Yaw as Euler angles in radians
 * @param[in]  *q			 Input unit quaternion representing the rotation
 */
void DSP_QT_EulerAngles_f64(double* eulerAngles, const DSP_Quaternion_f64* q);

/**
 * @brief		Converts a rotation described in Euler angels, in Tait-Bryan convention
 *				into a unit quatetnion representation using
 *				Body 3-2-1 sequence <=> Lab 1-2-3 sequence
 *
 *				-eulerAngles is a 3 element float array
 *				-eulerAngles = [Roll (x), Pitch (y), Yaw (z)]
 *
 * @param[out] *q            Rotation representation in unit quaternion
 * @param[in]  *eulerAngles  Roll, Pitch and Yaw as float array
 */
void DSP_QT_EulerToQuaternion_f64(DSP_Quaternion_f64* q, const double* eulerAngles);

#endif  // QUATERNION_H

#ifndef DSP_MATRIX_H__
#define DSP_MATRIX_H__

#include <stddef.h>

/**
 * @brief result = A + B
 *
 * @param result
 * @param a
 * @param b
 * @param rows
 * @param cols
 */
void DSP_Matrix_Add_f32(float* __restrict result, const float* A, const float* B, size_t row, size_t col);

/**
 * @brief result += A
 *
 * @param result
 * @param A
 * @param rows
 * @param cols
 */
void DSP_Matrix_AddInline_f32(float* __restrict result, const float* A, size_t rows, size_t cols);

/**
 * @brief result = A - B
 *
 * @param result
 * @param a
 * @param b
 * @param rows
 * @param cols
 */
void DSP_Matrix_Subtract_f32(float* __restrict result, const float* A, const float* B, size_t rows, size_t cols);

/**
 * @brief result -= A
 *
 * @param result
 * @param A
 * @param rows
 * @param cols
 */
void DSP_Matrix_SubtractInline_f32(float* result, const float* A, size_t rows, size_t cols);

/**
 * @brief result = A * B
 *
 * @param result
 * @param A
 * @param B
 */
void DSP_Matrix_Multiply_f32(float* __restrict result,
                             const float* A,
                             size_t rowsA,
                             size_t colsA,
                             const float* B,
                             size_t colsB);

/**
 * @brief result = sM
 *
 * @param result
 * @param M
 * @param scalar
 * @param dim
 */
void DSP_Matrix_Scale_f32(float* __restrict result, const float* M, float s, size_t rows, size_t cols);

/**
 * @brief result *= s;
 *
 * @param result
 * @param s
 * @param dim
 */
void DSP_Matrix_ScaleInline_f32(float* result, float s, size_t rows, size_t cols);

/**
 * @brief result = A .* B (Element-wise multiplication)
 *
 * @param result
 * @param A
 * @param B
 */
void DSP_Matrix_MultiplyElems_f32(float* __restrict result, const float* A, const float* B, size_t rows, size_t cols);

/**
 * @brief result = result .* A (Element-wise multiplication)
 *
 * @param result
 * @param A
 * @param dim
 */
void DSP_Matrix_MultiplyElemsInline_f32(float* result, const float* A, size_t rows, size_t cols);

/**
 * @brief result = A^T
 *
 * @param result
 * @param M
 * @param dim
 */
void DSP_Matrix_Transpose_f32(float* __restrict result, const float* M, size_t rows, size_t cols);

/**
 * @brief M = M^T (In-place transpose for N x N matrix)
 *
 * @param M
 * @param dim
 */
void DSP_Matrix_TransposeInline_f32(float* M, size_t dim);

float DSP_Matrix_Determinant_f32(const float* matrix, size_t rows, size_t cols);

#endif
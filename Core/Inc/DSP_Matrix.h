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
void DSP_Matrix_AddInplace_f32(float* __restrict result, const float* A, size_t rows, size_t cols);

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
void DSP_Matrix_SubtractInplace_f32(float* result, const float* A, size_t rows, size_t cols);

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
 * @brief Performs an operation of the form result = A * B * A^T.
 *        A is a M x N matrix, B is a N x N matrix, and result is a M x M matrix.
 *
 * @param result
 * @param A
 * @param B
 * @param M
 * @param N
 */
void DSP_Matrix_SandwichMultiply_f32(float* __restrict result, const float* A, const float* B, size_t M, size_t N);

/**
 * @brief result = sA
 *
 * @param result
 * @param A
 * @param s
 * @param rows
 * @param cols
 */
void DSP_Matrix_Scale_f32(float* __restrict result, const float* A, float s, size_t rows, size_t cols);

/**
 * @brief result *= s;
 *
 * @param result
 * @param s
 * @param rows
 * @param cols
 */
void DSP_Matrix_ScaleInplace_f32(float* result, float s, size_t rows, size_t cols);

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
 * @param rows
 * @param cols
 */
void DSP_Matrix_MultiplyElemsInplace_f32(float* result, const float* A, size_t rows, size_t cols);

/**
 * @brief result = A^T
 *
 * @param result
 * @param M
 * @param rows
 * @param cols
 */
void DSP_Matrix_Transpose_f32(float* __restrict result, const float* M, size_t rows, size_t cols);

/**
 * @brief M = M^T (In-place transpose for N x N matrix)
 *
 * @param A
 * @param N
 */
void DSP_Matrix_TransposeInplace_f32(float* A, size_t N);


static inline void DSP_Matrix_SwapRows_f32(float* A, size_t r1, size_t r2, size_t cols)
{
    float* a = A + r1 * cols;
    float* b = A + r2 * cols;

    for(size_t k = 0; k < cols; k++)
    {
        float tmp = a[k];
        a[k]      = b[k];
        b[k]      = tmp;
    }
}

/**
 * @brief LUP Decomposition. This function decomposes the matrix A into L and U factors with partial pivoting.
 *        Decomposition is done in-place, so the input matrix A will be overwritten with the L and U factors.
 *        The permutation vector P records the row swaps.
 *        The last element of P (P[N]) counts the number of row swaps for determinant calculation.
 *
 * @param[inout] *A   - Square matrix to decompose, will contain L and U factors (L has unit diagonal elements)
 * @param[in]     N   - dimension of the matrix
 * @param[in]     tol - small tolerance number to detect if the matrix is a degenerate
 * @param[out]    P   - permutation vector of size N + 1
 * @return        int
 *                    - 0 on success
 *                    - 1 if the matrix is near degenerate
 */
int DSP_Matrix_LUPDecompose_f32(float* __restrict A, size_t N, float tol, size_t* __restrict P);

/**
 * @brief Solve for Ax = b using the LUP decomposition of A.
 *        The function assumes that A has already been decomposed into L and U factors using DSP_Matrix_LUPDecompose_f32,
 *
 * @param[in]  *LU - Decomposed matrix containing L and U factors
 * @param[in]  *P - Permutation vector from LUP decomposition
 * @param[out] *x - Result vector to store the solution of size N
 * @param[in]  *b - Right-hand side vector of size N
 * @param[in]   N - Dimension of the matrix
 */
void DSP_Matrix_LUPSolve_f32(const float* LU, const size_t* P, float* __restrict x, const float* b, size_t N);

/**
 * @brief Solve for a solution of xA = b using the LUP decomposition of A.
 *        The function assumes that A has already been decomposed into L and U factors using DSP_Matrix_LUPDecompose_f32.
 *        The cache locality of this function is terrible, so you may be better off by just solving for A^Tx^T = b^T manually ¯\_(ツ)_/¯
 *
 * @param[in]  *LU - Decomposed matrix containing L and U factors
 * @param[in]  *P  - Row permutation vector from LUP decomposition
 * @param[out] *x  - Result vector to store the solution of size N
 * @param[in]  *b  - Right-hand side vector of size N
 * @param[in]   N  - Dimension of the matrix
 */
void DSP_Matrix_LUPRightSolve_f32(const float* LU, const size_t* P, float* __restrict x, const float* b, size_t N);

/**
 * @brief Compute the inverse of a matrix using its LUP decomposition.
 *
 * @param[in]   *LU - Decomposed matrix containing L and U factors
 * @param[in]   *P - Permutation vector from LUP decomposition
 * @param[in]    N - Dimension of the matrix
 * @param[out] *IA - Output matrix to store the inverse
 */
void DSP_Matrix_LUPInvert_f32(float* __restrict IA, const float* LU, const size_t* P, size_t N);

/**
 * @brief Compute the determinant of a matrix using its LUP decomposition.
 *
 * @param[in]   *LU - Decomposed matrix containing L and U factors
 * @param[in]   *P - Permutation vector from LUP decomposition
 * @param[in]    N - Dimension of the matrix
 * @return float - The determinant of the initial matrix
 */
float DSP_Matrix_LUPDeterminant_f32(const float* LU, const size_t* P, size_t N);

/*
** Double precision versions
*/

/**
 * @brief result = A + B
 *
 * @param result
 * @param a
 * @param b
 * @param rows
 * @param cols
 */
void DSP_Matrix_Add_f64(double* __restrict result, const double* A, const double* B, size_t row, size_t col);

/**
 * @brief result += A
 *
 * @param result
 * @param A
 * @param rows
 * @param cols
 */
void DSP_Matrix_AddInplace_f64(double* __restrict result, const double* A, size_t rows, size_t cols);

/**
 * @brief result = A - B
 *
 * @param result
 * @param a
 * @param b
 * @param rows
 * @param cols
 */
void DSP_Matrix_Subtract_f64(double* __restrict result, const double* A, const double* B, size_t rows, size_t cols);

/**
 * @brief result -= A
 *
 * @param result
 * @param A
 * @param rows
 * @param cols
 */
void DSP_Matrix_SubtractInplace_f64(double* __restrict result, const double* A, size_t rows, size_t cols);

/**
 * @brief result = A * B
 *
 * @param result
 * @param A
 * @param B
 */
void DSP_Matrix_Multiply_f64(double* __restrict result,
                             const double* A,
                             size_t rowsA,
                             size_t colsA,
                             const double* B,
                             size_t colsB);

/**
 * @brief Performs an operation of the form result = A * B * A^T.
 *        A is a M x N matrix, B is a N x N matrix, and result is a M x M matrix.
 *
 * @param result
 * @param A
 * @param B
 * @param M
 * @param N
 */
void DSP_Matrix_SandwichMultiply_f64(double* __restrict result, const double* A, const double* B, size_t M, size_t N);

/**
 * @brief result = sA
 *
 * @param result
 * @param A
 * @param s
 * @param rows
 * @param cols
 */
void DSP_Matrix_Scale_f64(double* __restrict result, const double* A, double s, size_t rows, size_t cols);

/**
 * @brief result *= s;
 *
 * @param result
 * @param s
 * @param rows
 * @param cols
 */
void DSP_Matrix_ScaleInplace_f64(double* __restrict result, double s, size_t rows, size_t cols);

/**
 * @brief result = A .* B (Element-wise multiplication)
 *
 * @param result
 * @param A
 * @param B
 */
void DSP_Matrix_MultiplyElems_f64(double* __restrict result,
                                  const double* A,
                                  const double* B,
                                  size_t rows,
                                  size_t cols);

/**
 * @brief result = result .* A (Element-wise multiplication)
 *
 * @param result
 * @param A
 * @param rows
 * @param cols
 */
void DSP_Matrix_MultiplyElemsInplace_f64(double* __restrict result, const double* A, size_t rows, size_t cols);

/**
 * @brief result = A^T
 *
 * @param result
 * @param M
 * @param rows
 * @param cols
 */
void DSP_Matrix_Transpose_f64(double* __restrict result, const double* M, size_t rows, size_t cols);

/**
 * @brief M = M^T (In-place transpose for N x N matrix)
 *
 * @param A
 * @param N
 */
void DSP_Matrix_TransposeInplace_f64(double* A, size_t N);


static inline void DSP_Matrix_SwapRows_f64(double* A, size_t r1, size_t r2, size_t cols)
{
    double* a = A + r1 * cols;
    double* b = A + r2 * cols;

    for(size_t k = 0; k < cols; k++)
    {
        double tmp = a[k];
        a[k]       = b[k];
        b[k]       = tmp;
    }
}

/**
 * @brief LUP Decomposition. This function decomposes the matrix A into L and U factors with partial pivoting.
 *        Decomposition is done in-place, so the input matrix A will be overwritten with the L and U factors.
 *        The permutation vector P records the row swaps.
 *        The last element of P (P[N]) counts the number of row swaps for determinant calculation.
 *
 * @param[inout] *A   - Square matrix to decompose, will contain L and U factors (L has unit diagonal elements)
 * @param[in]     N   - dimension of the matrix
 * @param[in]     tol - small tolerance number to detect if the matrix is a degenerate
 * @param[out]    P   - permutation vector of size N + 1
 * @return        int
 *                    - 0 on success
 *                    - 1 if the matrix is near degenerate
 */
int DSP_Matrix_LUPDecompose_f64(double* __restrict A, size_t N, double tol, size_t* __restrict P);

/**
 * @brief Solve for Ax = b using the LUP decomposition of A.
 *        The function assumes that A has already been decomposed into L and U factors using DSP_Matrix_LUPDecompose_f64,
 *
 * @param[in]  *LU - Decomposed matrix containing L and U factors
 * @param[in]  *P - Permutation vector from LUP decomposition
 * @param[out] *x - Result vector to store the solution of size N
 * @param[in]  *b - Right-hand side vector of size N
 * @param[in]   N - Dimension of the matrix
 */
void DSP_Matrix_LUPSolve_f64(const double* LU, const size_t* P, double* __restrict x, const double* b, size_t N);

/**
 * @brief Solve for a solution of xA = b using the LUP decomposition of A.
 *        The function assumes that A has already been decomposed into L and U factors using DSP_Matrix_LUPDecompose_f64.
 *        The cache locality of this function is terrible, so you may be better off by just solving for A^Tx^T = b^T manually ¯\_(ツ)_/¯
 *
 * @param[in]  *LU - Decomposed matrix containing L and U factors
 * @param[in]  *P  - Row permutation vector from LUP decomposition
 * @param[out] *x  - Result vector to store the solution of size N
 * @param[in]  *b  - Right-hand side vector of size N
 * @param[in]   N  - Dimension of the matrix
 */
void DSP_Matrix_LUPRightSolve_f64(const double* LU, const size_t* P, double* __restrict x, const double* b, size_t N);

/**
 * @brief Compute the inverse of a matrix using its LUP decomposition.
 *
 * @param[in]   *LU - Decomposed matrix containing L and U factors
 * @param[in]   *P - Permutation vector from LUP decomposition
 * @param[in]    N - Dimension of the matrix
 * @param[out] *IA - Output matrix to store the inverse
 */
void DSP_Matrix_LUPInvert_f64(double* __restrict IA, const double* LU, const size_t* P, size_t N);

/**
 * @brief Compute the determinant of a matrix using its LUP decomposition.
 *
 * @param[in]   *LU - Decomposed matrix containing L and U factors
 * @param[in]   *P - Permutation vector from LUP decomposition
 * @param[in]    N - Dimension of the matrix
 * @return double - The determinant of the initial matrix
 */
double DSP_Matrix_LUPDeterminant_f64(const double* LU, const size_t* P, size_t N);

#endif
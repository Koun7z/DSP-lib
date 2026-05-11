#include "DSP_Matrix.h"

#include <string.h>
#include <math.h>

void DSP_Matrix_Add_f64(double* __restrict result, const double* A, const double* B, size_t rows, size_t cols)
{
    for(size_t i = 0; i < rows * cols; i++)
    {
        result[i] = A[i] + B[i];
    }
}

void DSP_Matrix_AddInplace_f64(double* __restrict result, const double* A, size_t rows, size_t cols)
{
    for(size_t i = 0; i < rows * cols; i++)
    {
        result[i] += A[i];
    }
}

void DSP_Matrix_Subtract_f64(double* __restrict result, const double* A, const double* B, size_t rows, size_t cols)
{
    for(size_t i = 0; i < rows * cols; i++)
    {
        result[i] = A[i] - B[i];
    }
}
void DSP_Matrix_SubtractInplace_f64(double* result, const double* A, size_t rows, size_t cols)
{
    for(size_t i = 0; i < rows * cols; i++)
    {
        result[i] -= A[i];
    }
}

void DSP_Matrix_Multiply_f64(double* __restrict result,
                             const double* A,
                             size_t rowsA,
                             size_t colsA,
                             const double* B,
                             size_t colsB)
{
    for(size_t i = 0; i < rowsA; i++)
    {
        for(size_t j = 0; j < colsB; j++)
        {
            double sum = 0.0;

            for(size_t k = 0; k < colsA; k++)
            {
                sum += A[i * colsA + k] * B[k * colsB + j];
            }

            result[i * colsB + j] = sum;
        }
    }
}

void DSP_Matrix_SandwichMultiply_f64(double* __restrict result, const double* A, const double* B, size_t M, size_t N)
{
    for(size_t i = 0; i < M; i++)
    {
        for(size_t j = 0; j < M; j++)
        {
            double sum_ij = 0.0;

            for(size_t k = 0; k < N; k++)
            {
                double AB_ik = 0.0;
                for(size_t t = 0; t < N; t++)
                {
                    AB_ik += A[i * N + t] * B[t * N + k];
                }

                sum_ij += AB_ik * A[j * N + k];
            }

            result[i * M + j] = sum_ij;
        }
    }
}

void DSP_Matrix_Scale_f64(double* __restrict result, const double* A, double s, size_t rows, size_t cols)
{
    for(size_t i = 0; i < rows * cols; i++)
    {
        result[i] = A[i] * s;
    }
}

void DSP_Matrix_ScaleInplace_f64(double* result, double s, size_t rows, size_t cols)
{
    for(size_t i = 0; i < rows * cols; i++)
    {
        result[i] *= s;
    }
}

void DSP_Matrix_MultiplyElems_f64(double* __restrict result, const double* A, const double* B, size_t rows, size_t cols)
{
    for(size_t i = 0; i < rows * cols; i++)
    {
        result[i] = A[i] * B[i];
    }
}

void DSP_Matrix_MultiplyElemsInplace_f64(double* result, const double* A, size_t rows, size_t cols)
{
    for(size_t i = 0; i < rows * cols; i++)
    {
        result[i] *= A[i];
    }
}

void DSP_Matrix_Transpose_f64(double* __restrict result, const double* M, size_t rows, size_t cols)
{
    for(size_t i = 0; i < rows; i++)
    {
        for(size_t j = 0; j < cols; j++)
        {
            result[j * rows + i] = M[i * cols + j];
        }
    }
}

void DSP_Matrix_TransposeInplace_f64(double* A, size_t N)
{
    for(size_t i = 0; i < N; i++)
    {
        for(size_t j = 0; j < i; j++)
        {
            double temp  = A[i * N + j];
            A[i * N + j] = A[j * N + i];
            A[j * N + i] = temp;
        }
    }
}

int DSP_Matrix_LUPDecompose_f64(double* __restrict A, size_t N, double tol, size_t* __restrict P)
{
    size_t i_max;

    for(size_t i = 0; i < N; i++)
    {
        P[i] = i;
    }
    P[N] = 0;

    for(size_t i = 0; i < N; i++)
    {
        double max_A = 0.0;

        i_max = i;

        for(size_t k = i; k < N; k++)
        {
            double abs_A = fabs(A[k * N + i]);
            if(abs_A > max_A)
            {
                max_A = abs_A;
                i_max = k;
            }
        }

        if(max_A < tol)
        {
            return 1;  // failure, matrix is degenerate
        }

        if(i_max != i)
        {
            const size_t tmp = P[i];
            P[i]             = P[i_max];
            P[i_max]         = tmp;

            DSP_Matrix_SwapRows_f64(A, i, i_max, N);

            // counting pivots
            P[N]++;
        }

        for(size_t j = i + 1; j < N; j++)
        {
            A[j * N + i] /= A[i * N + i];

            for(size_t k = i + 1; k < N; k++)
            {
                A[j * N + k] -= A[j * N + i] * A[i * N + k];
            }
        }
    }

    return 0;
}

void DSP_Matrix_LUPSolve_f64(const double* LU, const size_t* P, double* __restrict x, const double* b, size_t N)
{
    for(size_t i = 0; i < N; i++)
    {
        x[i] = b[P[i]];

        for(size_t k = 0; k < i; k++)
        {
            x[i] -= LU[i * N + k] * x[k];
        }
    }

    for(int i = N - 1; i >= 0; i--)
    {
        for(size_t k = i + 1; k < N; k++)
        {
            x[i] -= LU[i * N + k] * x[k];
        }

        x[i] /= LU[i * N + i];
    }
}

void DSP_Matrix_LUPRightSolve_f64(const double* LU, const size_t* P, double* __restrict x, const double* b, size_t N)
{
    for(size_t j = 0; j < N; j++)
    {
        x[P[j]] = b[j];
        for(size_t i = 0; i < j; i++)
        {
            x[P[j]] -= x[P[i]] * LU[i * N + j];
        }
        x[P[j]] /= LU[j * N + j];
    }

    for(int j = N - 2; j >= 0; j--)
    {
        for(size_t i = j + 1; i < N; i++)
        {
            x[P[j]] -= x[P[i]] * LU[i * N + j];
        }
    }
}

void DSP_Matrix_LUPInvert_f64(double* __restrict IA, const double* LU, const size_t* P, size_t N)
{
    for(size_t j = 0; j < N; j++)
    {
        for(size_t i = 0; i < N; i++)
        {
            IA[i * N + j] = (P[i] == j ? 1.0 : 0.0);

            for(size_t k = 0; k < i; k++)
            {
                IA[i * N + j] -= LU[i * N + k] * IA[k * N + j];
            }
        }

        for(int i = N - 1; i >= 0; i--)
        {
            for(size_t k = i + 1; k < N; k++)
            {
                IA[i * N + j] -= LU[i * N + k] * IA[k * N + j];
            }

            IA[i * N + j] /= LU[i * N + i];
        }
    }
}

double DSP_Matrix_LUPDeterminant_f64(const double* LU, const size_t* P, size_t N)
{
    double det = LU[0 * N + 0];

    for(size_t i = 1; i < N; i++)
    {
        det *= LU[i * N + i];
    }

    return P[N] % 2 == 0 ? det : -det;
}
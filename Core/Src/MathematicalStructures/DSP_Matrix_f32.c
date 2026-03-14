#include "DSP_Matrix.h"

#include <string.h>


void DSP_Matrix_Add_f32(float* __restrict result, const float* A, const float* B, size_t rows, size_t cols)
{
    for(size_t i = 0; i < rows * cols; i++)
    {
        result[i] = A[i] + B[i];
    }
}

void DSP_Matrix_AddInline_f32(float* __restrict result, const float* A, size_t rows, size_t cols)
{
    for(size_t i = 0; i < rows * cols; i++)
    {
        result[i] += A[i];
    }
}

void DSP_Matrix_Subtract_f32(float* __restrict result, const float* A, const float* B, size_t rows, size_t cols)
{
    for(size_t i = 0; i < rows * cols; i++)
    {
        result[i] = A[i] - B[i];
    }
}
void DSP_Matrix_SubtractInline_f32(float* result, const float* A, size_t rows, size_t cols)
{
    for(size_t i = 0; i < rows * cols; i++)
    {
        result[i] -= A[i];
    }
}

void DSP_Matrix_Multiply_f32(float* __restrict result,
                             const float* A,
                             size_t rowsA,
                             size_t colsA,
                             const float* B,
                             size_t colsB)
{
    for(size_t i = 0; i < rowsA; i++)
    {
        for(size_t j = 0; j < colsB; j++)
        {
            float sum = 0.0f;

            for(size_t k = 0; k < colsA; k++)
            {
                sum += A[i * colsA + k] * B[k * colsB + j];
            }

            result[i * colsB + j] = sum;
        }
    }
}

void DSP_Matrix_Scale_f32(float* __restrict result, const float* M, float s, size_t rows, size_t cols)
{
    for(size_t i = 0; i < rows * cols; i++)
    {
        result[i] = M[i] * s;
    }
}

void DSP_Matrix_ScaleInline_f32(float* result, float s, size_t rows, size_t cols)
{
    for(size_t i = 0; i < rows * cols; i++)
    {
        result[i] *= s;
    }
}

void DSP_Matrix_MultiplyElems_f32(float* __restrict result, const float* A, const float* B, size_t rows, size_t cols)
{
    for(size_t i = 0; i < rows * cols; i++)
    {
        result[i] = A[i] * B[i];
    }
}

void DSP_Matrix_MultiplyElemsInline_f32(float* result, const float* A, size_t rows, size_t cols)
{
    for(size_t i = 0; i < rows * cols; i++)
    {
        result[i] *= A[i];
    }
}

void DSP_Matrix_Transpose_f32(float* result, const float* M, size_t rows, size_t cols)
{
    for(size_t i = 0; i < rows; i++)
    {
        for(size_t j = 0; j < cols; j++)
        {
            result[j * rows + i] = M[i * cols + j];
        }
    }
}

void DSP_Matrix_TransposeInline_f32(float* M, size_t dim)
{
    for(size_t i = 0; i < dim; i++)
    {
        for(size_t j = 0; j < i; j++)
        {
            float temp     = M[i * dim + j];
            M[i * dim + j] = M[j * dim + i];
            M[j * dim + i] = temp;
        }
    }
}

float DSP_Matrix_Determinant_f32(const float* matrix, size_t rows, size_t cols)
{
    return 0.0f;  // Placeholder for determinant calculation
}
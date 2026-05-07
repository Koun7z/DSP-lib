#include "DSP_Matrix.h"

#include "test_registry.h"

#include <stdio.h>

START_TEST(multiplication_test)
{
    float A[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    float B[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};

    float C[4] = {22.0f, 28.0f, 49.0f, 64.0f};

    float result[4] = {0.0f};
    DSP_Matrix_Multiply_f32(result, A, 2, 3, B, 2);

    for(int i = 0; i < 4; i++)
    {
        ck_assert_float_eq_tol(result[i], C[i], 1e-6f);
    }
}
END_TEST

START_TEST(sandwich_multiplication_test)
{
    size_t M = 4;
    size_t N = 3;

    float A[12]      = {1.0f, 2.0f, 3.0f, 8.0f, 1.0f, 7.0f, 3.0f, 7.0f, 2.0f, 9.0f, 5.0f, 8.0f};
    float result[16] = {106.0f, 301.0f, 143.0f, 369.0f, 301.0f, 1015.0f, 394.0f, 1186.0f,
                        143.0f, 394.0f, 235.0f, 509.0f, 369.0f, 1186.0f, 509.0f, 1415.0f};

    float B[9] = {5.0f, 0.0f, 3.0f, 0.0f, 2.0f, 1.0f, 3.0f, 1.0f, 7.0f};

    float ans[16];

    DSP_Matrix_SandwichMultiply_f32(ans, A, B, M, N);

    for(int i = 0; i < 16; i++)
    {
        ck_assert_float_eq_tol(ans[i], result[i], 1e-6f);
    }
}
END_TEST

START_TEST(addition_test)
{
    float A[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    float B[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};

    float C[6] = {2.0f, 4.0f, 6.0f, 8.0f, 10.0f, 12.0f};

    float result[6] = {0.0f};
    DSP_Matrix_Add_f32(result, A, B, 2, 3);

    for(int i = 0; i < 6; i++)
    {
        ck_assert_float_eq_tol(result[i], C[i], 1e-6f);
    }
}
END_TEST

START_TEST(addition_inline_test)
{
    float A[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    float B[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};

    float C[6] = {2.0f, 4.0f, 6.0f, 8.0f, 10.0f, 12.0f};

    DSP_Matrix_AddInplace_f32(A, B, 2, 3);

    for(int i = 0; i < 6; i++)
    {
        ck_assert_float_eq_tol(A[i], C[i], 1e-6f);
    }
}
END_TEST

START_TEST(subtraction_test)
{
    float A[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    float B[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};

    float C[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    float result[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    DSP_Matrix_Subtract_f32(result, A, B, 2, 3);

    for(int i = 0; i < 6; i++)
    {
        ck_assert_float_eq_tol(result[i], C[i], 1e-6f);
    }
}

START_TEST(subtraction_inline_test)
{
    float A[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    float B[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};

    float C[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    DSP_Matrix_SubtractInplace_f32(A, B, 2, 3);

    for(int i = 0; i < 6; i++)
    {
        ck_assert_float_eq_tol(A[i], C[i], 1e-6f);
    }
}

START_TEST(scale_test)
{
    float A[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};

    float C[6] = {2.0f, 4.0f, 6.0f, 8.0f, 10.0f, 12.0f};

    float result[6] = {0.0f};
    DSP_Matrix_Scale_f32(result, A, 2.0f, 2, 3);

    for(int i = 0; i < 6; i++)
    {
        ck_assert_float_eq_tol(result[i], C[i], 1e-6f);
    }
}
END_TEST

START_TEST(scale_inline_test)
{
    float A[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};

    float C[6] = {2.0f, 4.0f, 6.0f, 8.0f, 10.0f, 12.0f};

    DSP_Matrix_ScaleInplace_f32(A, 2.0f, 2, 3);

    for(int i = 0; i < 6; i++)
    {
        ck_assert_float_eq_tol(A[i], C[i], 1e-6f);
    }
}
END_TEST

START_TEST(multiply_elements_test)
{
    float A[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    float B[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};

    float C[6] = {1.0f, 4.0f, 9.0f, 16.0f, 25.0f, 36.0f};

    float result[6] = {0.0f};
    DSP_Matrix_MultiplyElems_f32(result, A, B, 2, 3);

    for(int i = 0; i < 6; i++)
    {
        ck_assert_float_eq_tol(result[i], C[i], 1e-6f);
    }
}
END_TEST

START_TEST(multiply_elements_inline_test)
{
    float A[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    float B[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};

    float C[6] = {1.0f, 4.0f, 9.0f, 16.0f, 25.0f, 36.0f};

    DSP_Matrix_MultiplyElemsInplace_f32(A, B, 2, 3);

    for(int i = 0; i < 6; i++)
    {
        ck_assert_float_eq_tol(A[i], C[i], 1e-6f);
    }
}
END_TEST

START_TEST(transpose_test)
{
    float A[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    float B[6] = {1.0f, 4.0f, 2.0f, 5.0f, 3.0f, 6.0f};

    float result[6] = {0.0f};
    DSP_Matrix_Transpose_f32(result, A, 2, 3);

    for(int i = 0; i < 6; i++)
    {
        ck_assert_float_eq_tol(result[i], B[i], 1e-6f);
    }
}
END_TEST

START_TEST(transpose_inline_test)
{
    float A[9] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f};
    float B[9] = {1.0f, 4.0f, 7.0f, 2.0f, 5.0f, 8.0f, 3.0f, 6.0f, 9.0f};

    DSP_Matrix_TransposeInplace_f32(A, 3);

    for(int i = 0; i < 9; i++)
    {
        ck_assert_float_eq_tol(A[i], B[i], 1e-6f);
    }
}
END_TEST

__attribute__((constructor)) void register_matrix_suite(void)
{
    tr_add_test("Matrix", "core", multiplication_test);
    tr_add_test("Matrix", "core", sandwich_multiplication_test);
    tr_add_test("Matrix", "core", addition_test);
    tr_add_test("Matrix", "core", addition_inline_test);
    tr_add_test("Matrix", "core", subtraction_test);
    tr_add_test("Matrix", "core", subtraction_inline_test);
    tr_add_test("Matrix", "core", scale_test);
    tr_add_test("Matrix", "core", scale_inline_test);
    tr_add_test("Matrix", "core", multiply_elements_test);
    tr_add_test("Matrix", "core", multiply_elements_inline_test);
    tr_add_test("Matrix", "core", transpose_test);
    tr_add_test("Matrix", "core", transpose_inline_test);
}

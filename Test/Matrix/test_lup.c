#include "DSP_Matrix.h"

#include "test_registry.h"

#include <stdio.h>

START_TEST(lup_decomposition_test)
{
    const size_t N = 6;
    float A[36]    = {4, 3, 2, 2, 1, 5, 3, 1, 1, 1, 0, 2, 2, 1, 3, 2, 1, 3,
                      2, 1, 2, 3, 1, 4, 1, 0, 1, 1, 2, 2, 5, 2, 3, 4, 2, 1};

    float A_original[36];
    memcpy(A_original, A, sizeof(A));

    size_t P[N + 1];
    memset(P, 0, sizeof(P));

    int status = DSP_Matrix_LUPDecompose_f32(A, N, 1e-6f, P);
    ck_assert_int_eq(status, 0);

    // Extract L and U from the in-place matrix A
    float L[36] = {0};
    float U[36] = {0};

    for(size_t i = 0; i < N; i++)
    {
        for(size_t j = 0; j < N; j++)
        {
            if(j < i)
            {
                L[i * N + j] = A[i * N + j];
                U[i * N + j] = 0.0f;
            }
            else if(j == i)
            {
                L[i * N + j] = 1.0f;
                U[i * N + j] = A[i * N + j];
            }
            else
            {
                L[i * N + j] = 0.0f;
                U[i * N + j] = A[i * N + j];
            }
        }
    }

    // Compute L * U
    float LU[36] = {0};
    for(size_t i = 0; i < N; i++)
    {
        for(size_t j = 0; j < N; j++)
        {
            for(size_t k = 0; k < N; k++)
            {
                LU[i * N + j] += L[i * N + k] * U[k * N + j];
            }
        }
    }

    // Reorder original matrix using permutation P
    float PA[36] = {0};
    for(size_t i = 0; i < N; i++)
    {
        size_t row = P[i];
        for(size_t j = 0; j < N; j++)
        {
            PA[i * N + j] = A_original[row * N + j];
        }
    }

    // Compare PA and L*U
    for(size_t i = 0; i < N * N; i++)
    {
        ck_assert_float_eq_tol(PA[i], LU[i], 1e-6f);
    }
}
END_TEST


START_TEST(lup_solve_test)
{
    const size_t N = 6;
    float A[]      = {4, 3, 2, 2, 1, 5, 3, 1, 1, 1, 0, 2, 2, 1, 3, 2, 1, 3,
                      2, 1, 2, 3, 1, 4, 1, 0, 1, 1, 2, 2, 5, 2, 3, 4, 2, 1};

    float A_orig[36];
    memcpy(A_orig, A, sizeof(A));

    size_t P[N + 1];
    memset(P, 0, sizeof(P));

    int status = DSP_Matrix_LUPDecompose_f32(A, N, 1e-6f, P);
    ck_assert_int_eq(status, 0);
    float b[6] = {2, 1, 3, 7, 6, 9};
    float x[6] = {0};
    DSP_Matrix_LUPSolve_f32(A, P, x, b, N);

    float expected[6] = {0.2906980f, -1.6046516f, -1.2325581f, 2.5697672f, 1.9883720f, 0.1976744f};
    for(size_t i = 0; i < N; i++)
    {
        ck_assert_float_eq_tol(x[i], expected[i], 1e-6f);
    }
}
END_TEST

START_TEST(lup_invert_test)
{
    const size_t N = 6;
    float A[]      = {4, 3, 2, 2, 1, 5, 3, 1, 1, 1, 0, 2, 2, 1, 3, 2, 1, 3,
                      2, 1, 2, 3, 1, 4, 1, 0, 1, 1, 2, 2, 5, 2, 3, 4, 2, 1};

    float A_orig[N * N];
    memcpy(A_orig, A, sizeof(A));

    size_t P[N + 1];
    memset(P, 0, sizeof(P));

    int status = DSP_Matrix_LUPDecompose_f32(A, N, 1e-6f, P);
    ck_assert_int_eq(status, 0);

    float IA[N * N];
    DSP_Matrix_LUPInvert_f32(IA, A, P, N);

    float expected[] = {-0.133720930232558f, 0.558139534883721f,  -0.093023255813953f, -0.122093023255814f,
                        0.145348837209302f,  0.029069767441860f,  0.558139534883721f,  -0.720930232558139f,
                        -0.046511627906977f, -0.186046511627907f, -0.302325581395349f, 0.139534883720930f,
                        -0.093023255813953f, -0.046511627906977f, 0.674418604651163f,  -0.302325581395349f,
                        -0.116279069767442f, -0.023255813953488f, -0.122093023255814f, -0.186046511627907f,
                        -0.302325581395349f, 0.540697674418605f,  -0.215116279069767f, 0.156976744186047f,
                        0.145348837209302f,  -0.302325581395349f, -0.116279069767442f, -0.215116279069767f,
                        0.494186046511628f,  0.098837209302326f,  0.029069767441860f,  0.139534883720930f,
                        -0.023255813953488f, 0.156976744186047f,  0.098837209302326f,  -0.180232558139535f};
    for(size_t i = 0; i < N * N; i++)
    {
        ck_assert_float_eq_tol(IA[i], expected[i], 1e-6f);
    }
}
END_TEST

START_TEST(lup_determinant_test)
{
    const size_t N = 6;
    float A[36]    = {4, 3, 2, 2, 1, 5, 3, 1, 1, 1, 0, 2, 2, 1, 3, 2, 1, 3,
                      2, 1, 2, 3, 1, 4, 1, 0, 1, 1, 2, 2, 5, 2, 3, 4, 2, 1};

    size_t P[N + 1];
    memset(P, 0, sizeof(P));

    float A_copy[36];
    memcpy(A_copy, A, sizeof(A_copy));

    int status = DSP_Matrix_LUPDecompose_f32(A_copy, N, 1e-6f, P);
    ck_assert_int_eq(status, 0);

    float det = DSP_Matrix_LUPDeterminant_f32(A_copy, P, N);

    // Thats the highest precision we'll get :(
    ck_assert_float_eq_tol(det, 1.7200003e+02f, 1e-4f);
}
END_TEST


START_TEST(lup_right_solve_no_pivot_test)
{
    const size_t N = 4;
    // clang-format off
    float A[16] = {
        10, 2, 1, 1,
         3, 9, 2, 1,
         2, 1, 8, 2,
         1, 1, 1, 7};
    // clang-format on

    // Copy for decomposition
    float LU[N * N];
    memcpy(LU, A, sizeof(A));

    size_t P[N + 1];
    memset(P, 0, sizeof(P));

    ck_assert_int_eq(DSP_Matrix_LUPDecompose_f32(LU, N, 1e-6f, P), 0);

    // Right‑hand side row vector
    float b[4] = {2, 1, 3, 7};
    float x[4] = {0};

    // Solve x*A = b
    DSP_Matrix_LUPRightSolve_f32(LU, P, x, b, N);

    float expected[4] = {0.066294487090021f, -0.034891835310537f, 0.260293091416609f, 0.921144452198186f};

    for(size_t i = 0; i < N; i++)
    {
        ck_assert_float_eq_tol(x[i], expected[i], 1e-6f);
    }

    float ans[4] = {0};
    DSP_Matrix_Multiply_f32(ans, x, 1, 4, A, N);

    for(size_t i = 0; i < N; i++)
    {
        ck_assert_float_eq_tol(ans[i], b[i], 1e-6f);
    }
}
END_TEST

START_TEST(lup_right_solve_with_pivot_test)
{
    const size_t N = 4;
    // clang-format off
    float A[16] = {
        10, 2, 1, 1,
         3, 1, 2, 1,
         2, 7, 1, 2,
         1, 1, 5, 7};
    // clang-format on

    // Copy for decomposition
    float LU[N * N];
    memcpy(LU, A, sizeof(A));

    size_t P[N + 1];
    memset(P, 0, sizeof(P));

    ck_assert_int_eq(DSP_Matrix_LUPDecompose_f32(LU, N, 1e-6f, P), 0);

    // Right‑hand side row vector
    float b[4] = {2, 1, 3, 7};
    float x[4] = {0};

    // Solve x*A = b
    DSP_Matrix_LUPRightSolve_f32(LU, P, x, b, N);

    float expected[4] = {0.574861367837338f, -1.665434380776340f, 0.053604436229205f, 1.140480591497227f};

    for(size_t i = 0; i < N; i++)
    {
        ck_assert_float_eq_tol(x[i], expected[i], 1e-6f);
    }

    float ans[4] = {0};
    DSP_Matrix_Multiply_f32(ans, x, 1, 4, A, N);

    for(size_t i = 0; i < N; i++)
    {
        ck_assert_float_eq_tol(ans[i], b[i], 1e-6f);
    }
}
END_TEST


__attribute__((constructor)) void register_lup_suite(void)
{
    tr_add_test("Matrix", "lup", lup_decomposition_test);
    tr_add_test("Matrix", "lup", lup_solve_test);
    tr_add_test("Matrix", "lup", lup_invert_test);
    tr_add_test("Matrix", "lup", lup_determinant_test);
    tr_add_test("Matrix", "lup", lup_right_solve_no_pivot_test);
    tr_add_test("Matrix", "lup", lup_right_solve_with_pivot_test);
}
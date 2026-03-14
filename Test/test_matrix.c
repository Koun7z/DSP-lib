#include <check.h>
#include <stdlib.h>

#include "DSP_Matrix.h"

START_TEST(multiplication_test)
{
    float A[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    float B[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};

    float C[4] = {22.0f, 28.0f, 49.0f, 64.0f};

    float result[4] = {0.0f};
    DSP_Matrix_Multiply_f32(result, A, 2, 3, B, 2);

    for(int i = 0; i < 4; i++)
    {
        ck_assert_float_eq_tol(result[i], C[i], 0.00001f);
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
        ck_assert_float_eq_tol(result[i], B[i], 0.00001f);
    }
}
END_TEST

START_TEST(transpose_inline_test)
{
    float A[9] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f};
    float B[9] = {1.0f, 4.0f, 7.0f, 2.0f, 5.0f, 8.0f, 3.0f, 6.0f, 9.0f};

    DSP_Matrix_TransposeInline_f32(A, 3);

    for(int i = 0; i < 9; i++)
    {
        ck_assert_float_eq_tol(A[i], B[i], 0.00001f);
    }
}
END_TEST

Suite* matrix_suite(void)
{
    Suite* s    = suite_create("Matrix");
    TCase* core = tcase_create("core");

    tcase_add_test(core, multiplication_test);
    tcase_add_test(core, transpose_test);
    tcase_add_test(core, transpose_inline_test);
    suite_add_tcase(s, core);
    return s;
}

int main(void)
{
    int failed  = 0;
    Suite* s    = matrix_suite();
    SRunner* sr = srunner_create(s);
    srunner_run_all(sr, CK_VERBOSE);
    failed = srunner_ntests_failed(sr);
    srunner_free(sr);
    return (failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}

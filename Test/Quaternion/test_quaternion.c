#include "test_registry.h"

#include "DSP_Quaternion.h"

START_TEST(multiplication_test)
{
    DSP_Quaternion_f32 q1 = {1.0f, 2.0f, 3.0f, 4.0f};
    DSP_Quaternion_f32 q2 = {5.0f, 6.0f, 7.0f, 8.0f};


    DSP_Quaternion_f32 result;
    DSP_QT_Multiply_f32(&result, &q1, &q2);
    ck_assert_float_eq_tol(result.r, -60.0f, 1e-6f);
    ck_assert_float_eq_tol(result.i, 12.0f, 1e-6f);
    ck_assert_float_eq_tol(result.j, 30.0f, 1e-6f);
    ck_assert_float_eq_tol(result.k, 24.0f, 1e-6f);
}

__attribute__((constructor)) void register_quaternion_suite(void)
{
    tr_add_test("Quaternion", "core", multiplication_test);
}

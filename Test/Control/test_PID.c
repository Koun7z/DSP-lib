#include "DSP_SignalFiltering.h"
#include "DSP_PID.h"

#include "DSP_Utils.h"

#include "test_registry.h"

#include <stdio.h>
#include <stdlib.h>

START_TEST(PID_test_f32)
{
    FILE* fd = fopen("./matlab/data/pid_test.csv", "r");
    if(fd == NULL)
    {
        ck_abort_msg("Failed to open test data file");
    }

    float* expected = malloc(sizeof(float) * 128);
    size_t cnt      = 0;
    size_t size     = 128;
    while(!feof(fd))
    {
        float val;

        fscanf(fd, "%f\n", &val);

        if(cnt >= size)
        {
            size    *= 2;
            expected = realloc(expected, sizeof(float) * size);
        }
        expected[cnt] = val;
        cnt++;
    }

    FILE* fp = fopen("./matlab/data/dsp_plant.csv", "r");
    if(fp == NULL)
    {
        ck_abort_msg("Failed to open output file");
    }

    double* b         = malloc(sizeof(double) * 16);
    double* a         = malloc(sizeof(double) * 16);
    size_t order      = 0;
    size_t size_plant = 16;
    while(!feof(fp))
    {
        double val_a;
        double val_b;

        fscanf(fp, "%lf,%lf\n", &val_b, &val_a);

        if(order >= size_plant)
        {
            size_plant *= 2;
            b           = realloc(b, sizeof(double) * size_plant);
            a           = realloc(a, sizeof(double) * size_plant);
        }

        b[order] = val_b;
        a[order] = val_a;
        order++;
    }

    // Reverse direction
    DSP_ReverseArray_f64(b, order);
    DSP_ReverseArray_f64(a, order);

    double* buff = malloc(sizeof(double) * order * 2);

    DSP_IIR_RT_Instance_f64 plant;
    DSP_IIR_RT_Init_f64(&plant, order - 1, b, a, buff, &buff[order]);

    DSP_PID_Instance_f32 pid;
    DSP_PID_Init_f32(&pid, 2.9192874f, 1.6070489f, 0.7738435f, 10, 0.01f);

    float* output = malloc(sizeof(float) * cnt);
    float SP      = 1.0f;
    float PV      = 0.0f;
    for(size_t i = 0; i < cnt; i++)
    {
        const float u = DSP_PID_Update_f32(&pid, SP, PV);

        PV        = DSP_IIR_RT_Update_f64(&plant, u);
        output[i] = PV;
    }

    for(size_t i = 0; i < cnt; i++)
    {
        // Tolerance here is very relaxed due to differences in floating-point precision and
        // implementation details between MATLAB 'pidstd' and our C code.
        ck_assert_float_eq_tol(output[i], expected[i], 1e-1f);
    }

    fclose(fp);
    fclose(fd);
    free(output);
    free(expected);
    free(b);
    free(a);
    free(buff);
}

START_TEST(PID_test_f64)
{
    FILE* fd = fopen("./matlab/data/pid_test.csv", "r");
    if(fd == NULL)
    {
        ck_abort_msg("Failed to open test data file");
    }

    double* expected = malloc(sizeof(double) * 128);
    size_t cnt       = 0;
    size_t size      = 128;
    while(!feof(fd))
    {
        double val;

        fscanf(fd, "%lf\n", &val);

        if(cnt >= size)
        {
            size    *= 2;
            expected = realloc(expected, sizeof(double) * size);
        }
        expected[cnt] = val;
        cnt++;
    }

    FILE* fp = fopen("./matlab/data/dsp_plant.csv", "r");
    if(fp == NULL)
    {
        ck_abort_msg("Failed to open output file");
    }

    double* b         = malloc(sizeof(double) * 16);
    double* a         = malloc(sizeof(double) * 16);
    size_t order      = 0;
    size_t size_plant = 16;
    while(!feof(fp))
    {
        double val_a;
        double val_b;

        fscanf(fp, "%lf,%lf\n", &val_b, &val_a);

        if(order >= size_plant)
        {
            size_plant *= 2;
            b           = realloc(b, sizeof(double) * size_plant);
            a           = realloc(a, sizeof(double) * size_plant);
        }

        b[order] = val_b;
        a[order] = val_a;
        order++;
    }

    // Reverse direction
    DSP_ReverseArray_f64(b, order);
    DSP_ReverseArray_f64(a, order);

    double* buff = malloc(sizeof(double) * order * 2);

    DSP_IIR_RT_Instance_f64 plant;
    DSP_IIR_RT_Init_f64(&plant, order - 1, b, a, buff, &buff[order]);

    DSP_PID_Instance_f64 pid;
    DSP_PID_Init_f64(&pid, 2.9192874, 1.6070489, 0.7738435, 10, 0.01);

    FILE* fw = fopen("./matlab/data/pid_output.csv", "w");
    if(fw == NULL)
    {
        ck_abort_msg("Failed to open output file");
    }

    double* output = malloc(sizeof(double) * cnt);
    double SP      = 1.0;
    double PV      = 0.0;
    for(size_t i = 0; i < cnt; i++)
    {
        const double u = DSP_PID_Update_f64(&pid, SP, PV);

        PV        = DSP_IIR_RT_Update_f64(&plant, u);
        output[i] = PV;

        fprintf(fw, "%f,%f\n", expected[i], output[i]);
    }
    fclose(fw);

    for(size_t i = 0; i < cnt; i++)
    {
        ck_assert_double_eq_tol(output[i], expected[i], 1e-1);
    }

    fclose(fp);
    fclose(fd);
    free(output);
    free(expected);
    free(b);
    free(a);
    free(buff);
}

__attribute__((constructor)) void register_PID_suite(void)
{
    tr_add_test("Control", "PID_f32", PID_test_f32);
    tr_add_test("Control", "PID_f64", PID_test_f64);
}
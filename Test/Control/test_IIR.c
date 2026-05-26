#include "DSP_IIR.h"

#include "DSP_Utils.h"

#include "test_registry.h"

#include <stdio.h>
#include <stdlib.h>

START_TEST(IIR_f32_filter_test)
{
    FILE* fd = fopen("./python/data/iir_test_f32.csv", "r");
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

    FILE* fp = fopen("./python/data/dsp_plant.csv", "r");
    if(fp == NULL)
    {
        ck_abort_msg("Failed to open output file");
    }

    float* b          = malloc(sizeof(float) * 16);
    float* a          = malloc(sizeof(float) * 16);
    size_t order      = 0;
    size_t size_plant = 16;
    while(!feof(fp))
    {
        float val_a;
        float val_b;

        fscanf(fp, "%f,%f\n", &val_b, &val_a);

        if(order >= size_plant)
        {
            size_plant *= 2;
            b           = realloc(b, sizeof(float) * size_plant);
            a           = realloc(a, sizeof(float) * size_plant);
        }

        b[order] = val_b;
        a[order] = val_a;
        order++;
    }

    float* buff = malloc(sizeof(float) * order);

    DSP_IIR_RT_Instance_f32 plant;
    DSP_IIR_RT_Init_f32(&plant, order - 1, b, a, buff);
    FILE* fw = fopen("./python/data/iir_output_f32.csv", "w");
    if(fw == NULL)
    {
        ck_abort_msg("Failed to open output file");
    }

    float* output = malloc(sizeof(float) * cnt);
    for(size_t i = 0; i < cnt; i++)
    {
        output[i] = DSP_IIR_RT_Update_f32(&plant, 1.0f);

        fprintf(fw, "%f\n", output[i]);
    }

    fclose(fw);
    for(size_t i = 0; i < cnt; i++)
    {
        ck_assert_float_eq_tol(output[i], expected[i], 1e-6f);
    }

    fclose(fd);
    free(output);
    free(expected);
    free(b);
    free(a);
    free(buff);
    fclose(fp);
}

START_TEST(IIR_f64_filter_test)
{
    FILE* fd = fopen("./python/data/iir_test_f64.csv", "r");
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

    FILE* fp = fopen("./python/data/dsp_plant.csv", "r");
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

    double* buff = malloc(sizeof(double) * order);

    DSP_IIR_RT_Instance_f64 plant;
    DSP_IIR_RT_Init_f64(&plant, order - 1, b, a, buff);

    FILE* fw = fopen("./python/data/iir_output_f64.csv", "w");
    if(fw == NULL)
    {
        ck_abort_msg("Failed to open output file");
    }

    double* output = malloc(sizeof(double) * cnt);
    for(size_t i = 0; i < cnt; i++)
    {
        output[i] = DSP_IIR_RT_Update_f64(&plant, 1.0);

        fprintf(fw, "%f\n", output[i]);
        ck_assert_double_eq_tol(output[i], expected[i], 1e-12);
    }

    fclose(fd);
    free(output);
    free(expected);
    free(b);
    free(a);
    free(buff);
    fclose(fp);
}

__attribute__((constructor)) void register_IIR_suite(void)
{
    tr_add_test("Control", "IIR", IIR_f32_filter_test);
    tr_add_test("Control", "IIR", IIR_f64_filter_test);
}
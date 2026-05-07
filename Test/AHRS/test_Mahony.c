#include "DSP_AHRS_Mahony.h"

#include "test_registry.h"

#include <stdio.h>
#include <stdlib.h>

#define BASE_BLOCK_SIZE 128

START_TEST(DSP_AHRS_Mahony)
{
    FILE* fg = fopen("./python/data/sim_gyro_data.csv", "r");
    FILE* fa = fopen("./python/data/sim_accel_data.csv", "r");

    if(fg == NULL || fa == NULL)
    {
        ck_abort_msg("Failed to open test data files");
    }

    typedef struct
    {
        float gx;
        float gy;
        float gz;
        float ax;
        float ay;
        float az;
    } test_data_t;

    test_data_t* test_data = malloc(sizeof(test_data_t) * BASE_BLOCK_SIZE);
    size_t cnt             = 0;
    size_t size            = BASE_BLOCK_SIZE;

    while(!feof(fg) && !feof(fa))
    {
        float gx, gy, gz;
        float ax, ay, az;

        fscanf(fg, "%f,%f,%f\n", &gx, &gy, &gz);
        fscanf(fa, "%f,%f,%f\n", &ax, &ay, &az);

        if(cnt >= size)
        {
            size     *= 2;
            test_data = realloc(test_data, sizeof(test_data_t) * size);
        }
        test_data[cnt].gx = gx;
        test_data[cnt].gy = gy;
        test_data[cnt].gz = gz;
        test_data[cnt].ax = ax;
        test_data[cnt].ay = ay;
        test_data[cnt].az = az;
        cnt++;
    }

    DSP_AHRS_Mahony_Instance_f32 mahony_filter;
    DSP_AHRS_Mahony_Init_f32(&mahony_filter, 1.0f, 0.3f, 0.0f);

    DSP_AHRS_DataInstance_f32 data;
    DSP_AHRS_DataInit_f32(&data);

    FILE* fw = fopen("./python/data/ahrs_mahony_output.csv", "w");

    for(size_t i = 0; i < cnt; i++)
    {
        data.GyroData[0] = test_data[i].gx;
        data.GyroData[1] = test_data[i].gy;
        data.GyroData[2] = test_data[i].gz;

        data.AccData[0] = test_data[i].ax;
        data.AccData[1] = test_data[i].ay;
        data.AccData[2] = test_data[i].az;

        DSP_AHRS_Mahony_FilterUpdate_f32(&mahony_filter, &data, 0.01f);
        fprintf(fw, "%f,%f,%f,%f\n", data.AttitudeEstimate.r, data.AttitudeEstimate.i, data.AttitudeEstimate.j,
                data.AttitudeEstimate.k);
    }

    free(test_data);
    fclose(fw);

    ck_assert_msg(1, "Mahony AHRS output saved to ./python/data/ahrs_mahony_output.csv");
}

__attribute__((constructor)) void register_mahony_tests()
{
    tr_add_test("AHRS", "Mahony", DSP_AHRS_Mahony);
}
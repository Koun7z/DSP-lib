#include "DSP_AHRS_Madgwick.h"

#include "test_registry.h"

#include "AHRS_TestData.h"

#include <stdio.h>
#include <stdlib.h>

#define BASE_BLOCK_SIZE 128

START_TEST(DSP_AHRS_Madgwick)
{
    size_t cnt                 = 0;
    AHRS_TestData_t* test_data = AHRS_LoadTestData("./Test/RepoIMU/TStick/TStick_Test02_Trial1.csv", &cnt);
    if(test_data == NULL || cnt == 0)
    {
        ck_abort_msg("Failed to load test data");
    }

    DSP_AHRS_Madgwick_Instance_f32 madgwick_filter;
    DSP_AHRS_Madgwick_Init_f32(&madgwick_filter, 0.033f, false);

    DSP_AHRS_DataInstance_f32 data;
    DSP_AHRS_DataInit_f32(&data);

    FILE* fw = fopen("./python/data/ahrs_madgwick_output.csv", "w");

    for(size_t i = 0; i < cnt; i++)
    {
        data.GyroData[0] = test_data[i].GyroData[0];
        data.GyroData[1] = test_data[i].GyroData[1];
        data.GyroData[2] = test_data[i].GyroData[2];

        data.AccData[0] = test_data[i].AccData[0];
        data.AccData[1] = test_data[i].AccData[1];
        data.AccData[2] = test_data[i].AccData[2];

        DSP_AHRS_Madgwick_FilterUpdate_f32(&madgwick_filter, &data, 0.01f);
        fprintf(fw, "%f,%f,%f,%f\n", data.AttitudeEstimate.r, data.AttitudeEstimate.i, data.AttitudeEstimate.j,
                data.AttitudeEstimate.k);
    }

    free(test_data);
    fclose(fw);

    ck_assert_msg(1, "Madgwick AHRS output saved to ./python/data/ahrs_madgwick_output.csv");
}

__attribute__((constructor)) void register_madgwick_tests()
{
    tr_add_test("AHRS", "Madgwick", DSP_AHRS_Madgwick);
}
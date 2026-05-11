#include "DSP_AHRS_NC.h"

#include "test_registry.h"
#include "AHRS_TestUtils.h"

#include <stdio.h>
#include <stdlib.h>

#define BASE_BLOCK_SIZE 128

START_TEST(test_data)
{
    const char* input_path  = "./python/data/ahrs_synth_data.csv";
    const char* output_path = "./python/data/ahrs_nc_synth_output.csv";

    char resolved_path[PATH_MAX];
    const char* full_path = get_full_path(input_path, resolved_path, sizeof(resolved_path));

    char resolved_out_path[PATH_MAX];
    const char* full_out_path = get_full_path(output_path, resolved_out_path, sizeof(resolved_out_path));


    size_t cnt                 = 0;
    AHRS_TestData_t* test_data = AHRS_LoadTestData(full_path, &cnt);
    if(test_data == NULL || cnt == 0)
    {
        ck_abort_msg("Failed to load test data");
    }

    DSP_AHRS_NC_Instance_f32 nc_filter;
    DSP_AHRS_NC_Init_f32(&nc_filter, 0.1f, 0.0f, 0.95f, -1.0f, -1.0f);

    DSP_AHRS_DataInstance_f32 data;
    DSP_AHRS_DataInit_f32(&data);

    FILE* fw = fopen(full_out_path, "w");
    fprintf(fw, "# %s\n", full_path);

    if(fw == NULL)
    {
        ck_abort_msg("Failed to open output file");
    }

    bool passed = true;
    for(size_t i = 0; i < cnt; i++)
    {
        data.GyroData[0] = test_data[i].GyroData[0];
        data.GyroData[1] = test_data[i].GyroData[1];
        data.GyroData[2] = test_data[i].GyroData[2];

        data.AccData[0] = test_data[i].AccData[0];
        data.AccData[1] = test_data[i].AccData[1];
        data.AccData[2] = test_data[i].AccData[2];

        DSP_AHRS_NC_FilterUpdate_f32(&nc_filter, &data, 0.01f);
        fprintf(fw, "%f,%f,%f,%f\n", data.AttitudeEstimate.r, data.AttitudeEstimate.i, data.AttitudeEstimate.j,
                data.AttitudeEstimate.k);

        // With no noise in data, the filter should be able to track the reference almost perfectly.
        // Simple test if the filter at lest converges on the expected value.
        float diff = fabsf(data.AttitudeEstimate.r - test_data[i].AttitudeReference.r)
                   + fabsf(data.AttitudeEstimate.i - test_data[i].AttitudeReference.i)
                   + fabsf(data.AttitudeEstimate.j - test_data[i].AttitudeReference.j)
                   + fabsf(data.AttitudeEstimate.k - test_data[i].AttitudeReference.k);
        if(diff > 0.1f)
        {
            passed = false;
        }
    }

    free(test_data);
    fclose(fw);

    ck_assert_msg(passed, "NC AHRS output saved to %s", full_out_path);
}

START_TEST(RepoIMU_data)
{
    const char* input_path  = "./Test/RepoIMU/TStick/TStick_Test02_Trial1.csv";
    const char* output_path = "./python/data/ahrs_nc_RepoIMU_output.csv";

    char resolved_path[PATH_MAX];
    const char* full_path = get_full_path(input_path, resolved_path, sizeof(resolved_path));

    char resolved_out_path[PATH_MAX];
    const char* full_out_path = get_full_path(output_path, resolved_out_path, sizeof(resolved_out_path));


    size_t cnt                 = 0;
    AHRS_TestData_t* test_data = AHRS_LoadTestData(full_path, &cnt);
    if(test_data == NULL || cnt == 0)
    {
        ck_abort_msg("Failed to load test data");
    }

    DSP_AHRS_NC_Instance_f32 nc_filter;
    DSP_AHRS_NC_Init_f32(&nc_filter, 0.1f, 0.0f, 0.95f, -1.0f, -1.0f);

    DSP_AHRS_DataInstance_f32 data;
    DSP_AHRS_DataInit_f32(&data);

    FILE* fw = fopen(full_out_path, "w");
    fprintf(fw, "# %s\n", full_path);

    if(fw == NULL)
    {
        ck_abort_msg("Failed to open output file");
    }

    for(size_t i = 0; i < cnt; i++)
    {
        data.GyroData[0] = test_data[i].GyroData[0];
        data.GyroData[1] = test_data[i].GyroData[1];
        data.GyroData[2] = test_data[i].GyroData[2];

        data.AccData[0] = test_data[i].AccData[0];
        data.AccData[1] = test_data[i].AccData[1];
        data.AccData[2] = test_data[i].AccData[2];

        DSP_AHRS_NC_FilterUpdate_f32(&nc_filter, &data, 0.01f);
        fprintf(fw, "%f,%f,%f,%f\n", data.AttitudeEstimate.r, data.AttitudeEstimate.i, data.AttitudeEstimate.j,
                data.AttitudeEstimate.k);
    }

    free(test_data);
    fclose(fw);

    ck_assert_msg(1, "NC AHRS output saved to %s", full_out_path);
}

__attribute__((constructor)) void register_tests()
{
    tr_add_test("AHRS", "NC", RepoIMU_data);
    tr_add_test("AHRS", "NC", test_data);
}
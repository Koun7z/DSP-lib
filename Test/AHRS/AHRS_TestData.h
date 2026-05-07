#ifndef AHRS_TESTDATA_H__
#define AHRS_TESTDATA_H__

#include "DSP_Quaternion.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

typedef struct
{
    float GyroData[3];
    float AccData[3];
    float MagData[3];

    DSP_Quaternion_f32 AttitudeReference;
} AHRS_TestData_t;


#define BUF_SIZE 4096
static inline int count_lines(FILE* file)
{
    char buf[BUF_SIZE];
    int counter = 0;
    for(;;)
    {
        size_t res = fread(buf, 1, BUF_SIZE, file);
        if(ferror(file))
        {
            return -1;
        }

        for(size_t i = 0; i < res; i++)
        {
            if(buf[i] == '\n')
            {
                counter++;
            }
        }

        if(feof(file))
        {
            break;
        }
    }

    return counter;
}

static inline AHRS_TestData_t* AHRS_LoadTestData(char* path, size_t* out_cnt)
{
    FILE* f = fopen(path, "r");
    if(f == NULL)
    {
        return NULL;
    }
    int line_cnt = count_lines(f);

    fseek(f, 0, SEEK_SET);

    AHRS_TestData_t* data = malloc(sizeof(AHRS_TestData_t) * line_cnt);
    if(data == NULL)
    {
        fclose(f);
        return NULL;
    }

    const size_t header_lines = 2;

    char line[1024];
    for(size_t i = 0; i < header_lines; i++)
    {
        if(!fgets(line, sizeof(line), f))
        {
            free(data);
            fclose(f);
            return NULL;
        }
    }


    size_t cnt = 0;
    while(true)
    {
        float t;
        float qr, qi, qj, qk;
        float ax, ay, az;
        float gx, gy, gz;
        float mx, my, mz;


        int res = fscanf(f, "%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f", &t, &qr, &qi, &qj, &qk, &ax, &ay, &az, &gx,
                         &gy, &gz, &mx, &my, &mz);


        if(res != 14)
        {
            break;
        }


        data[cnt].AttitudeReference.r = qr;
        data[cnt].AttitudeReference.i = qi;
        data[cnt].AttitudeReference.j = qj;
        data[cnt].AttitudeReference.k = qk;
        data[cnt].GyroData[0]         = gx;
        data[cnt].GyroData[1]         = gy;
        data[cnt].GyroData[2]         = gz;
        data[cnt].AccData[0]          = ax;
        data[cnt].AccData[1]          = ay;
        data[cnt].AccData[2]          = az;
        data[cnt].MagData[0]          = mx;
        data[cnt].MagData[1]          = my;
        data[cnt].MagData[2]          = mz;
        cnt++;
    }

    fclose(f);

    *out_cnt = cnt;
    return data;
}

#endif /* AHRS_TESTDATA_H__ */
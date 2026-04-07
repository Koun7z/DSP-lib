cmake_minimum_required(VERSION 3.14)

set(SRC_F32
    ${SRC_DIR}/AHRS/DSP_AHRS_Data.c
    ${SRC_DIR}/AHRS/DSP_AHRS_NC_f32.c
    ${SRC_DIR}/AHRS/DSP_AHRS_EKF_f32.c
)

set(SRC_F64)

target_sources(DSP-lib PRIVATE ${SRC_F32})
target_sources(DSP-lib PRIVATE ${SRC_F64})
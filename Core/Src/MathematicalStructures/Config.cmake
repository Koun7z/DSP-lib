cmake_minimum_required(VERSION 3.14)

set(SRC_F32
    ${SRC_DIR}/MathematicalStructures/DSP_Quaternion_f32.c
    ${SRC_DIR}/MathematicalStructures/DSP_Matrix_f32.c
)

set(SRC_F64
    ${SRC_DIR}/MathematicalStructures/DSP_Quaternion_f64.c
    ${SRC_DIR}/MathematicalStructures/DSP_Matrix_f64.c
)

target_sources(DSP-lib PRIVATE ${SRC_F32} ${SRC_F64})
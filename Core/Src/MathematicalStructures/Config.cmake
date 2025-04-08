cmake_minimum_required(VERSION 3.14)

set(SRC_F32 Src/MathematicalStructures/Quaternion_f32.c)

target_sources(DSP-lib PRIVATE ${SRC_F32})
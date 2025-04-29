cmake_minimum_required(VERSION 3.14)

set(SRC_F32 Src/ProcessControl/DSP_PID_f32.c
            Src/ProcessControl/DSP_SimplePID_f32.c)

set(SRC_F64 Src/ProcessControl/DSP_PID_f64.c)
target_sources(DSP-lib PRIVATE ${SRC_F32} ${SRC_F64})
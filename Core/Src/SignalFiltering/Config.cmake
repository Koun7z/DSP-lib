cmake_minimum_required(VERSION 3.14)

set(SRC_F32 Src/SignalFiltering/DSP_FIR_RT_f32.c
            Src/SignalFiltering/DSP_IIR_RT_f32.c)

set(SRC_F64 Src/SignalFiltering/DSP_FIR_RT_f64.c
            Src/SignalFiltering/DSP_IIR_RT_f64.c)

target_sources(DSP-lib PRIVATE ${SRC_F32})
target_sources(DSP-lib PRIVATE ${SRC_F64})
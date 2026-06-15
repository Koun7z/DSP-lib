#ifndef DSP_BENCHMARK_H__
#define DSP_BENCHMARK_H__

#include <stdint.h>

#if defined(_WIN32)

#  include <windows.h>

static LARGE_INTEGER dsp_bench_freq;

static inline void dsp_bench_init(void)
{
    QueryPerformanceFrequency(&dsp_bench_freq);
}

static inline uint64_t DSP_BENCH_GET_TICK(void)
{
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
    return (uint64_t)t.QuadPart;
}

#  define DSP_BENCH_TICK_RESOLUTION_HZ ((uint64_t)dsp_bench_freq.QuadPart)

#elif defined(__linux__) || defined(__APPLE__)

#  include <time.h>

static inline void dsp_bench_init(void) {}

static inline uint64_t DSP_BENCH_GET_TICK(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}


#  define DSP_BENCH_TICK_RESOLUTION_HZ (1000000000ULL)

#else

// this only works on ARM Cortex-M with DWT cycle counter,
// make sure to call dsp_bench_init() before using the benchmark macros

#  define DSP_BENCH_TICK_RESOLUTION_HZ (SystemCoreClock)
#  define DSP_BENCH_GET_TICK()         (DWT->CYCCNT)

static inline void dsp_bench_init(void)
{
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

#endif


#define DSP_BENCH_GET_RUNTIME_US(__f__, __avg_over_n__)                                                            \
    do                                                                                                             \
    {                                                                                                              \
        static uint64_t __f_total_runtime__ = 0;                                                                   \
        static uint64_t __f_call_counter__  = 0;                                                                   \
        static uint64_t __f_max_runtime__   = 0;                                                                   \
                                                                                                                   \
        uint64_t start = DSP_BENCH_GET_TICK();                                                                     \
        (__f__);                                                                                                   \
        uint64_t end = DSP_BENCH_GET_TICK();                                                                       \
                                                                                                                   \
        uint64_t runtime     = end - start;                                                                        \
        __f_total_runtime__ += runtime;                                                                            \
        __f_call_counter__++;                                                                                      \
                                                                                                                   \
        if(runtime > __f_max_runtime__)                                                                            \
        {                                                                                                          \
            __f_max_runtime__ = runtime;                                                                           \
        }                                                                                                          \
                                                                                                                   \
        if(__f_call_counter__ >= (__avg_over_n__))                                                                 \
        {                                                                                                          \
            float avg_us = ((float)__f_total_runtime__ / __f_call_counter__) / (float)DSP_BENCH_TICK_RESOLUTION_HZ \
                         * 1e6f;                                                                                   \
            float max_us = ((float)__f_max_runtime__) / (float)DSP_BENCH_TICK_RESOLUTION_HZ * 1e6f;                \
            printf("Runtime of %s: avg - %f us, max - %f us\n", #__f__, avg_us, max_us);                           \
                                                                                                                   \
            __f_total_runtime__ = 0;                                                                               \
            __f_call_counter__  = 0;                                                                               \
            __f_max_runtime__   = 0;                                                                               \
        }                                                                                                          \
    }                                                                                                              \
    while(0)


static inline uint32_t xor_shift_f32(uint32_t* state)
{
    uint32_t x = *state;
    x         ^= x << 13;
    x         ^= x >> 17;
    x         ^= x << 5;
    *state     = x;
    return x;
}

void DSP_Bench_RandomArray_f32(float* out, size_t size, uint32_t seed)
{
    uint32_t state = seed ? seed : UINT32_C(0xdeadbeef);

    for(size_t i = 0; i < size; i++)
    {
        uint32_t r = xor_shift_f32(&state);

        // Convert to float in range [0, 1)
        out[i] = (float)r / (float)UINT32_MAX;
    }
}


#endif /* DSP_BENCHMARK_H__ */
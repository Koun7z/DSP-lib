# Digital Signal Processing Library

A platform agnostic C library implementing commonly used digital signal processing algorithms and other more complex mathematical functions.

The main goal of the library is to provide a exhaustive collection of efficient and small algorithms for embedded systems, that can be used in most hobby projects in area of robotics, UAV's or any other application requiring real time data processing, state estimation or automatic control algorithms.

Most of the algorithms don't use any dynamic allocation, so the memory footprint of the library is very small and predictable.

## Currently implemented features (_and some plans for not so near future_):

- Signal Processing:
  - [x] Finite impulse response filters:
    - [x] For real time conversion
    - [ ] For converting blocks of samples
  - [x] Infinite impulse response filters:
    - [x] For real time conversion
    - [ ] For converting blocks of samples
  - [ ] Fourier Transform
- Process Control:
  - [x] PID controller
    - D-term filter
    - Output saturation + anti-windup (back-calculation)
    - Partial setpoint gain for K and D parts (simplified 2D PID)
- Attitude and Heading Reference System Algorithms:
  - [x] Quaternion Complementary Attitude Estimator
  - [x] Quaternion Extended Kalman Filter
  - [x] Madgwick Orientation Filter
  - [x] Mahony Orientation Filter
- Other Math Algorithms:
  - [x] Quaternion math
  - [ ] Complex numbers math
  - [ ] Vector and matrix math (WIP)
    - [x] Basic Matrix operations (addition, multiplication, transposition, inversion)
    - [x] LU decomposition with partial pivoting (with left and right solve, inverse, and determinant helpers)
    - [ ] Cholesky decomposition
    - [ ] QR decomposition

> [!NOTE]
> The library is in a state of constant development. All parts of the library are subject to change.

## Building the library:

### Recommended compile options:

To ensure the library runs as fast as it can it is recommended to use `-O3` (or `-Ofast` if speed is a bigger concern than accuracy) compile flag.

If running the library on microcontroller of any kind:

- Enable the FPU , so that the processor doesn't get overwhelmed with floating-point emulation
- Use `-ffunction-sections` and `-fdata-sections` compile flags as well as `-Wl` `-gc-sections` linker flags to reduce the final binary size.

### Necessary files:

The library is set up with CMake in mind, but if you want to use other tools,
all files needed to build the library are inside the `Core` folder.

### Building with CMake:

To build with CMake add the library as a subdirectory to the CMakeList.txt in your main project.
If you're using `add_compile_option()` or `add_link_options()` make sure to add them before the `add_subdirectory()`
call, if you want to use the same options for compiling this library.
Otherwise, specify compiler and linker flags using `target_compile_options()`/`target_link_options()` (recommended if you don't want to enable aggressive optimization in the whole project).

```cmake
#Example CMakeLists file
cmake_minimum_required(VERSION 3.14)
project(example_project)

.
.
.

# Common flags
add_compile_option(${COMPILE_OPTIONS})
add_link_options(${LINKER_OPTIONS})

# Add the library as subdirectory
# ${DSP-lib} is a relative path to your instance of the library
add_subdirectory(${DSP-lib})

# Link the binary to your project
target_link_libraries(${PROJECT_NAME} PRIVATE DSP-lib)

# DSP-lib specific flags
target_compile_options(DSP-lib PRIVATE ${DSP_COMPILE_OPTIONS})
```

## License

This library is licensed under
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT).

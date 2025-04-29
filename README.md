# Digital Signal Processing Library

A multiplatform C library implementing commonly used digital signal processing algorithms and other more complex
mathematical functions.

It was made with a purpose of learning and using it in personal projects (mostly on Arm microcontrollers,
though it's not specifically optimised for that).

## Currently implemented features (_and some plans for not so near future_):

- Signal Processing:
    - [x] Finite impulse response filters:
        - [x] For real time convertion
        - [ ] For converting blocks of samples
    - [x] Infinite impulse response filters:
        - [x] For real time convertion
        - [ ] For converting blocks of samples
    - [ ] Fourier Transform
- Process Control:
    - [x] PID controller
      - D-term filter
      - Output saturation + anti-windup
      - Partial set point gains for K and D parts
- Other math algorithms:
    - [x] Quaternion math
    - [ ] Complex numbers math
    - [ ] Vector and matrix math

> [!NOTE]
> The library is in a state of constant development. All parts of the library are subject to change.

## Building the library:

### Recommended compile options:

To ensure the library runs as fast as it can it is recommended to use `-Ofast` or at least `-03` compile flag.

If running the library on microcontroller of any kind:

- Enable the FPU , so that the processor doesn't get overwhelmed with floating-point emulation
- Use `-ffunction-sections` and `-fdata-sections` compile flags as well as `-Wl` `-gc-sections` linker flags
  to reduce the final binary size.

### Necessary files:

The library is set up to be built with CMake, but if you want to use other tools,
all files needed to build the library are inside the `Core` folder.

### Building with CMake:

To build with CMake add the library as a subdirectory to the CMakeList.txt in your main project.
If you're using `add_compile_option()` or `add_link_options()` make sure to add them before the `add_subdirectory()`
call, if you want to use the same options for compiling this library.
Otherwise, specify compiler and linker flags using `target_compile_options()`/`target_link_options()`.

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
add_subdirectory(${DSP-lib}/Core)

# Link the binary to your project
target_link_libraries(${PROJECT_NAME} PRIVATE DSP-lib)

# DSP-lib specific flags
target_compile_options(DSP-lib PRIVATE ${DSP_COMPILE_OPTIONS})
```

## License

This library is licensed under
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT).

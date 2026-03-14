cmake_minimum_required(VERSION 3.14)
project(DSP-lib)

add_library(DSP-lib STATIC)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src)
set(INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc)

include(${SRC_DIR}/MathematicalStructures/Config.cmake)
include(${SRC_DIR}/ProcessControl/Config.cmake)
include(${SRC_DIR}/SignalFiltering/Config.cmake)
include(${SRC_DIR}/AHRS/Config.cmake)

target_include_directories(DSP-lib PUBLIC
    ${INC_DIR}
    ${INC_DIR}/AHRS)

# ---- Tests ----
option(BUILD_TESTS "Build unit tests (Check + CTest)" OFF)

if(BUILD_TESTS)
    include(Test/Config.cmake)
endif()

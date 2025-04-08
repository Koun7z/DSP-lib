cmake_minimum_required(VERSION 3.14)
cmake_policy(SET CMP0077 NEW)
project(DSP-lib)

add_library(DSP-lib STATIC)

include(Src/MathematicalStructures/Config.cmake)
include(Src/ProcessControl/Config.cmake)
include(Src/SignalFiltering/Config.cmake)

target_include_directories(DSP-lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Inc)
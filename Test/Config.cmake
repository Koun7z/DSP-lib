set(TEST_DIR ${CMAKE_SOURCE_DIR}/Test)

enable_testing()

find_package(PkgConfig REQUIRED)
pkg_check_modules(CHECK REQUIRED check)

find_package(Threads REQUIRED)

# Test sources here
add_executable(test_dsp
    ${TEST_DIR}/test_matrix.c
)

# Include and link
target_include_directories(test_dsp PRIVATE
    ${CHECK_INCLUDE_DIRS}
    ${INC_DIR}/
    ${INC_DIR}/AHRS
)

target_compile_options(test_dsp PRIVATE ${CHECK_CFLAGS_OTHER})
target_link_libraries(test_dsp
    PRIVATE
    DSP-lib
    ${CHECK_LIBRARIES}
    Threads::Threads
    m
)

# Register with CTest
add_test(NAME test_dsp COMMAND test_dsp)
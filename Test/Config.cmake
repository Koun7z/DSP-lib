set(TEST_DIR ${CMAKE_SOURCE_DIR}/Test)

enable_testing()

find_package(PkgConfig REQUIRED)
pkg_check_modules(CHECK REQUIRED check)

find_package(Threads REQUIRED)

target_compile_options(DSP-lib PRIVATE
    -Wall -Wextra
    $<$<CONFIG:Debug>:-O0 -g>
    $<$<CONFIG:Release>:-O3>
)

function(add_dsp_test TEST_NAME)
    add_executable(${TEST_NAME}
        ${TEST_DIR}/test_main.c
        ${TEST_DIR}/test_registry.cpp
        ${ARGN}
    )

    target_include_directories(${TEST_NAME} PRIVATE
        ${TEST_DIR}
        ${CHECK_INCLUDE_DIRS}
        ${INC_DIR}
        ${INC_DIR}/AHRS
    )

    target_compile_options(${TEST_NAME} PRIVATE
        ${CHECK_CFLAGS_OTHER}
        -Wall -Wextra
    )

    target_compile_options(${TEST_NAME} PRIVATE
        $<$<CONFIG:Debug>:-O0 -g>
        $<$<CONFIG:Release>:-O3>
    )

    target_link_libraries(${TEST_NAME} PRIVATE
        DSP-lib
        ${CHECK_LIBRARIES}
        Threads::Threads
        m
    )

    add_test(
        NAME ${TEST_NAME}
        COMMAND ${TEST_NAME}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
endfunction()

add_dsp_test(test_matrix
    ${TEST_DIR}/Matrix/test_matrix.c
    ${TEST_DIR}/Matrix/test_lup.c
)

add_dsp_test(test_control
    ${TEST_DIR}/Control/test_IIR.c
    ${TEST_DIR}/Control/test_PID.c
)

add_dsp_test(test_ahrs
    ${TEST_DIR}/AHRS/test_NC.c
    ${TEST_DIR}/AHRS/test_EKF.c
    ${TEST_DIR}/AHRS/test_Mahony.c
    ${TEST_DIR}/AHRS/test_Madgwick.c
)

add_dsp_test(test_quaternion
    ${TEST_DIR}/Quaternion/test_quaternion.c
)

add_executable(dsp-bench
    ${TEST_DIR}/bench_main.c
)

target_include_directories(dsp-bench PRIVATE
    ${TEST_DIR}
    ${CHECK_INCLUDE_DIRS}
    ${INC_DIR}
    ${INC_DIR}/AHRS
)

target_compile_options(dsp-bench PRIVATE
    ${CHECK_CFLAGS_OTHER}
    -Wall -Wextra
    $<$<CONFIG:Debug>:-O0 -g>
    $<$<CONFIG:Release>:-O3>
)

target_link_libraries(dsp-bench PRIVATE
    DSP-lib
    ${CHECK_LIBRARIES}
    Threads::Threads
    m
)
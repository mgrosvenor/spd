# Cross-compile test script invoked by CTest via cmake -P
# Usage: cmake -DTOOLCHAIN_FILE=<path> -DSRC_DIR=<path> -P test_cross_compile.cmake
#
# Exits 0 (PASS), 1 (FAIL), or 77 (SKIP — compiler not found).

cmake_minimum_required(VERSION 3.13)

if(NOT DEFINED TOOLCHAIN_FILE)
    message(FATAL_ERROR "TOOLCHAIN_FILE not set")
endif()
if(NOT DEFINED SRC_DIR)
    message(FATAL_ERROR "SRC_DIR not set")
endif()

set(BUILD_DIR "${CMAKE_BINARY_DIR}/cross_${TARGET_NAME}")
file(MAKE_DIRECTORY "${BUILD_DIR}")

execute_process(
    COMMAND ${CMAKE_COMMAND}
        -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_FILE}
        -DCMAKE_C_STANDARD=99
        -DCMAKE_C_STANDARD_REQUIRED=ON
        -DCMAKE_C_EXTENSIONS=OFF
        -DCMAKE_BUILD_TYPE=Release
        ${SRC_DIR}
    WORKING_DIRECTORY "${BUILD_DIR}"
    RESULT_VARIABLE config_result
    OUTPUT_VARIABLE config_output
    ERROR_VARIABLE  config_error
)

if(NOT config_result EQUAL 0)
    # Check whether the compiler was simply not found (SKIP) vs actual error
    if(config_error MATCHES "Could not find compiler" OR
       config_error MATCHES "CMAKE_C_COMPILER.*NOTFOUND" OR
       config_error MATCHES "No CMAKE_C_COMPILER could be found")
        message(STATUS "SKIP: compiler not found for ${TARGET_NAME}")
        # CTest exit code 77 = not run / skip... but cmake -P can't set exit codes easily.
        # We write a result file instead and let the wrapper shell script handle it.
        file(WRITE "${BUILD_DIR}/result.txt" "SKIP")
    else()
        message(STATUS "FAIL: cmake configure failed for ${TARGET_NAME}")
        message(STATUS "${config_error}")
        file(WRITE "${BUILD_DIR}/result.txt" "FAIL")
    endif()
    return()
endif()

execute_process(
    COMMAND ${CMAKE_COMMAND} --build "${BUILD_DIR}" --target sdp
    RESULT_VARIABLE build_result
    OUTPUT_VARIABLE build_output
    ERROR_VARIABLE  build_error
)

if(build_result EQUAL 0)
    message(STATUS "PASS: ${TARGET_NAME}")
    file(WRITE "${BUILD_DIR}/result.txt" "PASS")
else()
    message(STATUS "FAIL: build failed for ${TARGET_NAME}")
    message(STATUS "${build_error}")
    file(WRITE "${BUILD_DIR}/result.txt" "FAIL")
endif()

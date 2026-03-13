include_directories(${CMAKE_SOURCE_DIR}/src)

# ── Unit tests (native) ──────────────────────────────────────────────────

add_executable(test_crc test_crc.c)
target_link_libraries(test_crc sdp)
add_test(NAME test_crc COMMAND test_crc)

add_executable(test_e2e test_e2e.c)
target_link_libraries(test_e2e sdp)
add_test(NAME test_e2e COMMAND test_e2e)

# ── Unix domain socket e2e (POSIX only) ───────────────────────────────────

if(UNIX)
    add_executable(test_e2e_socket test_e2e_socket.c)
    target_link_libraries(test_e2e_socket sdp)
    find_package(Threads REQUIRED)
    target_link_libraries(test_e2e_socket Threads::Threads)
    add_test(NAME test_e2e_socket COMMAND test_e2e_socket)
endif()

# ── Cross-compile compile-only tests ──────────────────────────────────────
# Each test invokes the cross-compiler directly on sdp.c.
# If the compiler is not installed the test outputs SKIP and exits 0.
# PASS_REGULAR_EXPRESSION ensures CTest marks SKIP as pass.

set(CROSS_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/cross_compile_test.sh")
set(SRC_DIR      "${CMAKE_SOURCE_DIR}/src")

macro(add_cross_compile_test NAME COMPILER CFLAGS)
    add_test(
        NAME    cross_compile_${NAME}
        COMMAND ${CROSS_SCRIPT} ${COMPILER} "${CFLAGS}" ${SRC_DIR} ${NAME}
    )
    set_tests_properties(cross_compile_${NAME} PROPERTIES
        LABELS "cross_compile"
        PASS_REGULAR_EXPRESSION "(PASS|SKIP)"
    )
endmacro()

add_cross_compile_test(arm-none-eabi-cm4    arm-none-eabi-gcc       "-mcpu=cortex-m4 -mthumb")
add_cross_compile_test(aarch64-apple-m4     cc                      "-arch arm64 -mcpu=apple-m4")
add_cross_compile_test(avr-attiny1616        avr-gcc                 "-mmcu=attiny1616 -Os")
add_cross_compile_test(pic-xc8              xc8-cc                  "-mcpu=18F4520")
add_cross_compile_test(xtensa-esp32-u4wdh   xtensa-esp32-elf-gcc    "-mlongcalls -mtext-section-literals")
add_cross_compile_test(x86_64-linux-musl    x86_64-linux-musl-gcc   "")

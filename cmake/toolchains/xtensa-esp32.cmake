set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR xtensa)

# Espressif ESP32 Xtensa toolchain (installed via esp-idf or espressif/tools)
find_program(CMAKE_C_COMPILER
    NAMES xtensa-esp32-elf-gcc xtensa-esp-elf-gcc
    HINTS
        "$ENV{HOME}/.espressif/tools/xtensa-esp-elf/esp-13.2.0_20230928/xtensa-esp-elf/bin"
        "/opt/espressif/tools/xtensa-esp-elf/esp-13.2.0_20230928/xtensa-esp-elf/bin"
)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_FLAGS_INIT "-mlongcalls")

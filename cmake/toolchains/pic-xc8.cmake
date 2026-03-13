set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR pic)

# Microchip XC8 compiler for PIC
find_program(CMAKE_C_COMPILER NAMES xc8-cc xc8)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# PIC18F4520 — representative 8-bit PIC target
set(CMAKE_C_FLAGS_INIT "-mcpu=18F4520")

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

# 64-bit x86 Linux cross-compiler (musl or glibc)
find_program(CMAKE_C_COMPILER
    NAMES x86_64-linux-musl-gcc x86_64-linux-gnu-gcc
)

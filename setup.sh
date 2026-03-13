#!/bin/sh
# setup.sh — install SDP build and cross-compile toolchain dependencies
#
# Supported:
#   macOS  : Homebrew (brew)
#   Ubuntu/Debian : apt
#   RHEL/Fedora   : dnf
#
# Usage: ./setup.sh

set -e

detect_os() {
    if [ "$(uname)" = "Darwin" ]; then
        echo "macos"
    elif [ -f /etc/debian_version ]; then
        echo "debian"
    elif [ -f /etc/fedora-release ] || [ -f /etc/redhat-release ]; then
        echo "redhat"
    else
        echo "unknown"
    fi
}

OS=$(detect_os)
echo "Detected OS: $OS"

case "$OS" in

macos)
    if ! command -v brew > /dev/null 2>&1; then
        echo "Homebrew not found. Install from https://brew.sh and re-run this script."
        exit 1
    fi
    echo "Installing macOS tools via Homebrew..."
    brew install \
        cmake \
        ninja \
        arm-none-eabi-gcc \
        avr-gcc \
        sdcc

    # ESP32 Xtensa toolchain via Espressif IDF or tap
    if brew tap | grep -q "espressif"; then
        brew install espressif/esp/xtensa-esp32-elf
    else
        echo ""
        echo "NOTE: ESP32 Xtensa compiler not installed."
        echo "  To install manually:"
        echo "    brew tap espressif/esp && brew install xtensa-esp32-elf"
        echo "  Or install ESP-IDF from https://docs.espressif.com/projects/esp-idf/"
    fi

    # Microchip XC8 (PIC compiler) is not available via Homebrew.
    if ! command -v xc8-cc > /dev/null 2>&1; then
        echo ""
        echo "NOTE: Microchip XC8 (PIC compiler) not installed."
        echo "  Download from: https://www.microchip.com/en-us/tools-resources/develop/mplab-xc-compilers/xc8"
    fi

    # x86-64 Linux musl cross-compiler
    if command -v x86_64-linux-musl-gcc > /dev/null 2>&1; then
        echo "x86_64-linux-musl-gcc already installed."
    else
        echo ""
        echo "NOTE: x86_64-linux-musl-gcc not installed."
        echo "  To install: brew install filosottile/musl-cross/musl-cross"
    fi
    ;;

debian)
    echo "Installing Debian/Ubuntu tools via apt..."
    sudo apt-get update
    sudo apt-get install -y \
        cmake \
        ninja-build \
        gcc-arm-none-eabi \
        avr-gcc \
        avr-libc \
        sdcc \
        gcc-multilib \
        musl-tools

    # ESP32 Xtensa (not in standard apt repos)
    if ! command -v xtensa-esp32-elf-gcc > /dev/null 2>&1; then
        echo ""
        echo "NOTE: ESP32 Xtensa compiler not installed."
        echo "  Install via ESP-IDF: https://docs.espressif.com/projects/esp-idf/"
        echo "  Or: pip install esptool && install xtensa-esp-elf from Espressif releases."
    fi

    # Microchip XC8
    if ! command -v xc8-cc > /dev/null 2>&1; then
        echo ""
        echo "NOTE: Microchip XC8 (PIC compiler) not installed."
        echo "  Download from: https://www.microchip.com/en-us/tools-resources/develop/mplab-xc-compilers/xc8"
    fi
    ;;

redhat)
    echo "Installing RHEL/Fedora tools via dnf..."
    sudo dnf install -y \
        cmake \
        ninja-build \
        arm-none-eabi-gcc \
        avr-gcc \
        sdcc \
        musl-gcc

    # ESP32 Xtensa
    if ! command -v xtensa-esp32-elf-gcc > /dev/null 2>&1; then
        echo ""
        echo "NOTE: ESP32 Xtensa compiler not installed."
        echo "  Install via ESP-IDF: https://docs.espressif.com/projects/esp-idf/"
    fi

    # Microchip XC8
    if ! command -v xc8-cc > /dev/null 2>&1; then
        echo ""
        echo "NOTE: Microchip XC8 (PIC compiler) not installed."
        echo "  Download from: https://www.microchip.com/en-us/tools-resources/develop/mplab-xc-compilers/xc8"
    fi
    ;;

*)
    echo "Unsupported OS. Please install the following manually:"
    echo "  cmake, gcc-arm-none-eabi, avr-gcc, xtensa-esp32-elf-gcc, xc8-cc"
    exit 1
    ;;

esac

echo ""
echo "Build and test:"
echo "  cmake -B build -S . && cmake --build build && cd build && ctest"

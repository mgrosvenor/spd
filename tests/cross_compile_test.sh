#!/bin/sh
# cross_compile_test.sh <compiler> <cflags> <src_dir> <target_name>
# Exit codes: 0=pass (or skip), 1=fail
# Outputs: SKIP / PASS / FAIL prefix

set -e

COMPILER="$1"
CFLAGS="$2"
SRC_DIR="$3"
TARGET_NAME="$4"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CROSS_HEADERS="${SCRIPT_DIR}/cross-headers"

# Skip gracefully if compiler not found
if ! command -v "${COMPILER}" > /dev/null 2>&1; then
    echo "SKIP: ${TARGET_NAME}: compiler '${COMPILER}' not found"
    exit 0
fi

TMPDIR_OUT=$(mktemp -d)
trap 'rm -rf "${TMPDIR_OUT}"' EXIT

# Generate a minimal translation unit that instantiates the implementation.
IMPL_FILE="${TMPDIR_OUT}/sdp_impl.c"
printf '#define SDP_IMPLEMENTATION\n#include "sdp.h"\n' > "${IMPL_FILE}"

# First attempt: standard headers
# shellcheck disable=SC2086
if ${COMPILER} ${CFLAGS} -std=c99 -Wall -Wextra -Wpedantic \
    -I"${SRC_DIR}" \
    -c "${IMPL_FILE}" \
    -o "${TMPDIR_OUT}/sdp_impl.o" \
    > "${TMPDIR_OUT}/build.log" 2>&1; then
    echo "PASS: ${TARGET_NAME}"
    exit 0
fi

# Second attempt: with cross-header shim (for bare-metal toolchains without libc)
# shellcheck disable=SC2086
if ${COMPILER} ${CFLAGS} -std=c99 -Wall -Wextra -Wpedantic \
    -nostdinc \
    -I"${CROSS_HEADERS}" \
    -I"$(${COMPILER} -print-file-name=include 2>/dev/null)" \
    -I"${SRC_DIR}" \
    -c "${IMPL_FILE}" \
    -o "${TMPDIR_OUT}/sdp_impl.o" \
    > "${TMPDIR_OUT}/build2.log" 2>&1; then
    echo "PASS: ${TARGET_NAME} (with cross-header shim)"
    exit 0
fi

echo "FAIL: ${TARGET_NAME}"
cat "${TMPDIR_OUT}/build.log"
echo "--- with shim ---"
cat "${TMPDIR_OUT}/build2.log"
exit 1

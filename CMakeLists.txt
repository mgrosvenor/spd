cmake_minimum_required(VERSION 3.13)
project(spd
    VERSION 1.1.0
    DESCRIPTION "Simple Datagram Protocol — lightweight framed serial transport"
    LANGUAGES C
)

# ── Library target (header-only) ─────────────────────────────────────────────
#
# Consumers add one translation unit containing:
#     #define SDP_IMPLEMENTATION
#     #include "sdp.h"
#
# All other TUs include "sdp.h" for declarations only.
#

add_library(sdp INTERFACE)
add_library(sdp::sdp ALIAS sdp)

target_include_directories(sdp
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include/sdp>
)

target_compile_features(sdp INTERFACE c_std_99)

# Optional: override sliding window size (default 2, range 1–7)
if(DEFINED SDP_WINDOW)
    target_compile_definitions(sdp INTERFACE SDP_WINDOW=${SDP_WINDOW})
endif()

# Optional: override silence timeout in ms (default 30 000)
if(DEFINED SDP_SILENCE_TIMEOUT_MS)
    target_compile_definitions(sdp INTERFACE SDP_SILENCE_TIMEOUT_MS=${SDP_SILENCE_TIMEOUT_MS})
endif()

# ── Install rules ────────────────────────────────────────────────────────────

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS sdp
    EXPORT sdpTargets
)

install(FILES
    src/sdp.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/sdp
)

install(EXPORT sdpTargets
    FILE      sdpTargets.cmake
    NAMESPACE sdp::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sdp
)

configure_package_config_file(
    cmake/sdpConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/sdpConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sdp
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/sdpConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/sdpConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/sdpConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sdp
)

# ── Tests (only when this is the top-level project) ──────────────────────────

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    enable_testing()
    add_subdirectory(tests)
endif()

# Standalone configuration: no compiler, BSP checkout or generated board needed.
# cmake -DPNX_BOARD=h723_dmmc02 -DOUT_DIR=build/config-preview -P configs/cmake/configure.cmake
cmake_minimum_required(VERSION 3.23)
if(NOT DEFINED OUT_DIR OR OUT_DIR STREQUAL "")
    message(FATAL_ERROR "OUT_DIR is required; use a build-local output directory")
endif()
include("${CMAKE_CURRENT_LIST_DIR}/load_board.cmake")
set(IOC "${PNX_BOARD_IOC}")
set(BOARD_CONFIG "${PNX_BOARD_CONFIG}")
set(PARAMS "${PNX_BOARD_PARAMS}")
set(ROBOT_CONFIG "${PNX_BOARD_ROBOT_CONFIG}")
include("${CMAKE_CURRENT_LIST_DIR}/generate_config.cmake")

# Export the explicit CubeMX destination; never invoke regeneration on configure.
get_filename_component(repository "${PNX_CONFIG_DIR}/.." ABSOLUTE)
file(CONFIGURE OUTPUT "${OUT_DIR}/cubemx.script" CONTENT
    "config load \"${PNX_BOARD_IOC}\"\nproject path \"${repository}/boards/${PNX_BOARD}\"\nproject generate\nexit\n" @ONLY)

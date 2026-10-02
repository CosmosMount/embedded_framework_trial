cmake_minimum_required(VERSION 3.23)

include("${CMAKE_CURRENT_SOURCE_DIR}/configs/cmake/load_board.cmake")
# Generated source is consumed here, never used as a configuration store.
set(PNX_BOARD_CUBEMX_DIR "${CMAKE_CURRENT_SOURCE_DIR}/boards/${PNX_BOARD}/cmake/stm32cubemx")

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/configs/cmake/gcc-arm-none-eabi.cmake"
        CACHE FILEPATH "ARM toolchain for the selected board")
endif()

project(pnx_embedded LANGUAGES C CXX ASM)

# Debug tools follow the same board selection as the firmware.
file(CONFIGURE OUTPUT "${CMAKE_BINARY_DIR}/pnx-openocd.cfg"
    CONTENT "source [find ${PNX_BOARD_OPENOCD_TARGET}]\n" @ONLY)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PNX_EMBEDDED_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/configs")
set(PNX_EMBEDDED_GENERATED_DIR "${CMAKE_BINARY_DIR}/generated")
if(NOT EXISTS "${PNX_BOARD_CUBEMX_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
        "Generated CubeMX target missing: ${PNX_BOARD_CUBEMX_DIR}/CMakeLists.txt. "
        "Configuration can still be checked with configs/cmake/configure.cmake.")
endif()

set(PNX_BSP_SOURCE_DIR "" CACHE PATH "Development override for the selected board BSP")
if(NOT PNX_BSP_SOURCE_DIR)
    set(PNX_BSP_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/bsp/${PNX_BOARD}")
endif()
mark_as_advanced(PNX_BSP_SOURCE_DIR)
get_filename_component(PNX_BSP_SOURCE_DIR "${PNX_BSP_SOURCE_DIR}" ABSOLUTE)

set(PNX_BSP_TARGET_FILE "${PNX_BSP_SOURCE_DIR}/target.cmake")
if(NOT EXISTS "${PNX_BSP_TARGET_FILE}")
    message(FATAL_ERROR
        "Selected BSP has no target manifest: ${PNX_BSP_TARGET_FILE}")
endif()
unset(PNX_BSP_FAMILY)
include("${PNX_BSP_TARGET_FILE}")
if(NOT DEFINED PNX_BSP_FAMILY OR NOT PNX_BSP_FAMILY STREQUAL PNX_BOARD_FAMILY)
    message(FATAL_ERROR
        "PNX_BOARD='${PNX_BOARD}' requires BSP family '${PNX_BOARD_FAMILY}', "
        "but '${PNX_BSP_SOURCE_DIR}' provides '${PNX_BSP_FAMILY}'. "
        "Select the matching board BSP directory; PNX_BSP_SOURCE_DIR is only a development override.")
endif()
message(STATUS "PNX board: ${PNX_BOARD} (${PNX_BOARD_FAMILY}); BSP: ${PNX_BSP_SOURCE_DIR}")

set(PNX_ROBOT_CONFIG_OVERRIDE "" CACHE FILEPATH
    "Validation/development override for the selected board's robot configuration")
mark_as_advanced(PNX_ROBOT_CONFIG_OVERRIDE)
if(PNX_ROBOT_CONFIG_OVERRIDE)
    if(NOT EXISTS "${PNX_ROBOT_CONFIG_OVERRIDE}")
        message(FATAL_ERROR "PNX_ROBOT_CONFIG_OVERRIDE does not exist: ${PNX_ROBOT_CONFIG_OVERRIDE}")
    endif()
    get_filename_component(PNX_EMBEDDED_ROBOT_CONFIG "${PNX_ROBOT_CONFIG_OVERRIDE}" ABSOLUTE)
else()
    set(PNX_EMBEDDED_ROBOT_CONFIG "${PNX_BOARD_ROBOT_CONFIG}")
endif()
set(PNX_PARAMS_OVERRIDE "" CACHE FILEPATH "Validation/development params override")
mark_as_advanced(PNX_PARAMS_OVERRIDE)
if(PNX_PARAMS_OVERRIDE)
    get_filename_component(PNX_BOARD_PARAMS "${PNX_PARAMS_OVERRIDE}" ABSOLUTE)
endif()
set(PNX_EMBEDDED_BOARD_CONFIG "${PNX_BOARD_CONFIG}")

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${PNX_BOARD_IOC}"
    "${PNX_EMBEDDED_BOARD_CONFIG}"
    "${PNX_BOARD_CONFIG_DIR}/board.cmake"
    "${PNX_BOARD_LINKER_SCRIPT}"
    "${PNX_BOARD_PARAMS}"
    "${PNX_EMBEDDED_ROBOT_CONFIG}"
    "${PNX_EMBEDDED_CONFIG_DIR}/defaults.json"
    "${PNX_EMBEDDED_CONFIG_DIR}/cmake/generate_config.cmake"
    "${PNX_EMBEDDED_CONFIG_DIR}/cmake/import_ioc.cmake"
    "${PNX_BSP_TARGET_FILE}"
)

set(IOC "${PNX_BOARD_IOC}")
set(BOARD_CONFIG "${PNX_EMBEDDED_BOARD_CONFIG}")
set(PARAMS "${PNX_BOARD_PARAMS}")
set(ROBOT_CONFIG "${PNX_EMBEDDED_ROBOT_CONFIG}")
set(OUT_DIR "${PNX_EMBEDDED_GENERATED_DIR}")
include("${PNX_EMBEDDED_CONFIG_DIR}/cmake/generate_config.cmake")
unset(IOC)
unset(BOARD_CONFIG)
unset(PARAMS)
unset(ROBOT_CONFIG)
unset(OUT_DIR)

if(PNX_BOARD_FAMILY STREQUAL "stm32f4")
    if(adc_channel_count GREATER 0)
        message(FATAL_ERROR "F407 BSP does not provide ADC")
    endif()
endif()

function(pnx_exclude_bsp_module source_list module_name)
    file(TO_CMAKE_PATH "${PNX_BSP_SOURCE_DIR}/${module_name}/" module_prefix)
    set(filtered_sources)
    foreach(source IN LISTS ${source_list})
        file(TO_CMAKE_PATH "${source}" normalized_source)
        string(FIND "${normalized_source}" "${module_prefix}" module_position)
        if(NOT module_position EQUAL 0)
            list(APPEND filtered_sources "${source}")
        endif()
    endforeach()
    set(${source_list} "${filtered_sources}" PARENT_SCOPE)
endfunction()

file(GLOB_RECURSE PNX_EMBEDDED_SOURCES CONFIGURE_DEPENDS
    "${PNX_BSP_SOURCE_DIR}/*.c"
    "${PNX_BSP_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/devices/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/modules/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/libs/*.cpp"
)

list(APPEND PNX_EMBEDDED_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/app/app.cpp")

if(NOT ENABLE_USBX)
    pnx_exclude_bsp_module(PNX_EMBEDDED_SOURCES usb)
endif()

if(NOT HAS_DMIMU)
    list(FILTER PNX_EMBEDDED_SOURCES EXCLUDE REGEX "[/\\\\]devices[/\\\\]imu[/\\\\]dmimu[/\\\\]")
endif()

# Driver families share the same directory convention; keep selection explicit.
foreach(family IN ITEMS DJI DM LK XV2)
    if(NOT MOTOR_${family})
        string(TOLOWER "${family}" directory)
        list(FILTER PNX_EMBEDDED_SOURCES EXCLUDE REGEX "/devices/motors/${directory}/")
    endif()
endforeach()

foreach(protocol IN ITEMS DR16 VT03 PS2)
    if(NOT ENABLE_${protocol})
        string(TOLOWER "${protocol}" filename)
        list(FILTER PNX_EMBEDDED_SOURCES EXCLUDE REGEX "/devices/remotes/src/${filename}\\.cpp$")
    endif()
endforeach()
if(NOT ENABLE_PS2)
    list(FILTER PNX_EMBEDDED_SOURCES EXCLUDE REGEX "/modules/remoter/src/gamepad\\.cpp$")
endif()
if(robot_motor_count EQUAL 0)
    list(FILTER PNX_EMBEDDED_SOURCES EXCLUDE REGEX "[/\\\\]devices[/\\\\]motors[/\\\\]")
endif()

# The LED device is the SPI WS2812 consumer. GPIO indicators use BSP GPIO.
if(NOT HAS_LED)
    list(FILTER PNX_EMBEDDED_SOURCES EXCLUDE REGEX "/devices/led/")
endif()

if(NOT HAS_BMI088)
    list(FILTER PNX_EMBEDDED_SOURCES EXCLUDE REGEX "[/\\\\]devices[/\\\\]imu[/\\\\]bmi088[/\\\\]")
endif()

if(NOT HAS_AHRS AND NOT HAS_DMIMU)
    list(FILTER PNX_EMBEDDED_SOURCES EXCLUDE REGEX "[/\\\\]modules[/\\\\]ahrs[/\\\\]")
endif()

if(pwm_channel_count EQUAL 0)
    pnx_exclude_bsp_module(PNX_EMBEDDED_SOURCES pwm)
endif()

if(spi_bus_count EQUAL 0)
    pnx_exclude_bsp_module(PNX_EMBEDDED_SOURCES spi)
endif()

if(adc_channel_count EQUAL 0)
    pnx_exclude_bsp_module(PNX_EMBEDDED_SOURCES adc)
endif()

file(GLOB_RECURSE PNX_EMBEDDED_HEADERS CONFIGURE_DEPENDS
    "${PNX_BSP_SOURCE_DIR}/*.h"
    "${PNX_BSP_SOURCE_DIR}/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/devices/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/devices/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/modules/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/modules/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/libs/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/libs/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/app/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/app/*.hpp"
)

if(NOT HAS_DMIMU)
    list(FILTER PNX_EMBEDDED_HEADERS EXCLUDE REGEX "[/\\\\]devices[/\\\\]imu[/\\\\]dmimu[/\\\\]")
endif()

set(PNX_EMBEDDED_INCLUDE_DIRS)
set(PNX_BSP_PRIVATE_INCLUDE_DIRS)
foreach(header IN LISTS PNX_EMBEDDED_HEADERS)
    get_filename_component(header_dir "${header}" DIRECTORY)
    cmake_path(IS_PREFIX PNX_BSP_SOURCE_DIR "${header}" NORMALIZE is_bsp_header)
    if(is_bsp_header AND header_dir MATCHES "/src$")
        list(APPEND PNX_BSP_PRIVATE_INCLUDE_DIRS "${header_dir}")
    else()
        list(APPEND PNX_EMBEDDED_INCLUDE_DIRS "${header_dir}")
    endif()
endforeach()
list(REMOVE_DUPLICATES PNX_EMBEDDED_INCLUDE_DIRS)
list(REMOVE_DUPLICATES PNX_BSP_PRIVATE_INCLUDE_DIRS)
foreach(source IN LISTS PNX_EMBEDDED_SOURCES)
    cmake_path(IS_PREFIX PNX_BSP_SOURCE_DIR "${source}" NORMALIZE is_bsp_source)
    if(is_bsp_source)
        set_property(SOURCE "${source}" APPEND PROPERTY
            INCLUDE_DIRECTORIES "${PNX_BSP_PRIVATE_INCLUDE_DIRS}")
    endif()
endforeach()
set_property(SOURCE "${PNX_EMBEDDED_GENERATED_DIR}/bsp_bindings.cpp" APPEND PROPERTY
    INCLUDE_DIRECTORIES "${PNX_BSP_PRIVATE_INCLUDE_DIRS}")

set(CMSIS_DSP_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/cmsis-dsp/Source/BasicMathFunctions/arm_dot_prod_f32.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmsis-dsp/Source/MatrixFunctions/arm_mat_add_f32.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmsis-dsp/Source/MatrixFunctions/arm_mat_init_f32.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmsis-dsp/Source/MatrixFunctions/arm_mat_inverse_f32.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmsis-dsp/Source/MatrixFunctions/arm_mat_mult_f32.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmsis-dsp/Source/MatrixFunctions/arm_mat_sub_f32.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmsis-dsp/Source/MatrixFunctions/arm_mat_trans_f32.c"
)

add_executable(${PROJECT_NAME})
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY LINK_DEPENDS "${PNX_BOARD_LINKER_SCRIPT}")

add_subdirectory("${PNX_BOARD_CUBEMX_DIR}" "${CMAKE_BINARY_DIR}/stm32cubemx")
if(PNX_BOARD_FAMILY STREQUAL "stm32f4")
    # Preserve F4 stack checking consistently in ThreadX and its consumers.
    target_compile_definitions(stm32cubemx INTERFACE TX_ENABLE_STACK_CHECKING)
endif()
# CubeMX owns ThreadX startup; app_start integration remains a separate task.
include("${PNX_EMBEDDED_CONFIG_DIR}/cmake/usbx_integration.cmake")

target_sources(${PROJECT_NAME} PRIVATE
    ${PNX_EMBEDDED_SOURCES}
    ${CMSIS_DSP_SOURCES}
    "${PNX_EMBEDDED_GENERATED_DIR}/bsp_bindings.cpp"
)

# Keep the build-local product source selection inspectable without turning the
# source tree into a second generated-configuration location.
set(PNX_EMBEDDED_SOURCE_MANIFEST "${CMAKE_BINARY_DIR}/pnx_embedded_sources.txt")
list(JOIN PNX_EMBEDDED_SOURCES "\n" PNX_EMBEDDED_SOURCE_MANIFEST_BODY)
list(JOIN CMSIS_DSP_SOURCES "\n" PNX_EMBEDDED_CMSIS_SOURCE_MANIFEST_BODY)
string(CONCAT pnx_source_manifest_content
    "# Sources selected directly by the PNX product target. Generated during configure.\n"
    "${PNX_EMBEDDED_SOURCE_MANIFEST_BODY}\n"
    "${PNX_EMBEDDED_CMSIS_SOURCE_MANIFEST_BODY}\n"
    "${PNX_EMBEDDED_GENERATED_DIR}/bsp_bindings.cpp\n")
file(CONFIGURE OUTPUT "${PNX_EMBEDDED_SOURCE_MANIFEST}" CONTENT "${pnx_source_manifest_content}" @ONLY)
message(STATUS "Product source manifest: ${PNX_EMBEDDED_SOURCE_MANIFEST}")

target_include_directories(${PROJECT_NAME} PRIVATE
    ${PNX_EMBEDDED_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/cmsis-dsp/Include"
    "${PNX_EMBEDDED_GENERATED_DIR}"
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    PNX_DMA_DEDICATED_SECTION=${PNX_DMA_DEDICATED_SECTION}
    PNX_DMA_CACHE_LINE_SIZE=${PNX_DMA_CACHE_LINE_SIZE}
    ROBOT_MOTOR_COUNT=${robot_motor_count}
)

target_link_libraries(${PROJECT_NAME}
    stm32cubemx
)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

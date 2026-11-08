cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME J_ESC)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Generate into the build tree; checked-in files support manual/offline inspection.
find_package(Python3 3.10 REQUIRED COMPONENTS Interpreter)
set(OD_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/canopen_generated")
add_custom_command(
    OUTPUT "${OD_GENERATED_DIR}/od_generated.stamp"
    BYPRODUCTS "${OD_GENERATED_DIR}/od_generated.c" "${OD_GENERATED_DIR}/od_generated.h"
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_od.py"
        --csv "${CMAKE_CURRENT_SOURCE_DIR}/canopen/object_dictionary.csv"
        --out-dir "${OD_GENERATED_DIR}"
    COMMAND "${CMAKE_COMMAND}" -E touch "${OD_GENERATED_DIR}/od_generated.stamp"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_od.py"
        "${CMAKE_CURRENT_SOURCE_DIR}/canopen/object_dictionary.csv"
    VERBATIM
)
add_custom_target(generate_od DEPENDS "${OD_GENERATED_DIR}/od_generated.stamp")
add_dependencies(${CMAKE_PROJECT_NAME} generate_od)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    canopen/od.c canopen/od_user.c canopen/od_protocol.c canopen/od_cdc.c canopen/od_commands.c
    canopen/od_wire.c canopen/od_scope.c canopen/od_motion.c
    "${OD_GENERATED_DIR}/od_generated.c")
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    "${OD_GENERATED_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/canopen")

# These files execute in the 20 kHz ISR. Unoptimized typed sampling exceeds
# 50 us; active FOC plus synchronous encoder traffic also needs headroom.
# Keep the rest of Debug at -O0. Locals in these files may be optimized out.
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    set_property(SOURCE canopen/od.c canopen/od_scope.c
        motor_control/motor.c motor_control/control/foc.c
        motor_control/encoder/encoder.c motor_control/encoder/as5048a/as5048a.c
        motor_control/hal/encoder_spi.c APPEND PROPERTY COMPILE_OPTIONS -O2)
endif()

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/hal/inverter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/hal/realtime.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/hal/rand.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/hal/encoder_spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/hal/encoder_rs485.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/hal/config_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/config/config_storage.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/control/dc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/control/foc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/control/hfi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/encoder/as5048a/as5048a.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/encoder/hcfa/hcfa.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/encoder/tamagawa/tamagawa.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/encoder/encoder.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/math/angle_utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/math/compliance_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/math/filter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/math/foc_utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/math/motion_planner.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/math/my_math.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/math/pid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/math/pll.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/math/sine_generator.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/math/utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/math/vector_utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/observer/observer.c
    

    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/global.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/motor_api.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/motor.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/hal
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/config
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/control
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/encoder
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/encoder/as5048a
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/encoder/hcfa
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/encoder/tamagawa
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/math
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_control/observer
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    HW_VESC_6_4
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Convert the linked firmware to flashing images in the build directory.
# Resolve objcopy beside the compiler as well as on PATH.
if(IS_ABSOLUTE "${CMAKE_OBJCOPY}")
    set(FIRMWARE_OBJCOPY "${CMAKE_OBJCOPY}")
else()
    get_filename_component(FIRMWARE_TOOLCHAIN_DIR "${CMAKE_C_COMPILER}" DIRECTORY)
    find_program(FIRMWARE_OBJCOPY NAMES "${CMAKE_OBJCOPY}"
        HINTS "${FIRMWARE_TOOLCHAIN_DIR}" REQUIRED)
endif()

set(FIRMWARE_BIN "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin")
set(FIRMWARE_HEX "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex")
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND "${FIRMWARE_OBJCOPY}" -O binary
        "$<TARGET_FILE:${CMAKE_PROJECT_NAME}>" "${FIRMWARE_BIN}"
    COMMAND "${FIRMWARE_OBJCOPY}" -O ihex
        "$<TARGET_FILE:${CMAKE_PROJECT_NAME}>" "${FIRMWARE_HEX}"
    BYPRODUCTS "${FIRMWARE_BIN}" "${FIRMWARE_HEX}"
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.bin and ${CMAKE_PROJECT_NAME}.hex"
    VERBATIM
)

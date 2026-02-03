# CMakeLists.txt for E-Week 2026 Robot Controller
# Target: Raspberry Pi Pico (RP2040)

cmake_minimum_required(VERSION 3.13)

# C/C++ standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialize Pico SDK from installed location
# Note: Can come from environment variable, CMake cache, or pico-vscode extension

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
  set(USERHOME $ENV{USERPROFILE})
else()
  set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
  include(${picoVscode})
endif()
# ====================================================================================

# Set board type
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Project definition
project(eweek_2026
  VERSION 1.0.0
  DESCRIPTION "E-Week 2026 Robot Controller"
  LANGUAGES C CXX ASM
)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# Define source files
set(SOURCES
  src/main.cpp
  src/error_handler.cpp
  src/ring_buffer.cpp
  src/flysky_ibus.cpp
  src/drive_train.cpp
  src/drive_train_differential.cpp
  src/drive_train_mecanum.cpp
  src/mech_collect.cpp
  src/mech_deposit.cpp
  src/mech_launcher.cpp
  src/motor_driver_drv8833.cpp
  src/motor_driver_l298n.cpp
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Set program metadata
pico_set_program_name(${PROJECT_NAME} "eweek_2026")
pico_set_program_version(${PROJECT_NAME} "1.0.0")

# Configure stdio outputs
# Enable for debug, disable for production
pico_enable_stdio_uart(${PROJECT_NAME} 1)
pico_enable_stdio_usb(${PROJECT_NAME} 1)

# Add include directories
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/inc
)

# Link required libraries
target_link_libraries(${PROJECT_NAME}
  pico_stdlib
  hardware_pwm
  hardware_timer
  hardware_clocks
  hardware_watchdog
)

# Compiler warnings
target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall
  -Wextra
  -Werror=return-type
)

# Generate UF2, HEX, and other output formats
pico_add_extra_outputs(${PROJECT_NAME})

# Print build info
message(STATUS "")
message(STATUS "=== E-Week 2026 Build Configuration ===")
message(STATUS "Project:     ${PROJECT_NAME}")
message(STATUS "Version:     ${PROJECT_VERSION}")
message(STATUS "Board:       ${PICO_BOARD}")
message(STATUS "Build type:  ${CMAKE_BUILD_TYPE}")
message(STATUS "========================================")
message(STATUS "")

# EOF

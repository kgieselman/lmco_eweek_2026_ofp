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


#===============================================================================
# Git Version Information
#===============================================================================

# Find Git executable
find_package(Git QUIET)

if(GIT_FOUND AND EXISTS "${CMAKE_SOURCE_DIR}/.git")
  # Get short commit hash (7 characters)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short=7 HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )

  # Get full commit hash (40 characters)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_HASH_FULL
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )

  # Get branch name
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )

  # Get tag if HEAD is tagged
  execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --tags --exact-match HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_TAG
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )

  # Check if working directory is dirty
  execute_process(
    COMMAND ${GIT_EXECUTABLE} status --porcelain
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_STATUS
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  if(GIT_STATUS)
    set(GIT_IS_DIRTY 1)
  else()
    set(GIT_IS_DIRTY 0)
  endif()

  # Get commit count
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-list --count HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_COUNT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )

  # Get describe output (tag-count-hash or just hash if no tags)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --tags --always --dirty
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_DESCRIBE
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )

else()
  # Git not found or not a git repo - use defaults
  set(GIT_COMMIT_HASH "unknown")
  set(GIT_COMMIT_HASH_FULL "unknown")
  set(GIT_BRANCH "unknown")
  set(GIT_TAG "")
  set(GIT_IS_DIRTY 0)
  set(GIT_COMMIT_COUNT "0")
  set(GIT_DESCRIBE "unknown")
  message(WARNING "Git not found or not a git repository. Version info will be incomplete.")
endif()

# Configure version header from template
configure_file(
  ${CMAKE_SOURCE_DIR}/inc/version.h.in
  ${CMAKE_BINARY_DIR}/generated/version.h
  @ONLY
)

# Print version info during configuration
message(STATUS "")
message(STATUS "=== Git Version Info ===")
message(STATUS "Commit:    ${GIT_COMMIT_HASH}")
message(STATUS "Branch:    ${GIT_BRANCH}")
message(STATUS "Tag:       ${GIT_TAG}")
message(STATUS "Dirty:     ${GIT_IS_DIRTY}")
message(STATUS "Describe:  ${GIT_DESCRIBE}")
message(STATUS "========================")
message(STATUS "")

# Define source files
set(SOURCES
  src/main.cpp
  src/error_handler.cpp
  src/flysky_ibus.cpp
  src/drive_train_differential.cpp
  src/motor_driver.cpp
  src/mech_collect.cpp
  src/mech_deposit.cpp
  src/mech_launcher.cpp
  src/ssd1306_display.cpp
  src/display_view.cpp
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
  ${CMAKE_BINARY_DIR}/generated
)

# Link required libraries
target_link_libraries(${PROJECT_NAME}
  pico_stdlib
  pico_multicore
  hardware_pwm
  hardware_timer
  hardware_clocks
  hardware_watchdog
  hardware_i2c
  hardware_sync
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
message(STATUS "Commit:      ${GIT_COMMIT_HASH}")
message(STATUS "Board:       ${PICO_BOARD}")
message(STATUS "Build type:  ${CMAKE_BUILD_TYPE}")
message(STATUS "========================================")
message(STATUS "")

# EOF

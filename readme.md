# Lockheed Martin E-Week Competition 2026

## Overview

This project provides the operational flight program (OFP) for a remote-controlled
vehicle competing in the 2026 Lockheed Martin E-Week competition. The vehicle must
collect and deposit ping pong balls and cabbages, as well as launch ping pong balls
at targets.

**Target Platform:** Raspberry Pi Pico (RP2040)

## Competition Objectives

### Collect
- Each lap, 4 ping pong balls can be deposited, so at least 4 should be collected
- More can be collected and held for multiple laps

### Deposit
- Each lap, up to 4 ping pong balls can be deposited
- Any excess will be returned to the main ball pit
- Ideally only 4 will be deposited per lap

### Launch
- Each lap, up to 4 ping pong balls can be launched toward targets
- These are separate ping pong balls from those being collected
- Balls are manually fed by a teammate near the ball pit

## Architecture

### Source Modules

| Module | Header | Description |
|--------|--------|-------------|
| Motor Driver | `motor_driver.h` | Dual H-bridge driver (1-PWM + 2-DIR wiring, e.g. L298N, BTS7960) |
| Differential Drive | `drive_train_differential.h` | Two-motor tank-style drive with speed/turn mixing |
| FlySky iBUS | `flysky_ibus.h` | RC receiver protocol interface (115200 baud, 14 channels) |
| Scoop | `mech_scoop.h` | Servo-driven scoop mechanism for ball collection |
| Launcher | `mech_launcher.h` | Stepper feeder (DRV8825 via PIO) + dual flywheel launcher |
| Display View | `display_view.h` | Robot telemetry HUD on SSD1306 OLED (runs on core 1) |
| SSD1306 Driver | `ssd1306_display.h` | Low-level I2C OLED driver with 5×7 font framebuffer |
| Error Handler | `error_handler.h` | Error codes, reporting, and debug output utilities |
| Config | `config.h` | Compile-time feature flags and timing parameters |
| Pinout | `pinout.h` | All GPIO pin assignments |

### Dual-Core Usage

- **Core 0:** Main control loop (RC input, motor mixing, mechanism control, watchdog)
- **Core 1:** SSD1306 OLED display refresh (~20 FPS) with lock-free shared data from core 0

## Building

### Prerequisites

1. Raspberry Pi Pico SDK (v2.2.0 or later)
2. CMake (v3.13 or later)
3. ARM GCC toolchain
4. (Optional) VS Code with Pico extension
    + Open VS Code in a non-repo
    + Use Pico Extension to import the cloned project


### Build Steps

```bash
# Create build directory
mkdir build && cd build

# Configure (set PICO_SDK_PATH if not in environment)
cmake -DPICO_SDK_PATH=/path/to/pico-sdk ..

# Build
make -j4
```

### Output Files

After building, the following files will be in the `build/` directory:
- `eweek_2026.uf2` - Drag-and-drop to Pico in BOOTSEL mode
- `eweek_2026.elf` - For debugging with probe
- `eweek_2026.hex` - Intel HEX format

## Configuration

Edit `inc/config.h` to configure:
- Debug output enable/disable (`ENABLE_DEBUG`)
- Verbose iBUS logging (`ENABLE_DEBUG_IBUS_VERBOSE`)
- Watchdog timer (`ENABLE_WATCHDOG`)
- Motor safety cutoff on RC signal loss (`ENABLE_SIGNAL_LOSS_CUTOFF`)
- Encoder calibration (`ENABLE_ENCODER_CALIBRATION`)
- UART and USB stdio output (`ENABLE_STDIO_UART`, `ENABLE_STDIO_USB`)
- OLED display on core 1 (`ENABLE_DISPLAY`)

## Version Information

The build system automatically embeds Git version information into the firmware:

### Automatic Version Tracking

During the build process, CMake generates a `version.h` file containing:
- **Semantic Version**: Project version (e.g., "1.0.0")
- **Git Commit Hash**: Short (7 char) and full (40 char) commit hash
- **Git Branch**: Current branch name
- **Git Tag**: Tag name if HEAD is on a tag
- **Dirty Flag**: Indicates uncommitted changes in working directory
- **Build Timestamp**: Date and time of compilation
- **Build Type**: Debug, Release, etc.

### Usage in Code

Include the generated header to access version information:

```cpp
#include "version.h"

// Display full version string
printf("Firmware: %s\n", BUILD_VERSION_FULL);  // e.g., "1.0.0-abc1234"

// Boot message with build info
printf("%s\n", BUILD_INFO_STRING);  // e.g., "v1.0.0 (abc1234) built Feb 4 2026 09:30:45"

// Check for uncommitted changes
#if GIT_IS_DIRTY
  printf("Warning: Built from uncommitted changes\n");
#endif
```

### Template File

The version information is defined in `inc/version.h.in` and automatically populated during the CMake configuration step. The generated file is placed in `build/generated/version.h` and included via the build system.

## Hardware Setup

### Pin Assignments

See `inc/pinout.h` for complete pin mappings. Key connections:

| Function | GPIO |
|----------|------|
| iBUS TX | 4 |
| iBUS RX | 5 |
| UART0 TX (debug) | 0 |
| UART0 RX (debug) | 1 |

### Differential Drive

| Motor | PWM (Enable) | DIR_FWD | DIR_REV | Encoder |
|-------|-------------|---------|---------|---------|
| Left | 2 | 6 | 3 | N/A |
| Right | 9 | 8 | 7 | N/A |

### Scoop Mechanism

| Function | GPIO |
|----------|------|
| Scoop Servo PWM | 28 |

### Launcher (DRV8825 Stepper + Dual Flywheel)

| Function | GPIO |
|----------|------|
| Stepper STEP (PIO) | 22 |
| Stepper DIR | 21 |
| Stepper nSLEEP | 20 |
| Left Flywheel PWM | 15 |
| Left Flywheel DIR_FWD | 13 |
| Left Flywheel DIR_REV | 14 |
| Right Flywheel PWM | 10 |
| Right Flywheel DIR_FWD | 12 |
| Right Flywheel DIR_REV | 11 |

### I2C Display (SSD1306 OLED)

| Function | GPIO |
|----------|------|
| I2C0 SDA | 16 |
| I2C0 SCL | 17 |

### Status

| Function | GPIO |
|----------|------|
| Onboard LED | 25 |

## Testing

Unit tests run on the host machine (not on Pico):

```bash
cd test
make test
```

See `test/README.md` for more details.

## Safety Features

- **Watchdog Timer:** Automatically resets system if main loop stalls
- **Motor Timeout:** Motors stop if no valid RC signal received within 500 ms
- **Error Handling:** Categorized error codes (general, drive train, RC, mechanism, hardware) with debug reporting

## License

This project is developed for the 2026 Lockheed Martin E-Week competition.

## Authors

E-Week 2026 Team

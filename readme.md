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

## Project Structure

```
eweek_2026/
├── CMakeLists.txt           # Build configuration
├── README.md                # This file
├── pico_sdk_import.cmake    # Pico SDK import script
├── inc/                     # Header files
│   ├── config.h             # Project configuration and feature flags
│   ├── error_handler.h      # Error handling utilities
│   ├── pinout.h             # GPIO pin definitions
│   ├── drive_train.h        # Base drive train class
│   ├── drive_train_differential.h
│   ├── drive_train_mecanum.h
│   ├── flysky_ibus.h        # RC receiver interface
│   ├── ring_buffer.h        # Circular buffer utility
│   ├── mech_collect.h       # Collection mechanism
│   ├── mech_deposit.h       # Deposit mechanism
│   └── mech_launcher.h      # Launcher mechanism
├── src/                     # Source files
│   ├── main.cpp             # Application entry point
│   ├── error_handler.cpp
│   ├── drive_train.cpp
│   ├── drive_train_differential.cpp
│   ├── drive_train_mecanum.cpp
│   ├── flysky_ibus.cpp
│   ├── ring_buffer.cpp
│   ├── mech_collect.cpp
│   ├── mech_deposit.cpp
│   └── mech_launcher.cpp
└── test/                    # Unit tests (host-side)
    ├── Makefile
    ├── README.md
    ├── unit_test.h
    ├── test_ring_buffer.cpp
    ├── test_ibus.cpp
    ├── test_differential.cpp
    └── test_mecanum.cpp
```

## Building

### Prerequisites

1. Raspberry Pi Pico SDK (v2.0.0 or later)
2. CMake (v3.13 or later)
3. ARM GCC toolchain
4. (Optional) VS Code with Pico extension

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
- Drive train type (differential or mecanum)
- Debug output enable/disable
- Watchdog timeout
- Other feature flags

## Hardware Setup

### Pin Assignments

See `inc/pinout.h` for complete pin mappings. Key connections:

| Function | GPIO |
|----------|------|
| iBUS RX  | 5    |
| iBUS TX  | 4    |
| UART0 TX | 0 (debug) |
| UART0 RX | 1 (debug) |

### Drive Train Pins

#### Differential Drive
| Motor | PWM | DIR_FWD | DIR_REV |
|-------|-----|---------|---------|
| Left  | 9   | 7       | 6       |
| Right | 8   | 10      | 11      |

#### Mecanum Drive
| Motor | PWM | DIR_FWD | DIR_REV |
|-------|-----|---------|---------|
| FL    | 20  | 18      | 19      |
| FR    | 8   | 10      | 11      |
| RR    | 9   | 7       | 6       |
| RL    | 21  | 27      | 26      |

## Testing

Unit tests run on the host machine (not on Pico):

```bash
cd test
make test
```

See `test/README.md` for more details.

## Safety Features

- **Watchdog Timer:** Automatically resets system if main loop stalls
- **Motor Timeout:** Motors stop if no valid RC signal received
- **Error Handling:** Comprehensive error reporting and recovery

## Bill of Materials

| Part | Description | Quantity | Unit Cost | Cost |
|------|-------------|----------|-----------|------|
| TBD  | TBD         | TBD      | TBD       | TBD  |

## Lessons Learned

*To be filled in after competition*

## License

This project is developed for the 2026 Lockheed Martin E-Week competition.

## Authors

E-Week 2026 Team

---
*Last Updated: January 2026*

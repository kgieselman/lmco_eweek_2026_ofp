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

## Building

### Prerequisites

1. Raspberry Pi Pico SDK (v2.0.0 or later)
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
- Drive train type (differential or mecanum)
- Debug output enable/disable
- Watchdog timeout
- Other feature flags

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

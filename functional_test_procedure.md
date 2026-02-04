# Functional Test Procedure

## E-Week 2026 Robot Controller

**Document Version:** 1.0  
**Last Updated:** February 2026  
**Target Platform:** Raspberry Pi Pico (RP2040)

---

## Table of Contents

1. [Overview](#1-overview)
2. [Prerequisites](#2-prerequisites)
3. [Safety Precautions](#3-safety-precautions)
4. [Test Equipment](#4-test-equipment)
5. [Pre-Test Setup](#5-pre-test-setup)
6. [Test Procedures](#6-test-procedures)
   - [6.1 Power System Tests](#61-power-system-tests)
   - [6.2 Debug Interface Tests](#62-debug-interface-tests)
   - [6.3 RC Receiver Tests](#63-rc-receiver-tests)
   - [6.4 Motor Driver Tests](#64-motor-driver-tests)
   - [6.5 Drive Train Tests](#65-drive-train-tests)
   - [6.6 Safety System Tests](#66-safety-system-tests)
   - [6.7 Integration Tests](#67-integration-tests)
7. [Troubleshooting](#7-troubleshooting)
8. [Test Log Template](#8-test-log-template)

---

## 1. Overview

This document provides step-by-step procedures for validating the electrical wiring and functional behavior of the E-Week 2026 competition robot. Tests progress from basic power verification through full system integration.

### Test Philosophy

- **Incremental testing:** Verify each subsystem before connecting to others
- **Power-safe:** Always verify power rails before connecting sensitive components
- **Reversible:** Each test should leave the system in a known state
- **Documented:** Record all results for troubleshooting and future reference

---

## 2. Prerequisites

### Firmware

- [ ] Latest firmware compiled successfully
- [ ] `eweek_2026.uf2` file available
- [ ] Debug build with `ENABLE_DEBUG (1)` in `config.h`

### Configuration Verified

```c
// config.h settings for testing
#define DRIVE_TRAIN_MECANUM       (1)  // or DIFFERENTIAL
#define MOTOR_DRIVER_DRV8833      (1)  // or L298N
#define ENABLE_DEBUG              (1)
#define ENABLE_WATCHDOG           (0)  // Disable during testing
#define ENABLE_SIGNAL_LOSS_CUTOFF (1)
```

### Personnel

- Minimum 2 people recommended (1 operator, 1 observer)
- At least one person familiar with the codebase

---

## 3. Safety Precautions

### Electrical Safety

| ⚠️ WARNING |
|:-----------|
| Always disconnect battery/power before making wiring changes. |

- Verify polarity before connecting any power source
- Use current-limited power supply during initial testing (500mA limit)
- Keep conductive tools away from powered circuits
- Have a fire extinguisher rated for electrical fires nearby

### Mechanical Safety

| ⚠️ WARNING |
|:-----------|
| Wheels may spin unexpectedly. Keep hands clear of rotating parts. |

- Elevate robot so wheels spin freely during motor tests
- Secure loose wires away from moving parts
- Have an emergency power disconnect within reach
- Wear safety glasses during mechanism tests

### RF Safety

- Ensure RC transmitter is OFF until RC tests begin
- Verify transmitter is bound to correct receiver
- Test in area clear of other 2.4GHz interference

---

## 4. Test Equipment

### Required

| Item | Purpose |
|------|---------|
| Digital Multimeter (DMM) | Voltage/continuity checks |
| USB cable (micro or USB-C) | Pico programming and debug |
| Serial terminal software | Debug output monitoring |
| FlySky FS-i6X transmitter | RC control testing |
| FlySky FS-iA6B receiver | RC signal source |
| Current-limited power supply | Safe initial power-up |
| Battery (fully charged) | Final integration tests |

### Recommended

| Item | Purpose |
|------|---------|
| Oscilloscope | PWM signal verification |
| Logic analyzer | iBUS protocol debugging |
| Bench power supply (adjustable) | Motor characterization |
| Clamp meter | Current draw measurement |
| Zip ties and tape | Wire management |

### Software

| Software | Purpose | Link |
|----------|---------|------|
| PuTTY / minicom / screen | Serial terminal | — |
| Thonny or VS Code | Pico programming | — |
| Saleae Logic | Protocol analysis (optional) | saleae.com |

---

## 5. Pre-Test Setup

### 5.1 Visual Inspection

Perform before applying any power:

- [ ] **Solder joints:** No cold joints, bridges, or loose connections
- [ ] **Wire routing:** No pinched or strained wires
- [ ] **Connector seating:** All connectors fully inserted
- [ ] **Polarity markings:** Red/black wires match +/- terminals
- [ ] **Component orientation:** ICs and polarized components correct
- [ ] **Mechanical clearance:** Wires clear of moving parts

### 5.2 Continuity Checks

Using DMM in continuity mode:

| Test | From | To | Expected |
|------|------|-----|----------|
| Ground continuity | Pico GND | Motor driver GND | Beep |
| Ground continuity | Pico GND | Receiver GND | Beep |
| Power isolation | VBUS | GND | No beep |
| Power isolation | VSYS | GND | No beep |
| Motor isolation | Motor A+ | Motor A- | No beep (open) |

### 5.3 Firmware Installation

1. Hold BOOTSEL button on Pico
2. Connect USB cable while holding BOOTSEL
3. Release BOOTSEL after 1 second
4. Pico appears as USB mass storage device
5. Drag `eweek_2026.uf2` to the Pico drive
6. Pico automatically reboots and runs firmware

---

## 6. Test Procedures

### 6.1 Power System Tests

#### TEST-PWR-001: Input Voltage Verification

**Objective:** Verify power supply voltage is within acceptable range.

| Step | Action | Expected Result | ✓/✗ |
|------|--------|-----------------|-----|
| 1 | Disconnect all loads from power supply | — | |
| 2 | Set bench supply to 7.4V, 500mA limit | Display shows 7.4V | |
| 3 | Measure voltage at supply terminals with DMM | 7.2V – 7.6V | |
| 4 | Connect power to robot main power input | No sparks, no smoke | |
| 5 | Measure voltage at Pico VSYS pin | 4.8V – 5.2V (if regulated) | |
| 6 | Measure voltage at motor driver VIN | 7.2V – 7.6V | |

**Pass Criteria:** All voltages within specified ranges, no visible damage.

---

#### TEST-PWR-002: Current Draw at Idle

**Objective:** Verify quiescent current is reasonable.

| Step | Action | Expected Result | ✓/✗ |
|------|--------|-----------------|-----|
| 1 | Power on system with current-limited supply | System boots | |
| 2 | Observe current reading on supply | < 100mA idle | |
| 3 | Connect serial terminal, verify debug output | Boot messages visible | |
| 4 | Record idle current: ______ mA | — | |

**Pass Criteria:** Idle current < 100mA, debug output visible.

---

### 6.2 Debug Interface Tests

#### TEST-DBG-001: USB Serial Output

**Objective:** Verify USB CDC debug output is functional.

**Setup:** Connect Pico USB to computer, open serial terminal at 115200 baud.

| Step | Action | Expected Result | ✓/✗ |
|------|--------|-----------------|-----|
| 1 | Reset Pico (power cycle or reset button) | — | |
| 2 | Observe serial terminal | Boot banner appears within 3 seconds | |
| 3 | Verify banner content | Shows "E-Week 2026 Robot Controller" | |
| 4 | Verify initialization messages | "[Init]" messages for each subsystem | |
| 5 | Verify no error messages | No "[ERROR]" or "[WARN]" in boot sequence | |

**Expected Boot Output:**
```
========================================
  E-Week 2026 Robot Controller
========================================

[Init] Configuring iBUS receiver...
[iBUS] Initialized on UART1
[Init] Configuring Mecanum drive train...
[DriveTrain] Motor 0 configured (DRV8833): FWD=18, REV=19
[DriveTrain] Motor 1 configured (DRV8833): FWD=10, REV=11
[DriveTrain] Motor 2 configured (DRV8833): FWD=7, REV=6
[DriveTrain] Motor 3 configured (DRV8833): FWD=27, REV=26
[Init] Configuring collection mechanism...
[Init] Configuring deposit mechanism...
[Init] Configuring launcher mechanism...
[Init] Enabling watchdog (timeout: 1000 ms)...

[Init] System initialization complete!

[Main] Entering main loop
```

**Pass Criteria:** All initialization messages present, no errors.

---

#### TEST-DBG-002: UART Serial Output

**Objective:** Verify hardware UART debug output (GPIO 0/1).

**Setup:** Connect USB-serial adapter to GPIO 0 (TX) and GND. Open terminal at 115200 baud.

| Step | Action | Expected Result | ✓/✗ |
|------|--------|-----------------|-----|
| 1 | Disconnect USB from Pico | — | |
| 2 | Power Pico from external supply | — | |
| 3 | Observe serial terminal | Boot banner appears | |
| 4 | Compare output to USB serial | Identical content | |

**Pass Criteria:** UART output matches USB serial output.

---

### 6.3 RC Receiver Tests

#### TEST-RC-001: Receiver Power and Binding

**Objective:** Verify RC receiver is powered and bound to transmitter.

| Step | Action | Expected Result | ✓/✗ |
|------|--------|-----------------|-----|
| 1 | Power on robot (transmitter OFF) | Receiver LED solid or slow blink | |
| 2 | Power on transmitter | Receiver LED changes to solid | |
| 3 | Verify binding | Receiver LED solid (not flashing) | |
| 4 | Move transmitter sticks | No motor movement yet (expected) | |

**Pass Criteria:** Receiver binds successfully, LED solid.

---

#### TEST-RC-002: iBUS Signal Reception

**Objective:** Verify iBUS protocol communication.

| Step | Action | Expected Result | ✓/✗ |
|------|--------|-----------------|-----|
| 1 | Enable `ENABLE_DEBUG_IBUS_VERBOSE (1)` and reflash | — | |
| 2 | Power on system with transmitter bound | — | |
| 3 | Observe serial terminal | Channel values printed | |
| 4 | Center all sticks on transmitter | Values near 1500 (±20) | |
| 5 | Move right stick full forward | CHAN_RSTICK_VERT ≈ 2000 | |
| 6 | Move right stick full back | CHAN_RSTICK_VERT ≈ 1000 | |
| 7 | Move right stick full right | CHAN_RSTICK_HORIZ ≈ 2000 | |
| 8 | Move right stick full left | CHAN_RSTICK_HORIZ ≈ 1000 | |
| 9 | Repeat for left stick | Verify all 4 axes respond | |

**Channel Value Recording:**

| Control | Stick Position | Expected | Actual | ✓/✗ |
|---------|---------------|----------|--------|-----|
| Right Vertical | Center | 1500 | | |
| Right Vertical | Full Forward | 2000 | | |
| Right Vertical | Full Back | 1000 | | |
| Right Horizontal | Center | 1500 | | |
| Right Horizontal | Full Right | 2000 | | |
| Right Horizontal | Full Left | 1000 | | |
| Left Vertical | Center | 1500 | | |
| Left Horizontal | Center | 1500 | | |

**Pass Criteria:** All channels respond correctly within ±50 of expected values.

---

#### TEST-RC-003: Signal Loss Detection

**Objective:** Verify signal loss is detected and handled.

| Step | Action | Expected Result | ✓/✗ |
|------|--------|-----------------|-----|
| 1 | With system running and RC connected | No warnings | |
| 2 | Power OFF transmitter | — | |
| 3 | Observe serial terminal within 500ms | "[WARN] RC signal lost!" appears | |
| 4 | Verify motors stop (if running) | All motor outputs = 0 | |
| 5 | Power ON transmitter | — | |
| 6 | Verify signal restored | Warning messages stop | |

**Pass Criteria:** Signal loss detected within timeout, motors stop.

---

### 6.4 Motor Driver Tests

#### TEST-MOT-001: Motor Driver Power

**Objective:** Verify motor driver IC is receiving power.

| Step | Action | Expected Result | ✓/✗ |
|------|--------|-----------------|-----|
| 1 | Power on system | Motor driver power LED on (if equipped) | |
| 2 | Measure VIN at motor driver | Matches supply voltage | |
| 3 | Measure VM (motor voltage) | Matches supply voltage | |
| 4 | Measure logic voltage (if separate) | 3.3V ± 0.2V | |

**Pass Criteria:** All voltages within specification.

---

#### TEST-MOT-002: Individual Motor Wiring (DRV8833)

**Objective:** Verify each motor is wired correctly.

**Setup:** Elevate robot so wheels spin freely. Disconnect transmitter.

**Test each motor individually by temporarily modifying firmware:**

```cpp
// Add to main.cpp for testing (remove after)
// Test Motor 0 (Front Left)
m_motorDriver.setMotor(MotorDriver::MOTOR_A, 250);  // 50% forward
sleep_ms(2000);
m_motorDriver.setMotor(MotorDriver::MOTOR_A, 0);
```

| Motor | Position | Pin IN1 | Pin IN2 | Spins? | Direction Correct? | ✓/✗ |
|-------|----------|---------|---------|--------|-------------------|-----|
| 0 | Front Left | 18 | 19 | | | |
| 1 | Front Right | 10 | 11 | | | |
| 2 | Rear Right | 7 | 6 | | | |
| 3 | Rear Left | 27 | 26 | | | |

**If direction is wrong:** Swap IN1 and IN2 pin assignments in `pinout.h`.

**Pass Criteria:** All motors spin, directions match expectations.

---

#### TEST-MOT-003: Motor Current Draw

**Objective:** Verify motor current is within safe limits.

| Step | Action | Expected Result | ✓/✗ |
|------|--------|-----------------|-----|
| 1 | Set current limit to 2A on power supply | — | |
| 2 | Command single motor to 100% | Motor spins | |
| 3 | Record current draw | < 1.5A per motor (DRV8833 limit) | |
| 4 | Briefly stall motor (< 1 second) | Current increases but supply limits | |
| 5 | Record stall current | Note for battery sizing | |

**Current Measurements:**

| Motor | Free Spin Current | Stall Current | ✓/✗ |
|-------|------------------|---------------|-----|
| Front Left | | | |
| Front Right | | | |
| Rear Right | | | |
| Rear Left | | | |

**Pass Criteria:** No motor exceeds driver current rating.

---

### 6.5 Drive Train Tests

#### TEST-DT-001: Forward/Reverse Motion

**Objective:** Verify forward and reverse commands move robot correctly.

**Setup:** Robot elevated, wheels free to spin.

| Step | Action | Expected Result | ✓/✗ |
|------|--------|-----------------|-----|
| 1 | Push right stick full forward | All wheels spin forward | |
| 2 | Verify wheel directions | All spinning same direction | |
| 3 | Pull right stick full back | All wheels spin reverse | |
| 4 | Release stick to center | All wheels stop | |

**Pass Criteria:** Uniform forward/reverse motion.

---

#### TEST-DT-002: Turn Left/Right

**Objective:** Verify turning commands work correctly.

| Step | Action | Expected Result | ✓/✗ |
|------|--------|-----------------|-----|
| 1 | Push right stick full right | Left wheels forward, right wheels reverse | |
| 2 | Robot should turn | Clockwise rotation | |
| 3 | Push right stick full left | Right wheels forward, left wheels reverse | |
| 4 | Robot should turn | Counter-clockwise rotation | |

**Pass Criteria:** Robot turns in correct direction for stick input.

---

#### TEST-DT-003: Strafe Left/Right (Mecanum Only)

**Objective:** Verify strafing works correctly.

| Step | Action | Expected Result | ✓/✗ |
|------|--------|-----------------|-----|
| 1 | Push left stick full right | Robot strafes right | |
| 2 | Verify wheel pattern | FL+, FR-, RR+, RL- | |
| 3 | Push left stick full left | Robot strafes left | |
| 4 | Verify wheel pattern | FL-, FR+, RR-, RL+ | |

**Mecanum Strafe Pattern Reference:**

```
Strafe Right:        Strafe Left:
  FL(+)  FR(-)        FL(-)  FR(+)
  RL(-)  RR(+)        RL(+)  RR(-)
```

**Pass Criteria:** Robot moves sideways without rotating.

---

#### TEST-DT-004: Combined Motion

**Objective:** Verify combined speed/turn/strafe inputs.

| Step | Action | Expected Result | ✓/✗ |
|------|--------|-----------------|-----|
| 1 | Forward + slight right turn | Curves right while moving forward | |
| 2 | Forward + strafe right | Diagonal forward-right movement | |
| 3 | Full forward + full right turn | Tightest right curve | |
| 4 | All three inputs at once | Complex motion, no clipping | |

**Pass Criteria:** Combined inputs produce expected motion, no motor stalls.

---

#### TEST-DT-005: Ground Test

**Objective:** Verify robot moves correctly on the ground.

| ⚠️ CAUTION |
|:-----------|
| Clear area of obstacles. Have emergency stop ready. |

| Step | Action | Expected Result | ✓/✗ |
|------|--------|-----------------|-----|
| 1 | Place robot on flat surface | Stable, all wheels touching | |
| 2 | Apply gentle forward input | Robot moves forward in straight line | |
| 3 | Apply gentle turn input | Robot rotates in place | |
| 4 | Apply strafe input (mecanum) | Robot moves sideways | |
| 5 | Test at various speeds | Consistent behavior | |

**Observations:**
- Does robot pull to one side? (Trim adjustment needed)
- Any unusual sounds? (Mechanical issue)
- Smooth acceleration? (PWM frequency OK)

**Pass Criteria:** Robot moves as expected on ground.

---

### 6.6 Safety System Tests

#### TEST-SAF-001: Signal Loss Motor Cutoff

**Objective:** Verify motors stop on RC signal loss.

| Step | Action | Expected Result | ✓/✗ |
|------|--------|-----------------|-----|
| 1 | Drive robot forward at 50% speed | Robot moving | |
| 2 | Turn OFF transmitter | — | |
| 3 | Time until robot stops | < 600ms (timeout + processing) | |
| 4 | Verify serial output | "[WARN] RC signal lost!" | |
| 5 | Turn ON transmitter | Robot remains stopped until input | |

**Pass Criteria:** Robot stops within specified timeout.

---

#### TEST-SAF-002: Watchdog Recovery

**Objective:** Verify watchdog resets a stalled system.

**Setup:** Enable watchdog in config: `ENABLE_WATCHDOG (1)`

| Step | Action | Expected Result | ✓/✗ |
|------|--------|-----------------|-----|
| 1 | Boot system normally | Normal operation | |
| 2 | Add intentional infinite loop to code | — | |
| 3 | Observe system | Watchdog resets within 1 second | |
| 4 | Check serial output after reset | "[WARN] System rebooted by watchdog!" | |
| 5 | Remove test code, reflash | Normal operation restored | |

**Pass Criteria:** Watchdog triggers and system recovers.

---

#### TEST-SAF-003: Error LED Indication

**Objective:** Verify LED indicates error conditions.

| Step | Action | Expected Result | ✓/✗ |
|------|--------|-----------------|-----|
| 1 | Boot with a misconfiguration | Init fails | |
| 2 | Observe onboard LED | Blinking at ~2Hz (250ms on/off) | |
| 3 | Verify serial output | "[ERROR] System initialization failed" | |

**Pass Criteria:** LED blinks to indicate error state.

---

### 6.7 Integration Tests

#### TEST-INT-001: Full System Power On

**Objective:** Verify complete system boots correctly.

| Step | Action | Expected Result | ✓/✗ |
|------|--------|-----------------|-----|
| 1 | Connect fully charged battery | — | |
| 2 | Power on transmitter | — | |
| 3 | Power on robot | Boot sequence completes | |
| 4 | Time from power-on to ready | < 5 seconds | |
| 5 | Verify no error messages | Clean boot | |
| 6 | Test basic motion | Robot responds correctly | |

**Pass Criteria:** System boots and operates from battery power.

---

#### TEST-INT-002: Extended Operation

**Objective:** Verify system stability over time.

| Step | Action | Expected Result | ✓/✗ |
|------|--------|-----------------|-----|
| 1 | Run system for 10 minutes | Continuous operation | |
| 2 | Periodically send commands | Consistent response | |
| 3 | Monitor for errors | No unexpected warnings/errors | |
| 4 | Check component temperatures | No excessive heat | |
| 5 | Monitor battery voltage | Gradual decline only | |

**Thermal Check Points:**
- [ ] Motor driver IC: Warm but touchable (< 60°C)
- [ ] Motors: Warm but touchable
- [ ] Pico: Slightly warm
- [ ] Battery: No heat increase

**Pass Criteria:** 10 minutes of stable operation.

---

#### TEST-INT-003: Competition Simulation

**Objective:** Simulate competition conditions.

| Step | Action | Expected Result | ✓/✗ |
|------|--------|-----------------|-----|
| 1 | Set up mock competition course | — | |
| 2 | Practice driving through course | Responsive control | |
| 3 | Test collection mechanism (when ready) | Successful operation | |
| 4 | Test deposit mechanism (when ready) | Successful operation | |
| 5 | Test launcher (when ready) | Successful operation | |
| 6 | Complete 3 full "laps" | Consistent performance | |

**Pass Criteria:** Robot completes simulated competition tasks.

---

## 7. Troubleshooting

### No Serial Output

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| No output on USB | USB not enumerated | Wait 2+ seconds, check cable |
| No output on UART | Wrong baud rate | Set terminal to 115200 |
| Garbled output | Baud rate mismatch | Verify 115200-8-N-1 |
| Output stops | Watchdog reset loop | Disable watchdog, check code |

### Motors Not Spinning

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| No motors work | Power not connected | Check motor driver VIN |
| One motor dead | Wiring issue | Check continuity, pin assignment |
| Motors jitter | PWM too low | Increase PWM frequency |
| Motors hot | Stalled or shorted | Check mechanical binding |

### RC Not Working

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| Receiver not binding | Wrong protocol | Verify iBUS, rebind |
| No channel data | TX/RX pins swapped | Check wiring to GPIO 4/5 |
| Intermittent signal | Interference | Move away from WiFi routers |
| Wrong channel mapping | Config mismatch | Verify channel assignments |

### Robot Moves Wrong Direction

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| Forward = backward | Motor wires swapped | Swap IN1/IN2 in pinout.h |
| Turns wrong way | Left/right motors swapped | Swap motor assignments |
| Strafe wrong | Mecanum wheel orientation | Check wheel roller angles |

---

## 8. Test Log Template

### Test Session Information

| Field | Value |
|-------|-------|
| Date | |
| Tester(s) | |
| Firmware Version | |
| Hardware Revision | |
| Battery Voltage (Start) | |
| Battery Voltage (End) | |

### Test Results Summary

| Test ID | Test Name | Result | Notes |
|---------|-----------|--------|-------|
| TEST-PWR-001 | Input Voltage | ☐ Pass ☐ Fail | |
| TEST-PWR-002 | Current at Idle | ☐ Pass ☐ Fail | |
| TEST-DBG-001 | USB Serial | ☐ Pass ☐ Fail | |
| TEST-DBG-002 | UART Serial | ☐ Pass ☐ Fail | |
| TEST-RC-001 | Receiver Binding | ☐ Pass ☐ Fail | |
| TEST-RC-002 | iBUS Reception | ☐ Pass ☐ Fail | |
| TEST-RC-003 | Signal Loss | ☐ Pass ☐ Fail | |
| TEST-MOT-001 | Motor Driver Power | ☐ Pass ☐ Fail | |
| TEST-MOT-002 | Motor Wiring | ☐ Pass ☐ Fail | |
| TEST-MOT-003 | Motor Current | ☐ Pass ☐ Fail | |
| TEST-DT-001 | Forward/Reverse | ☐ Pass ☐ Fail | |
| TEST-DT-002 | Turn Left/Right | ☐ Pass ☐ Fail | |
| TEST-DT-003 | Strafe (Mecanum) | ☐ Pass ☐ Fail | |
| TEST-DT-004 | Combined Motion | ☐ Pass ☐ Fail | |
| TEST-DT-005 | Ground Test | ☐ Pass ☐ Fail | |
| TEST-SAF-001 | Signal Loss Cutoff | ☐ Pass ☐ Fail | |
| TEST-SAF-002 | Watchdog Recovery | ☐ Pass ☐ Fail | |
| TEST-SAF-003 | Error LED | ☐ Pass ☐ Fail | |
| TEST-INT-001 | Full System Boot | ☐ Pass ☐ Fail | |
| TEST-INT-002 | Extended Operation | ☐ Pass ☐ Fail | |
| TEST-INT-003 | Competition Sim | ☐ Pass ☐ Fail | |

### Issues Found

| Issue # | Test ID | Description | Severity | Resolution |
|---------|---------|-------------|----------|------------|
| 1 | | | ☐ Critical ☐ Major ☐ Minor | |
| 2 | | | ☐ Critical ☐ Major ☐ Minor | |
| 3 | | | ☐ Critical ☐ Major ☐ Minor | |

### Sign-Off

| Role | Name | Signature | Date |
|------|------|-----------|------|
| Tester | | | |
| Reviewer | | | |

---

*E-Week 2026 Team*

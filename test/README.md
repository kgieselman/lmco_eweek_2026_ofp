# Unit Test Suite

Comprehensive unit tests for the LMCO E-Week 2026 robotics competition firmware.

## Overview

These tests validate the core algorithms and data structures without requiring hardware. This allows you to catch bugs early and verify implementations before deploying to the Raspberry Pi Pico.

## Test Coverage

✅ **76 Total Tests** - All Passing

### Differential Drive Tests ([test_differential.cpp](test_differential.cpp))
**14 tests** validating:
- ✅ **Basic Movement**: Forward, backward, stopped states
- ✅ **Turning**: In-place rotation, gentle turns, sharp turns
- ✅ **PWM Scaling**: Anti-clipping algorithm, ratio preservation
- ✅ **Trim Application**: Motor calibration effects

### Mecanum Drive Tests ([test_mecanum.cpp](test_mecanum.cpp))
**17 tests** validating:
- ✅ **Basic Movement**: Forward, backward, stopped states
- ✅ **Strafing**: Left, right, diagonal movements
- ✅ **Rotation**: Clockwise, counterclockwise
- ✅ **Combined Movement**: Multi-axis control (forward + rotate, etc.)
- ✅ **PWM Scaling**: Vector preservation, overflow prevention
- ✅ **4-Motor Kinematics**: Individual motor calculations

### Ring Buffer Tests ([test_ring_buffer.cpp](test_ring_buffer.cpp))
**21 tests** validating:
- ✅ **FIFO Behavior**: First-in-first-out circular buffer operations
- ✅ **Push/Pop**: Adding and removing elements
- ✅ **Peek**: Reading without removal
- ✅ **Wrap-Around**: Boundary handling at buffer edges
- ✅ **Overflow Protection**: Handling full buffer conditions

### iBUS Protocol Tests ([test_ibus.cpp](test_ibus.cpp))
**24 tests** validating:
- ✅ **CRC Calculation**: Checksum validation
- ✅ **Message Parsing**: Protocol structure validation
- ✅ **14-Channel Extraction**: RC receiver data decoding
- ✅ **Sensor Configuration**: 2-byte and 4-byte sensor data
- ✅ **RC Input Handling**: Stick positions and switch values

## Quick Start

### Building Tests

```bash
cd test/
make          # Build all test executables
```

### Running Tests

Run all tests:
```bash
make test
```

Run individual test suites:
```bash
make test-diff      # Differential drive (14 tests)
make test-mecanum   # Mecanum drive (17 tests)
make test-ring      # Ring buffer (21 tests)
make test-ibus      # iBUS protocol (24 tests)
```

Clean build artifacts:
```bash
make clean
```

### Expected Output

Successful test run example:

```
Running Tests: Differential Drive
  Test: forward_motion ... PASSED
  Test: backward_motion ... PASSED
  Test: turn_in_place ... PASSED
  ...

═══════════════════════════════════════════════════
Test Summary
═══════════════════════════════════════════════════
Total Tests: 14
Passed: 14
Failed: 0
Pass Rate: 100.0%
═══════════════════════════════════════════════════
```

When running all tests (`make test`), each suite runs sequentially with a summary at the end.

## Understanding Test Failures

If a test fails, you'll see detailed output:

```
  Test: forward_full_speed ... FAILED
    ✗ Expected: 1000, Actual: 0
    Location: test_drive_train_differential.cpp:120
```

This tells you:
- Which test failed
- What was expected vs. what actually happened
- Exact line number in the test file

## Test Framework

### Custom Lightweight Framework

We use custom unit test frameworks designed for embedded systems:

- **[unit_test.h](unit_test.h)**: Lightweight framework for current tests

Framework provides:
- **No external dependencies** (no Google Test, Catch2, etc.)
- **Colorized output** for easy reading
- **Simple macros** similar to popular frameworks
- **Portable** - works on Linux, macOS, Windows

### Available Assertions

```cpp
ASSERT_TRUE(condition)                           // Condition must be true
ASSERT_FALSE(condition)                          // Condition must be false
ASSERT_EQUAL(expected, actual)                   // Values must be equal
ASSERT_NOT_EQUAL(not_expected, actual)           // Values must differ
ASSERT_FLOAT_EQUAL(expected, actual, tolerance)  // Floats within tolerance
ASSERT_IN_RANGE(value, min, max)                 // Value in range [min, max]
ASSERT_NULL(ptr)                                 // Pointer must be NULL
ASSERT_NOT_NULL(ptr)                             // Pointer must not be NULL
```

### Writing New Tests

1. **Define a test suite:**
```cpp
TEST_SUITE(MyFeature_Description)
    // Tests go here
END_TEST_SUITE()
```

2. **Define individual tests:**
```cpp
TEST(my_test_name)
{
    // Arrange
    int speed = 500;
    int turn = 0;
    
    // Act
    MotorValues result = calculate_differential_drive(speed, turn);
    
    // Assert
    ASSERT_EQUAL(1000, result.leftPWM);
    ASSERT_TRUE(result.leftFwd);
    
    // If all assertions pass:
    TEST_PASS();
}
```

3. **Register and run the test:**
```cpp
int main(void)
{
    test_suite_MyFeature_Description();
    RUN_TEST(my_test_name);
    
    PRINT_TEST_SUMMARY();
    return (g_tests_failed > 0) ? 1 : 0;
}
```

## Integration with CI/CD

### Exit Codes

The test executables return:
- **Exit code 0**: All tests passed
- **Exit code 1**: One or more tests failed

This makes them perfect for CI/CD integration!

### Example CI Script

```bash
#!/bin/bash
cd test/
make test
if [ $? -ne 0 ]; then
    echo "Tests failed!"
    exit 1
fi
```

## What Gets Tested

### Differential Drive Algorithm

Tests validate tank-style steering calculations:

```cpp
calcVal[MOTOR_LEFT ] = m_speed +  m_turn;
calcVal[MOTOR_RIGHT] = m_speed - m_turn;

// Plus trim, scaling, PWM, and direction handling...
```

### Mecanum Drive Kinematics

Tests validate omnidirectional 4-motor kinematics:

```cpp
mecanumVal[MOTOR_FRONT_LEFT ] = m_speed + m_strafe + m_turn;
mecanumVal[MOTOR_FRONT_RIGHT] = m_speed - m_strafe - m_turn;
mecanumVal[MOTOR_REAR_RIGHT ] = m_speed + m_strafe - m_turn;
mecanumVal[MOTOR_REAR_LEFT  ] = m_speed - m_strafe + m_turn;

// Plus PWM scaling and overflow protection...
```

### Ring Buffer

Tests validate circular FIFO buffer operations including push, pop, peek, wrap-around, and overflow handling.

### iBUS Protocol

Tests validate RC receiver protocol including CRC validation, message parsing, 14-channel data extraction, and sensor configuration.

## Known Limitations

1. **Hardware Not Tested**: These tests validate calculations only. Hardware interactions (GPIO, PWM, interrupts) require integration testing.

2. **Float Precision**: Some tests use tolerance-based comparisons due to floating-point arithmetic.

3. **No Mock Hardware**: We don't simulate the Pico SDK hardware layer.

## Debugging Failed Tests

If tests fail after code changes:

1. **Check the test output** - shows exactly what went wrong
2. **Verify constants match** - ensure test constants match source code
3. **Add debug output** - use `printf()` in the calculation functions
4. **Run tests individually** - isolate the failing test
5. **Compare with working version** - use git to see what changed

## Requirements

- C++11 compatible compiler (g++, clang++)
- Make
- Standard math library

No Pico SDK required for running these tests!

## Typical Workflow

1. Make code changes to source files
2. Run relevant tests: `make test-<module>`
3. If tests fail, fix code and re-run
4. Run all tests: `make test`
5. Commit when all tests pass

## Continuous Development

As you modify the code:

1. **Update tests first** (TDD approach) or alongside code changes
2. **Run tests before committing** to catch regressions
3. **Add tests for bug fixes** to prevent recurrence
4. **Keep tests fast** - all tests should run in under 1 second

## File Structure

```
test/
├── unit_test.h                 # Lightweight test framework
├── test_differential.cpp       # Differential drive tests (14 tests)
├── test_mecanum.cpp            # Mecanum drive tests (17 tests)
├── test_ring_buffer.cpp        # Ring buffer tests (21 tests)
├── test_ibus.cpp               # iBUS protocol tests (24 tests)
├── Makefile                    # Build system
├── build/                      # Test executables (auto-generated)
└── README.md                   # This file
```

## Future Enhancements

Potential additions:

- [ ] Encoder calibration algorithm tests
- [ ] Integration tests with mock hardware
- [ ] Performance benchmarks
- [ ] Code coverage reporting
- [ ] Additional sensor protocol tests

## Contributing

When adding new features or fixing bugs:

1. Write tests first (or alongside implementation)
2. Ensure all existing tests still pass
3. Document any new test utilities
4. Keep test names descriptive
5. Follow the existing test structure and style

## Questions?

For issues or questions about the tests:
- Check the code review document for related bugs
- Review the test output carefully
- Add debug prints to understand failures
- Consult the source code comments

---

**Happy Testing!**

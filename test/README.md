# Drive Train Unit Tests

Comprehensive unit tests for the differential and mecanum drive train calculations used in the LMCO E-Week 2026 robotics competition.

## Overview

These tests validate the mathematical calculations that control motor speeds and directions for both drive train types without requiring hardware. This allows you to catch bugs early and verify the algorithms before deploying to the Raspberry Pi Pico.

## Test Coverage

### Differential Drive Tests (`test_drive_train_differential.cpp`)
- ✅ **Basic Movement**: Forward, backward, stopped states
- ✅ **Turning**: In-place rotation, gentle turns, sharp turns
- ✅ **PWM Scaling**: Anti-clipping algorithm, ratio preservation
- ✅ **Trim Application**: Motor calibration effects
- ✅ **Edge Cases**: Maximum inputs, asymmetric configurations
- ✅ **Input Validation**: Boundary conditions

**Total: 29 test cases**

### Mecanum Drive Tests (`test_drive_train_mecanum.cpp`)
- ✅ **Basic Movement**: Forward, backward, stopped states  
- ✅ **Strafing**: Left, right, diagonal movements
- ✅ **Rotation**: Clockwise, counterclockwise
- ✅ **Combined Movement**: Multi-axis control
- ✅ **PWM Scaling**: Vector preservation, overflow prevention
- ✅ **Symmetry**: Verify kinematic correctness
- ✅ **Kinematics**: Individual motor calculations

**Total: 39 test cases**

## Quick Start

### Building Tests

```bash
cd test/
make all
```

### Running Tests

Run all tests:
```bash
make test
```

Run differential tests only:
```bash
make test-diff
```

Run mecanum tests only:
```bash
make test-mecanum
```

### Expected Output

```
╔═══════════════════════════════════════════════════════════╗
║     Differential Drive Train Unit Tests                  ║
╚═══════════════════════════════════════════════════════════╝

═══════════════════════════════════════════════════
Running Test Suite: DifferentialDrive_BasicMovement
═══════════════════════════════════════════════════

  Test: forward_full_speed ... PASSED
  Test: backward_full_speed ... PASSED
  ...

═══════════════════════════════════════════════════
Test Summary
═══════════════════════════════════════════════════
Total Tests: 29
Passed: 29
Failed: 0
Pass Rate: 100.0%
═══════════════════════════════════════════════════
```

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

We use a custom unit test framework (`unit_test.h`) designed for embedded systems:

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

The test executables return:
- **Exit code 0**: All tests passed
- **Exit code 1**: One or more tests failed

This makes them easy to integrate into continuous integration:

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

The tests validate the exact algorithm from `drive_train_differential.cpp`:

```cpp
calcVal[MOTOR_LEFT ] = m_speed +  m_turn;
calcVal[MOTOR_RIGHT] = m_speed - m_turn;

// Apply trim...
// Calculate scaling...
// Apply PWM and direction...
```

### Mecanum Drive Kinematics

The tests validate the mecanum kinematics from `drive_train_mecanum.cpp`:

```cpp
mecanumVal[MOTOR_FRONT_LEFT ] = m_speed + m_strafe + m_turn;
mecanumVal[MOTOR_FRONT_RIGHT] = m_speed - m_strafe - m_turn;
mecanumVal[MOTOR_REAR_RIGHT ] = m_speed + m_strafe - m_turn;
mecanumVal[MOTOR_REAR_LEFT  ] = m_speed - m_strafe + m_turn;
```

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

## Continuous Development

As you modify the drive train code:

1. **Update tests first** (TDD approach) or alongside code changes
2. **Run tests before committing** to catch regressions
3. **Add tests for bug fixes** to prevent recurrence
4. **Keep tests fast** - all tests should run in under 1 second

## File Structure

```
test/
├── unit_test.h                          # Test framework
├── test_drive_train_differential.cpp    # Differential drive tests
├── test_drive_train_mecanum.cpp         # Mecanum drive tests
├── Makefile                             # Build system
└── README.md                            # This file
```

## Future Enhancements

Potential additions:

- [ ] Ring buffer tests
- [ ] IBus message parsing tests
- [ ] Encoder calibration algorithm tests
- [ ] Integration tests with mock hardware
- [ ] Performance benchmarks
- [ ] Code coverage reporting

## Contributing

When adding new drive train features:

1. Write tests first (or alongside implementation)
2. Ensure all existing tests still pass
3. Document any new test utilities
4. Keep test names descriptive

## Questions?

For issues or questions about the tests:
- Check the code review document for related bugs
- Review the test output carefully
- Add debug prints to understand failures
- Consult the drive train source code comments

---

**Happy Testing! 🚗💨**

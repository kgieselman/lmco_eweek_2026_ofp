# Unit Tests

This directory contains unit tests for the E-Week 2026 robot controller.

## Overview

The unit tests are designed to run on the **host machine** (your development
computer), not on the Pico. This allows for rapid testing of algorithms and
logic without needing to flash the hardware.

## Test Files

| File | Description |
|------|-------------|
| `unit_test.h` | Lightweight testing framework |
| `test_ring_buffer.cpp` | Tests for circular buffer implementation |
| `test_ibus.cpp` | Tests for iBUS protocol (CRC, message parsing) |
| `test_differential.cpp` | Tests for differential drive motor mixing |
| `test_mecanum.cpp` | Tests for mecanum drive motor mixing |

## Building and Running

### Prerequisites

- GCC/G++ compiler (C++17 support)
- Make

### Build All Tests

```bash
make
```

### Run All Tests

```bash
make test
```

### Run Individual Test Suites

```bash
make test-ring      # Ring buffer tests
make test-ibus      # iBUS protocol tests
make test-diff      # Differential drive tests
make test-mecanum   # Mecanum drive tests
```

### Clean Build Artifacts

```bash
make clean
```

## Writing New Tests

### Test Framework

The `unit_test.h` header provides a simple testing framework:

```cpp
#include "unit_test.h"

// Define a test function
TEST_FUNC(my_test_name)
{
  ASSERT_TRUE(condition);
  ASSERT_EQUAL(expected, actual);
  ASSERT_FLOAT_EQUAL(expected, actual, tolerance);
  ASSERT_IN_RANGE(value, min, max);
}

// In main()
int main(void)
{
  SUITE_START("My Test Suite");
  RUN_TEST(my_test_name);
  
  PRINT_SUMMARY();
  return (g_tests_failed > 0) ? 1 : 0;
}
```

### Available Assertions

| Macro | Description |
|-------|-------------|
| `ASSERT_TRUE(cond)` | Assert condition is true |
| `ASSERT_FALSE(cond)` | Assert condition is false |
| `ASSERT_EQUAL(exp, act)` | Assert integers are equal |
| `ASSERT_NOT_EQUAL(unexp, act)` | Assert integers are not equal |
| `ASSERT_FLOAT_EQUAL(exp, act, tol)` | Assert floats are equal within tolerance |
| `ASSERT_IN_RANGE(val, min, max)` | Assert value is within range |
| `ASSERT_NOT_NULL(ptr)` | Assert pointer is not null |

### Test Organization

Tests are organized by:

1. **Module** - Each source file gets its own test file
2. **Suite** - Related tests grouped with `SUITE_START("Name")`
3. **Test** - Individual test functions

### Adding a New Test File

1. Create `test_mymodule.cpp`
2. Include necessary headers
3. Add mock implementations of Pico SDK functions if needed
4. Write test functions
5. Add main() with test runner
6. Update `Makefile` to include new test

## Mock Implementations

Since tests run on the host, Pico SDK functions are not available. Tests either:

1. Re-implement the logic being tested directly
2. Provide mock implementations of SDK functions

For example, `test_ring_buffer.cpp` includes a copy of the `RingBuffer` class
because the actual implementation depends on Pico SDK types.

## Continuous Integration

Tests return exit code 0 on success, non-zero on failure. This makes them
suitable for CI/CD pipelines:

```bash
make test && echo "All tests passed!"
```

## Test Coverage

Current test coverage includes:

- **Ring Buffer**: Basic operations, FIFO ordering, wraparound, edge cases
- **iBUS**: CRC calculation, message validation, channel extraction
- **Differential Drive**: Motor mixing, PWM scaling, symmetry
- **Mecanum Drive**: Motor mixing, strafe, rotation, combined movements

## Known Limitations

- Hardware-dependent features (interrupts, GPIO, PWM) cannot be fully tested
- Timing-dependent behavior must be tested on actual hardware
- Memory constraints of the Pico are not simulated

---
*E-Week 2026 Team*

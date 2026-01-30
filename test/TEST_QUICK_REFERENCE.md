# Test Framework Quick Reference

## Running Tests

### All Tests
```bash
cd test && make test
```

### Individual Test Suites
```bash
make test-diff      # Differential drive (14 tests)
make test-mecanum   # Mecanum drive (17 tests)
make test-ring      # Ring buffer (21 tests)
make test-ibus      # iBUS protocol (24 tests)
```

### Build Only (No Execution)
```bash
make                # Build all
make clean          # Remove build artifacts
```

## Test Results

✅ **76 Total Tests** - All Passing  
- Differential Drive: 14/14 ✓
- Mecanum Drive: 17/17 ✓
- Ring Buffer: 21/21 ✓
- iBUS Protocol: 24/24 ✓

## What Each Test Suite Validates

### Differential Drive
- Tank-style steering calculations
- PWM scaling and overflow protection
- Motor trim adjustments
- Turn-in-place and arc turning

### Mecanum Drive
- Omnidirectional movement (forward, strafe, rotate)
- Combined movement (e.g., forward + rotate)
- 4-motor kinematics calculations
- PWM overflow prevention

### Ring Buffer
- FIFO circular buffer behavior
- Push/pop operations
- Peek without removal
- Wrap-around at buffer boundaries
- Overflow protection

### iBUS Protocol
- CRC calculation and validation
- Message structure parsing
- 14-channel data extraction
- Sensor configuration (2-byte and 4-byte)
- RC stick and switch value handling

## Common Test Assertions

```cpp
ASSERT_TRUE(condition)              // Must be true
ASSERT_EQUAL(expected, actual)      // Values must match
ASSERT_FLOAT_EQUAL(exp, act, tol)   // Floats within tolerance
ASSERT_IN_RANGE(val, min, max)      // Value in range
```

## File Locations

```
test/
├── test_differential.cpp    # Tank drive tests
├── test_mecanum.cpp         # Mecanum drive tests
├── test_ring_buffer.cpp     # Circular buffer tests
├── test_ibus.cpp           # RC protocol tests
├── simple_test.h           # Test framework
├── Makefile                # Build configuration
└── build/                  # Executables (auto-generated)
```

## Typical Workflow

1. Make code changes to source files
2. Run relevant tests: `make test-<module>`
3. If tests fail, fix code
4. Run all tests: `make test`
5. Commit when all tests pass

## Exit Codes

- **0** = All tests passed
- **1** = One or more tests failed

Perfect for CI/CD integration!

## Need Help?

- See `EXPANDED_TEST_FRAMEWORK.md` for full documentation
- Check `README.md` for detailed test descriptions
- Review test source files for examples

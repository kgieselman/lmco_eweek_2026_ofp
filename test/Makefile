###############################################################################
# Makefile for Unit Tests
#
# Builds and runs unit tests on host machine (not on Pico)
#
# Usage:
#   make        - Build all tests
#   make test   - Build and run all tests
#   make clean  - Remove build artifacts
###############################################################################

# Compiler settings
CXX      = g++
CXXFLAGS = -Wall -Wextra -std=c++17 -I../inc -Iinclude
LDFLAGS  = -lm

# Directories
BUILD_DIR = build

# Test executables
TESTS = test_ring_buffer test_ibus test_differential test_motor_driver

# Default target
.PHONY: all
all: $(TESTS)

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Build ring buffer test
test_ring_buffer: $(BUILD_DIR) test_ring_buffer.cpp unit_test.h
	$(CXX) $(CXXFLAGS) test_ring_buffer.cpp -o $(BUILD_DIR)/test_ring_buffer $(LDFLAGS)
	@echo "Built: $(BUILD_DIR)/test_ring_buffer"

# Build iBUS test
test_ibus: $(BUILD_DIR) test_ibus.cpp unit_test.h
	$(CXX) $(CXXFLAGS) test_ibus.cpp -o $(BUILD_DIR)/test_ibus $(LDFLAGS)
	@echo "Built: $(BUILD_DIR)/test_ibus"

# Build differential drive test
test_differential: $(BUILD_DIR) test_differential.cpp unit_test.h
	$(CXX) $(CXXFLAGS) test_differential.cpp -o $(BUILD_DIR)/test_differential $(LDFLAGS)
	@echo "Built: $(BUILD_DIR)/test_differential"

# Build motor driver test
test_motor_driver: $(BUILD_DIR) test_motor_driver.cpp unit_test.h
	$(CXX) $(CXXFLAGS) test_motor_driver.cpp -o $(BUILD_DIR)/test_motor_driver $(LDFLAGS)
	@echo "Built: $(BUILD_DIR)/test_motor_driver"

# Run all tests
.PHONY: test
test: all
	@echo ""
	@echo "========================================"
	@echo "Running All Unit Tests"
	@echo "========================================"
	@./$(BUILD_DIR)/test_ring_buffer
	@./$(BUILD_DIR)/test_ibus
	@./$(BUILD_DIR)/test_differential
	@./$(BUILD_DIR)/test_motor_driver
	@echo "========================================"
	@echo "All tests complete"
	@echo "========================================"

# Individual test targets
.PHONY: test-ring
test-ring: test_ring_buffer
	@./$(BUILD_DIR)/test_ring_buffer

.PHONY: test-ibus
test-ibus: test_ibus
	@./$(BUILD_DIR)/test_ibus

.PHONY: test-diff
test-diff: test_differential
	@./$(BUILD_DIR)/test_differential

.PHONY: test-motor
test-motor: test_motor_driver
	@./$(BUILD_DIR)/test_motor_driver

# Clean build artifacts
.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)
	@echo "Cleaned build directory"

# Help
.PHONY: help
help:
	@echo "Available targets:"
	@echo "  all           - Build all tests (default)"
	@echo "  test          - Build and run all tests"
	@echo "  test-ring     - Run ring buffer tests only"
	@echo "  test-ibus     - Run iBUS protocol tests only"
	@echo "  test-diff     - Run differential drive tests only"
	@echo "  test-motor    - Run motor driver tests only"
	@echo "  clean         - Remove build artifacts"
	@echo "  help          - Show this help message"

# EOF

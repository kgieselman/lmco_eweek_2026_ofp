###############################################################################
# Makefile for Unit Tests
# 
# Builds and runs unit tests
###############################################################################

# Compiler settings
CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++11 -I../inc -Iinclude
LDFLAGS = -lm

# Directories
TEST_DIR = .
INC_DIR = ../inc
BUILD_DIR = build

# Test executables
TESTS = test_differential test_mecanum test_ring_buffer test_ibus

# Default target
.PHONY: all
all: $(TESTS)

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Build differential drive test
test_differential: $(BUILD_DIR) test_differential.cpp simple_test.h
	$(CXX) $(CXXFLAGS) test_differential.cpp -o $(BUILD_DIR)/test_differential $(LDFLAGS)
	@echo "Built: $(BUILD_DIR)/test_differential"

# Build mecanum drive test
test_mecanum: $(BUILD_DIR) test_mecanum.cpp simple_test.h
	$(CXX) $(CXXFLAGS) test_mecanum.cpp -o $(BUILD_DIR)/test_mecanum $(LDFLAGS)
	@echo "Built: $(BUILD_DIR)/test_mecanum"

# Build ring buffer test
test_ring_buffer: $(BUILD_DIR) test_ring_buffer.cpp simple_test.h
	$(CXX) $(CXXFLAGS) test_ring_buffer.cpp -o $(BUILD_DIR)/test_ring_buffer $(LDFLAGS)
	@echo "Built: $(BUILD_DIR)/test_ring_buffer"

# Build iBUS test
test_ibus: $(BUILD_DIR) test_ibus.cpp simple_test.h
	$(CXX) $(CXXFLAGS) test_ibus.cpp -o $(BUILD_DIR)/test_ibus $(LDFLAGS)
	@echo "Built: $(BUILD_DIR)/test_ibus"

# Run all tests
.PHONY: test
test: all
	@echo ""
	@echo "=========================================="
	@echo "Running All Unit Tests"
	@echo "=========================================="
	@./$(BUILD_DIR)/test_differential
	@./$(BUILD_DIR)/test_mecanum
	@./$(BUILD_DIR)/test_ring_buffer
	@./$(BUILD_DIR)/test_ibus

# Run differential tests only
.PHONY: test-diff
test-diff: test_differential
	@./$(BUILD_DIR)/test_differential

# Run mecanum tests only
.PHONY: test-mecanum
test-mecanum: test_mecanum
	@./$(BUILD_DIR)/test_mecanum

# Run ring buffer tests only
.PHONY: test-ring
test-ring: test_ring_buffer
	@./$(BUILD_DIR)/test_ring_buffer

# Run iBUS tests only
.PHONY: test-ibus
test-ibus: test_ibus
	@./$(BUILD_DIR)/test_ibus

# Clean build artifacts
.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)
	@echo "Cleaned build directory"

# Show help
.PHONY: help
help:
	@echo "Available targets:"
	@echo "  all            - Build all tests (default)"
	@echo "  test           - Build and run all tests"
	@echo "  test-diff      - Build and run differential drive tests only"
	@echo "  test-mecanum   - Build and run mecanum drive tests only"
	@echo "  test-ring      - Build and run ring buffer tests only"
	@echo "  test-ibus      - Build and run iBUS protocol tests only"
	@echo "  clean          - Remove build artifacts"
	@echo "  help           - Show this help message"


# EOF

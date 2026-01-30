/******************************************************************************
 * @file unit_test.h
 * @brief Lightweight unit testing framework for embedded systems
 * 
 * This framework provides simple test macros without requiring external
 * dependencies, making it suitable for Raspberry Pi Pico and other
 * embedded platforms.
 *****************************************************************************/
#pragma once

#include <stdio.h>
#include <string.h>
#include <math.h>

/* Test Statistics */
static int g_tests_run = 0;
static int g_tests_passed = 0;
static int g_tests_failed = 0;
static const char* g_current_suite = "";

/* Color codes for terminal output (optional) */
#define COLOR_RESET   "\033[0m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_RED     "\033[31m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_CYAN    "\033[36m"

/* Test Macros */
#define TEST_SUITE_BEGIN(name) \
    namespace test_suite_##name {

#define TEST_SUITE_END() \
    }

#define TEST(name) \
    static void test_##name(void); \
    static void test_runner_##name(void) { \
        printf("  Test: %s ... ", #name); \
        g_tests_run++; \
        test_##name(); \
    } \
    static void test_##name(void)

#define RUN_TEST_SUITE(suite_name) \
    printf("\n" COLOR_CYAN "═══════════════════════════════════════════════════\n"); \
    printf("Test Suite: %s\n", #suite_name); \
    printf("═══════════════════════════════════════════════════" COLOR_RESET "\n");

#define RUN_TEST(suite, name) test_suite_##suite::test_runner_##name()

/* Assertion Macros */
#define ASSERT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            printf(COLOR_RED "FAILED" COLOR_RESET "\n"); \
            printf("    ✗ Assertion failed: %s\n", #condition); \
            printf("    Location: %s:%d\n", __FILE__, __LINE__); \
            g_tests_failed++; \
            return; \
        } \
    } while(0)

#define ASSERT_FALSE(condition) ASSERT_TRUE(!(condition))

#define ASSERT_EQUAL(expected, actual) \
    do { \
        if ((expected) != (actual)) { \
            printf(COLOR_RED "FAILED" COLOR_RESET "\n"); \
            printf("    ✗ Expected: %d, Actual: %d\n", (int)(expected), (int)(actual)); \
            printf("    Location: %s:%d\n", __FILE__, __LINE__); \
            g_tests_failed++; \
            return; \
        } \
    } while(0)

#define ASSERT_NOT_EQUAL(not_expected, actual) \
    do { \
        if ((not_expected) == (actual)) { \
            printf(COLOR_RED "FAILED" COLOR_RESET "\n"); \
            printf("    ✗ Value should not be: %d\n", (int)(not_expected)); \
            printf("    Location: %s:%d\n", __FILE__, __LINE__); \
            g_tests_failed++; \
            return; \
        } \
    } while(0)

#define ASSERT_FLOAT_EQUAL(expected, actual, tolerance) \
    do { \
        float diff = fabs((expected) - (actual)); \
        if (diff > (tolerance)) { \
            printf(COLOR_RED "FAILED" COLOR_RESET "\n"); \
            printf("    ✗ Expected: %.6f, Actual: %.6f (tolerance: %.6f)\n", \
                   (float)(expected), (float)(actual), (float)(tolerance)); \
            printf("    Difference: %.6f\n", diff); \
            printf("    Location: %s:%d\n", __FILE__, __LINE__); \
            g_tests_failed++; \
            return; \
        } \
    } while(0)

#define ASSERT_IN_RANGE(value, min, max) \
    do { \
        if ((value) < (min) || (value) > (max)) { \
            printf(COLOR_RED "FAILED" COLOR_RESET "\n"); \
            printf("    ✗ Value %d not in range [%d, %d]\n", \
                   (int)(value), (int)(min), (int)(max)); \
            printf("    Location: %s:%d\n", __FILE__, __LINE__); \
            g_tests_failed++; \
            return; \
        } \
    } while(0)

#define ASSERT_NULL(ptr) \
    do { \
        if ((ptr) != NULL) { \
            printf(COLOR_RED "FAILED" COLOR_RESET "\n"); \
            printf("    ✗ Pointer should be NULL but was %p\n", (void*)(ptr)); \
            printf("    Location: %s:%d\n", __FILE__, __LINE__); \
            g_tests_failed++; \
            return; \
        } \
    } while(0)

#define ASSERT_NOT_NULL(ptr) \
    do { \
        if ((ptr) == NULL) { \
            printf(COLOR_RED "FAILED" COLOR_RESET "\n"); \
            printf("    ✗ Pointer should not be NULL\n"); \
            printf("    Location: %s:%d\n", __FILE__, __LINE__); \
            g_tests_failed++; \
            return; \
        } \
    } while(0)

#define TEST_PASS() \
    do { \
        printf(COLOR_GREEN "PASSED" COLOR_RESET "\n"); \
        g_tests_passed++; \
    } while(0)

/* Test Runner Utilities */
#define PRINT_TEST_SUMMARY() \
    do { \
        printf("\n" COLOR_CYAN "═══════════════════════════════════════════════════\n"); \
        printf("Test Summary\n"); \
        printf("═══════════════════════════════════════════════════" COLOR_RESET "\n"); \
        printf("Total Tests: %d\n", g_tests_run); \
        printf(COLOR_GREEN "Passed: %d" COLOR_RESET "\n", g_tests_passed); \
        if (g_tests_failed > 0) { \
            printf(COLOR_RED "Failed: %d" COLOR_RESET "\n", g_tests_failed); \
        } else { \
            printf("Failed: 0\n"); \
        } \
        float pass_rate = (g_tests_run > 0) ? (100.0f * g_tests_passed / g_tests_run) : 0.0f; \
        printf("Pass Rate: %.1f%%\n", pass_rate); \
        printf(COLOR_CYAN "═══════════════════════════════════════════════════" COLOR_RESET "\n\n"); \
    } while(0)

#define RESET_TEST_STATS() \
    do { \
        g_tests_run = 0; \
        g_tests_passed = 0; \
        g_tests_failed = 0; \
    } while(0)

/* Helper macro to verify tests don't crash */
#define VERIFY_NO_CRASH(code) \
    do { \
        code; \
        TEST_PASS(); \
    } while(0)


/* EOF ----------------------------------------------------------------------*/

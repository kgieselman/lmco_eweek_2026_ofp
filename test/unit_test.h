/******************************************************************************
 * @file unit_test.h
 * @brief Simple unit testing framework for embedded systems
 *****************************************************************************/
#pragma once


/* Includes -----------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>


/* Global Variables ---------------------------------------------------------*/
static int g_tests_run = 0;
static int g_tests_passed = 0;
static int g_tests_failed = 0;

// Terminal color codes
#define C_RESET   "\033[0m"
#define C_GREEN   "\033[32m"
#define C_RED     "\033[31m"
#define C_CYAN    "\033[36m"


/* Assertion Macros ---------------------------------------------------------*/
#define ASSERT_TRUE(condition) \
    if (!(condition)) { \
        printf(C_RED "✗ FAIL" C_RESET " - Line %d: %s\n", __LINE__, #condition); \
        g_tests_failed++; \
        return; \
    }

#define ASSERT_EQUAL(expected, actual) \
    if ((expected) != (actual)) { \
        printf(C_RED "✗ FAIL" C_RESET " - Line %d: Expected %d, got %d\n", \
               __LINE__, (int)(expected), (int)(actual)); \
        g_tests_failed++; \
        return; \
    }

#define ASSERT_FLOAT_EQUAL(expected, actual, tolerance) \
    if (fabs((expected) - (actual)) > (tolerance)) { \
        printf(C_RED "✗ FAIL" C_RESET " - Line %d: Expected %.3f, got %.3f\n", \
               __LINE__, (float)(expected), (float)(actual)); \
        g_tests_failed++; \
        return; \
    }

#define ASSERT_IN_RANGE(value, min, max) \
    if ((value) < (min) || (value) > (max)) { \
        printf(C_RED "✗ FAIL" C_RESET " - Line %d: %d not in [%d,%d]\n", \
               __LINE__, (int)(value), (int)(min), (int)(max)); \
        g_tests_failed++; \
        return; \
    }


/* Test definition macro ----------------------------------------------------*/
#define TEST_FUNC(name) void test_##name(void)


/* Test runner macro --------------------------------------------------------*/
#define RUN_TEST(name) \
    do { \
        printf("  %-50s ", #name); \
        fflush(stdout); \
        g_tests_run++; \
        test_##name(); \
        printf(C_GREEN "✓ PASS" C_RESET "\n"); \
        g_tests_passed++; \
    } while(0)


/* Test suite header --------------------------------------------------------*/
#define SUITE_START(name) \
    printf("\n" C_CYAN "══════════════════════════════════════════════\n"); \
    printf("  %s\n", name); \
    printf("══════════════════════════════════════════════" C_RESET "\n");


/* Summary ------------------------------------------------------------------*/
#define PRINT_SUMMARY() \
    printf("\n" C_CYAN "══════════════════════════════════════════════\n"); \
    printf("  SUMMARY\n"); \
    printf("══════════════════════════════════════════════" C_RESET "\n"); \
    printf("Total:  %d\n", g_tests_run); \
    printf(C_GREEN "Passed: %d" C_RESET "\n", g_tests_passed); \
    if (g_tests_failed > 0) { \
        printf(C_RED "Failed: %d" C_RESET "\n", g_tests_failed); \
    } else { \
        printf("Failed: 0\n"); \
    } \
    float rate = g_tests_run > 0 ? (100.0f * g_tests_passed / g_tests_run) : 0.0f; \
    printf("Rate:   %.1f%%\n\n", rate);


/* EOF ----------------------------------------------------------------------*/

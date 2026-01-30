/*******************************************************************************
 * @file unit_test.h
 * @brief Lightweight unit testing framework for embedded systems
 *
 * This header provides a simple testing framework that can run on the host
 * machine without requiring embedded hardware.
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>
#include <stdint.h>


/* Global Variables ----------------------------------------------------------*/
static int g_tests_run = 0;
static int g_tests_passed = 0;
static int g_tests_failed = 0;


/* Defines -------------------------------------------------------------------*/
#define COLOR_RESET   "\033[0m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_RED     "\033[31m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_YELLOW  "\033[33m"


/* Macros --------------------------------------------------------------------*/
#define ASSERT_TRUE(condition) \
  do { \
    if (!(condition)) { \
      printf(COLOR_RED "  FAIL" COLOR_RESET " Line %d: %s\n", __LINE__, #condition); \
      g_tests_failed++; \
      return; \
    } \
  } while(0)

#define ASSERT_FALSE(condition) \
  do { \
    if ((condition)) { \
      printf(COLOR_RED "  FAIL" COLOR_RESET " Line %d: Expected false: %s\n", __LINE__, #condition); \
      g_tests_failed++; \
      return; \
    } \
  } while(0)

#define ASSERT_EQUAL(expected, actual) \
  do { \
    if ((expected) != (actual)) { \
      printf(COLOR_RED "  FAIL" COLOR_RESET " Line %d: Expected %d, got %d\n", \
             __LINE__, (int)(expected), (int)(actual)); \
      g_tests_failed++; \
      return; \
    } \
  } while(0)

#define ASSERT_NOT_EQUAL(unexpected, actual) \
  do { \
    if ((unexpected) == (actual)) { \
      printf(COLOR_RED "  FAIL" COLOR_RESET " Line %d: Should not equal %d\n", __LINE__, (int)(actual)); \
      g_tests_failed++; \
      return; \
    } \
  } while(0)

#define ASSERT_FLOAT_EQUAL(expected, actual, tolerance) \
  do { \
    if (fabs((expected) - (actual)) > (tolerance)) { \
      printf(COLOR_RED "  FAIL" COLOR_RESET " Line %d: Expected %.6f, got %.6f\n", \
             __LINE__, (double)(expected), (double)(actual)); \
      g_tests_failed++; \
      return; \
    } \
  } while(0)

#define ASSERT_IN_RANGE(value, min, max) \
  do { \
    if ((value) < (min) || (value) > (max)) { \
      printf(COLOR_RED "  FAIL" COLOR_RESET " Line %d: %d not in [%d, %d]\n", \
             __LINE__, (int)(value), (int)(min), (int)(max)); \
      g_tests_failed++; \
      return; \
    } \
  } while(0)

#define ASSERT_NOT_NULL(ptr) \
  do { \
    if ((ptr) == nullptr) { \
      printf(COLOR_RED "  FAIL" COLOR_RESET " Line %d: Pointer is null\n", __LINE__); \
      g_tests_failed++; \
      return; \
    } \
  } while(0)

/** @brief Define a test function */
#define TEST_FUNC(name) void test_##name(void)

/** @brief Run a test and report result */
#define RUN_TEST(name) \
  do { \
    printf("  %-45s ", #name); \
    fflush(stdout); \
    g_tests_run++; \
    test_##name(); \
    printf(COLOR_GREEN "PASS" COLOR_RESET "\n"); \
    g_tests_passed++; \
  } while(0)

/** @brief Print a test suite header */
#define SUITE_START(name) \
  printf("\n" COLOR_CYAN "--- %s ---" COLOR_RESET "\n", name);

/** @brief Print test summary */
#define PRINT_SUMMARY() \
  do { \
    printf("\n" COLOR_CYAN "=== SUMMARY ===" COLOR_RESET "\n"); \
    printf("  Total:  %d\n", g_tests_run); \
    printf("  " COLOR_GREEN "Passed: %d" COLOR_RESET "\n", g_tests_passed); \
    if (g_tests_failed > 0) { \
      printf("  " COLOR_RED "Failed: %d" COLOR_RESET "\n", g_tests_failed); \
    } else { \
      printf("  Failed: 0\n"); \
    } \
    float rate = g_tests_run > 0 ? (100.0f * g_tests_passed / g_tests_run) : 0.0f; \
    printf("  Rate:   %.1f%%\n\n", rate); \
  } while(0)


/* EOF -----------------------------------------------------------------------*/

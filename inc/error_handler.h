/*******************************************************************************
 * @file error_handler.h
 * @brief Error handling utilities for the robot control system
 *
 * Provides error codes, error handling functions, and debug output utilities.
 * Designed for embedded systems with limited resources.
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Types ---------------------------------------------------------------------*/

/*******************************************************************************
 * @brief Error code enumeration
 *
 * Error codes are organized by module:
 * - 0x00XX: General errors
 * - 0x01XX: Drive train errors
 * - 0x02XX: iBUS/RC errors
 * - 0x03XX: Mechanism errors
 * - 0x04XX: Hardware errors
 ******************************************************************************/
typedef enum {
  /* General errors (0x00XX) */
  ERROR_NONE              = 0x0000,  /**< No error */
  ERROR_INVALID_PARAM     = 0x0001,  /**< Invalid parameter passed */
  ERROR_NULL_POINTER      = 0x0002,  /**< Null pointer encountered */
  ERROR_OUT_OF_RANGE      = 0x0003,  /**< Value out of valid range */
  ERROR_NOT_INITIALIZED   = 0x0004,  /**< Module not initialized */
  ERROR_ALREADY_INIT      = 0x0005,  /**< Module already initialized */
  ERROR_TIMEOUT           = 0x0006,  /**< Operation timed out */
  ERROR_BUFFER_FULL       = 0x0007,  /**< Buffer is full */
  ERROR_BUFFER_EMPTY      = 0x0008,  /**< Buffer is empty */

  /* Drive train errors (0x01XX) */
  ERROR_DT_INVALID_MOTOR  = 0x0100,  /**< Invalid motor index */
  ERROR_DT_INVALID_PIN    = 0x0101,  /**< Invalid GPIO pin number */
  ERROR_DT_NOT_INIT       = 0x0102,  /**< Motor not initialized */
  ERROR_DT_PWM_FAILED     = 0x0103,  /**< PWM initialization failed */
  ERROR_DT_ENCODER_FAILED = 0x0104,  /**< Encoder initialization failed */

  /* iBUS/RC errors (0x02XX) */
  ERROR_IBUS_INVALID_UART = 0x0200,  /**< Invalid UART instance */
  ERROR_IBUS_INIT_FAILED  = 0x0201,  /**< iBUS initialization failed */
  ERROR_IBUS_CRC_MISMATCH = 0x0202,  /**< CRC validation failed */
  ERROR_IBUS_MSG_TOO_SHORT= 0x0203,  /**< Message too short */
  ERROR_IBUS_MSG_TOO_LONG = 0x0204,  /**< Message too long */
  ERROR_IBUS_SIGNAL_LOST  = 0x0205,  /**< RC signal lost */
  ERROR_IBUS_INVALID_CHAN = 0x0206,  /**< Invalid channel number */

  /* Mechanism errors (0x03XX) */
  ERROR_MECH_NOT_INIT     = 0x0300,  /**< Mechanism not initialized */
  ERROR_MECH_BUSY         = 0x0301,  /**< Mechanism busy */
  ERROR_MECH_FAULT        = 0x0302,  /**< Mechanism fault detected */
  ERROR_MECH_LIMIT_HIT    = 0x0303,  /**< Limit switch triggered */

  /* Hardware errors (0x04XX) */
  ERROR_HW_GPIO_FAILED    = 0x0400,  /**< GPIO initialization failed */
  ERROR_HW_PWM_FAILED     = 0x0401,  /**< PWM hardware error */
  ERROR_HW_UART_FAILED    = 0x0402,  /**< UART hardware error */
  ERROR_HW_WATCHDOG       = 0x0403,  /**< Watchdog reset occurred */
} ErrorCode_t;

/* Function Definitions ------------------------------------------------------*/

/*******************************************************************************
 * @brief Initialize the error handler
 *
 * Must be called before using any error handler functions.
 ******************************************************************************/
void error_handler_init(void);

/*******************************************************************************
 * @brief Report an error
 *
 * Logs the error and takes appropriate action based on severity.
 * In debug mode, prints error information to stdio.
 *
 * @param code Error code to report
 * @param file Source file where error occurred (use __FILE__)
 * @param line Line number where error occurred (use __LINE__)
 ******************************************************************************/
void error_report(ErrorCode_t code, const char* file, int line);

/*******************************************************************************
 * @brief Get the last reported error code
 *
 * @return Last error code, or ERROR_NONE if no errors
 ******************************************************************************/
ErrorCode_t error_get_last(void);

/*******************************************************************************
 * @brief Clear the last error
 ******************************************************************************/
void error_clear(void);

/*******************************************************************************
 * @brief Get error count since initialization
 *
 * @return Total number of errors reported
 ******************************************************************************/
uint32_t error_get_count(void);

/*******************************************************************************
 * @brief Check if system is in error state
 *
 * @return true if uncleared error exists, false otherwise
 ******************************************************************************/
bool error_is_active(void);

/*******************************************************************************
 * @brief Get human-readable error string
 *
 * @param code Error code to look up
 * @return Pointer to static string describing the error
 ******************************************************************************/
const char* error_get_string(ErrorCode_t code);


/* Macros --------------------------------------------------------------------*/

/*******************************************************************************
 * @brief Report an error with file and line information
 *
 * @param code Error code to report
 ******************************************************************************/
#define ERROR_REPORT(code) error_report((code), __FILE__, __LINE__)

/*******************************************************************************
 * @brief Assert a condition and report error if false
 *
 * @param condition Condition to check
 * @param code Error code to report if condition is false
 ******************************************************************************/
#define ERROR_ASSERT(condition, code) \
  do { \
    if (!(condition)) { \
      ERROR_REPORT(code); \
    } \
  } while(0)

/*******************************************************************************
 * @brief Return from function if pointer is null
 *
 * @param ptr Pointer to check
 * @param retval Value to return if null
 ******************************************************************************/
#define ERROR_CHECK_NULL(ptr, retval) \
  do { \
    if ((ptr) == nullptr) { \
      ERROR_REPORT(ERROR_NULL_POINTER); \
      return (retval); \
    } \
  } while(0)

/*******************************************************************************
 * @brief Return from function if condition is false
 *
 * @param condition Condition to check
 * @param code Error code to report
 * @param retval Value to return if condition is false
 ******************************************************************************/
#define ERROR_CHECK(condition, code, retval) \
  do { \
    if (!(condition)) { \
      ERROR_REPORT(code); \
      return (retval); \
    } \
  } while(0)

#ifdef __cplusplus
}
#endif


/* EOF -----------------------------------------------------------------------*/

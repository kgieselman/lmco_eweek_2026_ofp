/*******************************************************************************
 * @file error_handler.cpp
 * @brief Implementation of error handling utilities
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "error_handler.h"
#include "config.h"

#if ENABLE_DEBUG
#include <stdio.h>
#endif


/* Private Static Variable Declaration ---------------------------------------*/

/** @brief Last reported error code */
static ErrorCode_t s_lastError = ERROR_NONE;

/** @brief Total error count since initialization */
static uint32_t s_errorCount = 0;

/** @brief Initialization flag */
static bool s_initialized = false;

/* Types ---------------------------------------------------------------------*/
typedef struct
{
  ErrorCode_t code;
  const char* pStr;
} errorString_t;


/* Constants -----------------------------------------------------------------*/

/*******************************************************************************
 * @brief Error code to string mapping
 ******************************************************************************/
static const errorString_t s_errorStrings[] =
{
  /* General errors */
  { ERROR_NONE,              "No error" },
  { ERROR_INVALID_PARAM,     "Invalid parameter" },
  { ERROR_NULL_POINTER,      "Null pointer" },
  { ERROR_OUT_OF_RANGE,      "Value out of range" },
  { ERROR_NOT_INITIALIZED,   "Not initialized" },
  { ERROR_ALREADY_INIT,      "Already initialized" },
  { ERROR_TIMEOUT,           "Operation timed out" },
  { ERROR_BUFFER_FULL,       "Buffer full" },
  { ERROR_BUFFER_EMPTY,      "Buffer empty" },

  /* Drive train errors */
  { ERROR_DT_INVALID_MOTOR,  "Invalid motor index" },
  { ERROR_DT_INVALID_PIN,    "Invalid GPIO pin" },
  { ERROR_DT_NOT_INIT,       "Motor not initialized" },
  { ERROR_DT_PWM_FAILED,     "PWM init failed" },
  { ERROR_DT_ENCODER_FAILED, "Encoder init failed" },

  /* iBUS/RC errors */
  { ERROR_IBUS_INVALID_UART, "Invalid UART" },
  { ERROR_IBUS_INIT_FAILED,  "iBUS init failed" },
  { ERROR_IBUS_CRC_MISMATCH, "CRC mismatch" },
  { ERROR_IBUS_MSG_TOO_SHORT,"Message too short" },
  { ERROR_IBUS_MSG_TOO_LONG, "Message too long" },
  { ERROR_IBUS_SIGNAL_LOST,  "RC signal lost" },
  { ERROR_IBUS_INVALID_CHAN, "Invalid channel" },

  /* Mechanism errors */
  { ERROR_MECH_NOT_INIT,     "Mechanism not init" },
  { ERROR_MECH_BUSY,         "Mechanism busy" },
  { ERROR_MECH_FAULT,        "Mechanism fault" },
  { ERROR_MECH_LIMIT_HIT,    "Limit switch hit" },

  /* Hardware errors */
  { ERROR_HW_GPIO_FAILED,    "GPIO init failed" },
  { ERROR_HW_PWM_FAILED,     "PWM hardware error" },
  { ERROR_HW_UART_FAILED,    "UART hardware error" },
  { ERROR_HW_WATCHDOG,       "Watchdog reset" },
};

/** @brief Number of entries in error string table */
static const size_t s_errorStringCount = sizeof(s_errorStrings) /
                                         sizeof(s_errorStrings[0]);


/* Function Definitions ------------------------------------------------------*/
void error_handler_init(void)
{
  s_lastError = ERROR_NONE;
  s_errorCount = 0;
  s_initialized = true;
}

void error_report(ErrorCode_t code, const char* file, int line)
{
  s_lastError = code;
  s_errorCount++;

#if ENABLE_DEBUG
  if (code != ERROR_NONE)
  {
    /* Extract just the filename from the path */
    const char* filename = file;
    for (const char* p = file; *p != '\0'; p++)
    {
      if (*p == '/' || *p == '\\')
      {
        filename = p + 1;
      }
    }

    printf("[ERROR] 0x%04X: %s (%s:%d)\n", code,
                                           error_get_string(code),
                                           filename,
                                           line);
  }
#else
  /* Suppress unused parameter warnings in release builds */
  (void)file;
  (void)line;
#endif
}

ErrorCode_t error_get_last(void)
{
  return s_lastError;
}

void error_clear(void)
{
  s_lastError = ERROR_NONE;
}

uint32_t error_get_count(void)
{
  return s_errorCount;
}

bool error_is_active(void)
{
  return (s_lastError != ERROR_NONE);
}

const char* error_get_string(ErrorCode_t code)
{
  for (size_t i = 0; i < s_errorStringCount; i++) {
    if (s_errorStrings[i].code == code) {
      return s_errorStrings[i].pStr;
    }
  }

  return "Unknown error";
}


/* EOF -----------------------------------------------------------------------*/

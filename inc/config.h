/*******************************************************************************
 * @file config.h
 * @brief Project configuration and feature flags
 *
 * This file contains compile-time configuration options for the E-Week 2026
 * robot control system. Modify these settings to customize behavior.
 *
 * @section config_sections Configuration Sections
 * - Hardware Selection: Motor driver wiring mode
 * - Feature Enables: Debug output, watchdog, safety features
 * - Timing Parameters: Timeouts, loop rates, calibration durations
 ******************************************************************************/
#pragma once


/*============================================================================*/
/* Hardware Selection                                                         */
/*============================================================================*/

/** @defgroup hw_select Hardware Selection
 *  @brief Select which hardware components are installed
 *  @{
 */

/*******************************************************************************
 * @brief Motor driver wiring mode selection
 *
 * Enable exactly ONE of the following options:
 * - MOTOR_DRIVER_MODE_2PWM:      Two PWM pins per motor (e.g. DRV8833, TB6612FNG)
 * - MOTOR_DRIVER_MODE_1PWM_2DIR: One PWM + two direction pins per motor (e.g. L298N)
 ******************************************************************************/
#define MOTOR_DRIVER_MODE_2PWM        (0)
#define MOTOR_DRIVER_MODE_1PWM_2DIR   (1)

/** @} */ /* End of hw_select */


/*============================================================================*/
/* Feature Enables                                                            */
/*============================================================================*/

/** @defgroup features Feature Enables
 *  @brief Enable or disable optional features
 *  @{
 */

/*******************************************************************************
 * @brief Master debug output enable
 *
 * When enabled (1), diagnostic messages are printed to stdio.
 * Disable (0) for production to reduce code size and improve performance.
 ******************************************************************************/
#define ENABLE_DEBUG              (1)

/*******************************************************************************
 * @brief Verbose iBUS protocol debugging
 *
 * Prints detailed iBUS message information (length, command, CRC).
 * Only effective when ENABLE_DEBUG is also enabled.
 * Warning: High output volume may affect timing.
 ******************************************************************************/
#define ENABLE_DEBUG_IBUS_VERBOSE (0)

/*******************************************************************************
 * @brief Hardware watchdog timer
 *
 * Automatically resets the system if the main loop stalls.
 * Strongly recommended for competition use.
 ******************************************************************************/
#define ENABLE_WATCHDOG           (1)

/*******************************************************************************
 * @brief Motor safety cutoff on RC signal loss
 *
 * When enabled, all motors immediately stop if RC signal is lost.
 * Strongly recommended for safety.
 ******************************************************************************/
#define ENABLE_SIGNAL_LOSS_CUTOFF (1)

/*******************************************************************************
 * @brief Encoder-based motor calibration
 *
 * When enabled, the calibrate() function measures encoder pulses to
 * calculate per-motor trim values for straight-line driving.
 * Requires encoders to be wired and configured in pinout.h.
 ******************************************************************************/
#define ENABLE_ENCODER_CALIBRATION (0)

/*******************************************************************************
 * @brief UART debug output (GPIO 0/1)
 *
 * Enable stdio output over hardware UART.
 * Useful for debugging with a serial adapter.
 ******************************************************************************/
#define ENABLE_STDIO_UART         (1)

/*******************************************************************************
 * @brief USB CDC debug output
 *
 * Enable stdio output over USB serial.
 * Useful for debugging without additional hardware.
 * Note: Adds startup delay for USB enumeration.
 ******************************************************************************/
#define ENABLE_STDIO_USB          (1)

/** @} */ /* End of features */


/*============================================================================*/
/* Timing Parameters                                                          */
/*============================================================================*/

/** @defgroup timing Timing Parameters
 *  @brief Timeouts, periods, and durations (all values in milliseconds)
 *  @{
 */

/* System Timing ------------------------------------------------------------ */

/*******************************************************************************
 * @brief Main loop minimum period (ms)
 *
 * Enforces consistent loop timing. Set to 0 to disable rate limiting
 * and run as fast as possible.
 ******************************************************************************/
#define TIMING_MAIN_LOOP_PERIOD_MS      (1)

/*******************************************************************************
 * @brief Watchdog timeout (ms)
 *
 * Maximum time between watchdog feeds before system reset.
 * Valid range: 1 to 8388 ms (~8.3 seconds max on RP2040)
 ******************************************************************************/
#define TIMING_WATCHDOG_TIMEOUT_MS      (1000)

/*******************************************************************************
 * @brief USB enumeration delay (ms)
 *
 * Delay at startup to allow USB CDC to enumerate before printing.
 * Only applies when ENABLE_STDIO_USB is enabled.
 ******************************************************************************/
#define TIMING_USB_STARTUP_DELAY_MS     (2000)

/* RC Signal Timing --------------------------------------------------------- */

/*******************************************************************************
 * @brief RC signal loss timeout (ms)
 *
 * Time without valid RC messages before signal is considered lost.
 * Motors stop if ENABLE_SIGNAL_LOSS_CUTOFF is enabled.
 ******************************************************************************/
#define TIMING_RC_SIGNAL_TIMEOUT_MS     (500)

/*******************************************************************************
 * @brief Signal loss warning print interval (ms)
 *
 * Minimum time between "signal lost" debug messages to avoid flooding.
 ******************************************************************************/
#define TIMING_SIGNAL_LOSS_PRINT_MS     (1000)

/* Motor Calibration Timing ------------------------------------------------- */

/*******************************************************************************
 * @brief Motor settling time before calibration measurement (ms)
 *
 * Time to wait after setting motor speed before counting encoder pulses.
 * Allows motor to reach steady-state speed.
 ******************************************************************************/
#define TIMING_MOTOR_SETTLE_MS          (500)

/*******************************************************************************
 * @brief Encoder pulse counting duration (ms)
 *
 * Duration to count encoder pulses during calibration.
 * Longer = more accurate, but slower calibration.
 ******************************************************************************/
#define TIMING_MOTOR_CALIBRATION_MS     (2000)

/* UI/Feedback Timing ------------------------------------------------------- */

/*******************************************************************************
 * @brief Error LED blink half-period (ms)
 *
 * Time LED stays on (and off) when blinking to indicate error.
 * Full blink cycle = 2 * this value.
 ******************************************************************************/
#define TIMING_ERROR_LED_BLINK_MS       (250)

/** @} */ /* End of timing */


/*============================================================================*/
/* Configuration Validation                                                   */
/*============================================================================*/

/** @defgroup validation Compile-Time Validation
 *  @brief Static assertions to catch configuration errors
 *  @{
 */

/* Validate motor driver mode selection (exactly one must be enabled) */
#if (MOTOR_DRIVER_MODE_2PWM + MOTOR_DRIVER_MODE_1PWM_2DIR) == 0
  #error "No motor driver mode selected. Enable MOTOR_DRIVER_MODE_2PWM or MOTOR_DRIVER_MODE_1PWM_2DIR."
#elif (MOTOR_DRIVER_MODE_2PWM + MOTOR_DRIVER_MODE_1PWM_2DIR) > 1
  #error "Multiple motor driver modes selected. Enable only ONE of MOTOR_DRIVER_MODE_2PWM or MOTOR_DRIVER_MODE_1PWM_2DIR."
#endif

/* Validate watchdog timeout range */
#if ENABLE_WATCHDOG
  #if (TIMING_WATCHDOG_TIMEOUT_MS < 1) || (TIMING_WATCHDOG_TIMEOUT_MS > 8388)
    #error "TIMING_WATCHDOG_TIMEOUT_MS must be between 1 and 8388 ms."
  #endif
#endif

/* Warn if debug verbose enabled without master debug */
#if ENABLE_DEBUG_IBUS_VERBOSE && !ENABLE_DEBUG
  #warning "ENABLE_DEBUG_IBUS_VERBOSE has no effect when ENABLE_DEBUG is disabled."
#endif

/** @} */ /* End of validation */


/*============================================================================*/
/* Motor Driver Mode Alias                                                    */
/*============================================================================*/

/** @defgroup mode_alias Motor Driver Mode Alias
 *  @brief Maps config flag to MotorDriver::Mode_e at compile time
 *
 *  Provides a compile-time constant that drive train code can use to
 *  construct the motor driver with the correct mode.
 *  @{
 */

#include "motor_driver.h"

#if MOTOR_DRIVER_MODE_2PWM
  constexpr MotorDriver::Mode_e MOTOR_DRIVER_MODE = MotorDriver::MODE_2PWM;
#elif MOTOR_DRIVER_MODE_1PWM_2DIR
  constexpr MotorDriver::Mode_e MOTOR_DRIVER_MODE = MotorDriver::MODE_1PWM_2DIR;
#endif

/** @} */ /* End of mode_alias */


/* EOF -----------------------------------------------------------------------*/

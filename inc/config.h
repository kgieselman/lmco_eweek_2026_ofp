/*******************************************************************************
 * @file config.h
 * @brief Project configuration and feature flags
 *
 * This file contains compile-time configuration options for the E-Week 2026
 * robot control system. Modify these settings to customize behavior.
 *
 * @section config_sections Configuration Sections
 * - Hardware Selection: Drive train type, motor driver IC
 * - Feature Enables: Debug output, watchdog, safety features
 * - Timing Parameters: Timeouts, loop rates, calibration durations
 * - Type Aliases: Unified motor driver types (auto-generated)
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
 * @brief Drive train type selection
 *
 * Enable exactly ONE of the following options:
 * - DRIVE_TRAIN_DIFFERENTIAL: Two-wheel tank drive (skid steering)
 * - DRIVE_TRAIN_MECANUM: Four-wheel omnidirectional drive
 ******************************************************************************/
#define DRIVE_TRAIN_DIFFERENTIAL  (0)
#define DRIVE_TRAIN_MECANUM       (1)

/*******************************************************************************
 * @brief Motor driver IC selection
 *
 * Enable exactly ONE of the following options:
 * - MOTOR_DRIVER_L298N: Classic H-bridge (1 PWM + 2 DIR pins per motor)
 * - MOTOR_DRIVER_DRV8833: TI DRV8833 (2 PWM pins per motor, more efficient)
 ******************************************************************************/
#define MOTOR_DRIVER_L298N        (0)
#define MOTOR_DRIVER_DRV8833      (1)

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

/* Validate drive train selection (exactly one must be enabled) */
#if (DRIVE_TRAIN_DIFFERENTIAL + DRIVE_TRAIN_MECANUM) == 0
  #error "No drive train selected. Enable DRIVE_TRAIN_DIFFERENTIAL or DRIVE_TRAIN_MECANUM."
#elif (DRIVE_TRAIN_DIFFERENTIAL + DRIVE_TRAIN_MECANUM) > 1
  #error "Multiple drive trains selected. Enable only ONE of DRIVE_TRAIN_DIFFERENTIAL or DRIVE_TRAIN_MECANUM."
#endif

/* Validate motor driver selection (exactly one must be enabled) */
#if (MOTOR_DRIVER_L298N + MOTOR_DRIVER_DRV8833) == 0
  #error "No motor driver selected. Enable MOTOR_DRIVER_L298N or MOTOR_DRIVER_DRV8833."
#elif (MOTOR_DRIVER_L298N + MOTOR_DRIVER_DRV8833) > 1
  #error "Multiple motor drivers selected. Enable only ONE of MOTOR_DRIVER_L298N or MOTOR_DRIVER_DRV8833."
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
/* Motor Driver Type Aliases                                                  */
/*============================================================================*/

/** @defgroup type_aliases Motor Driver Type Aliases
 *  @brief Unified type names for hardware abstraction
 *
 *  These aliases allow drive train code to use consistent type names
 *  regardless of which motor driver is selected. This eliminates
 *  preprocessor conditionals throughout the codebase.
 *  @{
 */

#if MOTOR_DRIVER_DRV8833
  #include "motor_driver_drv8833.h"

  using MotorDriver  = MotorDriverDRV8833;
  using MotorChannel = MotorDriverDRV8833::MotorChannel_e;
  using StopMode     = MotorDriverDRV8833::StopMode_e;

  constexpr MotorChannel MOTOR_CHANNEL_A = MotorDriverDRV8833::MOTOR_A;
  constexpr MotorChannel MOTOR_CHANNEL_B = MotorDriverDRV8833::MOTOR_B;
  constexpr StopMode     STOP_MODE_COAST = MotorDriverDRV8833::STOP_COAST;
  constexpr StopMode     STOP_MODE_BRAKE = MotorDriverDRV8833::STOP_BRAKE;

#elif MOTOR_DRIVER_L298N
  #include "motor_driver_l298n.h"

  using MotorDriver  = MotorDriverL298N;
  using MotorChannel = MotorDriverL298N::MotorChannel_e;
  using StopMode     = MotorDriverL298N::StopMode_e;

  constexpr MotorChannel MOTOR_CHANNEL_A = MotorDriverL298N::MOTOR_A;
  constexpr MotorChannel MOTOR_CHANNEL_B = MotorDriverL298N::MOTOR_B;
  constexpr StopMode     STOP_MODE_COAST = MotorDriverL298N::STOP_COAST;
  constexpr StopMode     STOP_MODE_BRAKE = MotorDriverL298N::STOP_BRAKE;

#endif

/** @} */ /* End of type_aliases */


/* EOF -----------------------------------------------------------------------*/

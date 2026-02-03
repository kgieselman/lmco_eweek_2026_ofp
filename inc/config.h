/*******************************************************************************
 * @file config.h
 * @brief Project configuration and feature flags
 *
 * This file contains compile-time configuration options for the E-Week 2026
 * robot control system. Modify these settings to customize behavior.
 ******************************************************************************/
#pragma once


/* Defines -------------------------------------------------------------------*/

/*******************************************************************************
 * @brief Select the drive train type
 *
 * Set to 1 to use Mecanum (omnidirectional) drive train.
 * Set to 0 to use differential (tank) drive train.
 * 
 * @todo Update logic to support ENABLE_MECANUM and ENABLE_DIFFERENTIAL.
 *       Should panic/break if both are enabled.
 ******************************************************************************/
#define USE_DRIVE_TRAIN_MECANUM (1)

/*******************************************************************************
 * @brief Select the motor driver type
 *
 * Set to 1 to use DRV8833 motor driver (2 PWM pins per motor).
 * Set to 0 to use L298N motor driver (1 PWM + 2 direction pins per motor).
 ******************************************************************************/
#define USE_MOTOR_DRIVER_DRV8833 (1)


/* Motor Driver Type Aliases -------------------------------------------------*/
/*******************************************************************************
 * @brief Unified motor driver type aliases
 *
 * These aliases allow drive train code to use a single type name regardless
 * of which motor driver is selected. This eliminates preprocessor conditionals
 * throughout the codebase.
 *
 * @note Include the appropriate motor driver header before using these types.
 ******************************************************************************/
#if USE_MOTOR_DRIVER_DRV8833
  #include "motor_driver_drv8833.h"
  using MotorDriver  = MotorDriverDRV8833;
  using MotorChannel = MotorDriverDRV8833::MotorChannel_e;
  using StopMode     = MotorDriverDRV8833::StopMode_e;

  /** @brief Motor channel A constant */
  constexpr MotorChannel MOTOR_CHANNEL_A = MotorDriverDRV8833::MOTOR_A;
  /** @brief Motor channel B constant */
  constexpr MotorChannel MOTOR_CHANNEL_B = MotorDriverDRV8833::MOTOR_B;
  /** @brief Coast stop mode constant */
  constexpr StopMode STOP_MODE_COAST = MotorDriverDRV8833::STOP_COAST;
  /** @brief Brake stop mode constant */
  constexpr StopMode STOP_MODE_BRAKE = MotorDriverDRV8833::STOP_BRAKE;
#else
  #include "motor_driver_l298n.h"
  using MotorDriver  = MotorDriverL298N;
  using MotorChannel = MotorDriverL298N::MotorChannel_e;
  using StopMode     = MotorDriverL298N::StopMode_e;

  /** @brief Motor channel A constant */
  constexpr MotorChannel MOTOR_CHANNEL_A = MotorDriverL298N::MOTOR_A;
  /** @brief Motor channel B constant */
  constexpr MotorChannel MOTOR_CHANNEL_B = MotorDriverL298N::MOTOR_B;
  /** @brief Coast stop mode constant */
  constexpr StopMode STOP_MODE_COAST = MotorDriverL298N::STOP_COAST;
  /** @brief Brake stop mode constant */
  constexpr StopMode STOP_MODE_BRAKE = MotorDriverL298N::STOP_BRAKE;
#endif


/*******************************************************************************
 * @brief Enable debug output over UART/USB
 *
 * When enabled, diagnostic messages are printed to stdio.
 * Disable for production to improve performance.
 ******************************************************************************/
#define ENABLE_DEBUG (1)

/*******************************************************************************
 * @brief Enable verbose iBUS debugging
 *
 * Prints detailed iBUS protocol information.
 * Only effective when ENABLE_DEBUG is also enabled.
 ******************************************************************************/
#define DEBUG_IBUS_VERBOSE (0)

/*******************************************************************************
 * @brief Milliseconds between prints when sinal is lost
 ******************************************************************************/
#define SIGNAL_LOSS_PRINT_TIMEOUT_MS (1000)

/*******************************************************************************
 * @brief Milliseconds before toggling LED that signals error ocurred
 ******************************************************************************/
#define ERROR_LED_DUTY_TIME_MS (250)

/*******************************************************************************
 * @brief Enable the hardware watchdog timer
 *
 * The watchdog will reset the system if the main loop stalls.
 ******************************************************************************/
#define ENABLE_WATCHDOG (1)

/*******************************************************************************
 * @brief Watchdog timeout in milliseconds
 *
 * Maximum time between watchdog feeds before system reset.
 * Range: 1 to 8388 ms (approximately 8.3 seconds)
 ******************************************************************************/
#define WATCHDOG_TIMEOUT_MS (1000)

/*******************************************************************************
 * @brief RC signal timeout in milliseconds
 *
 * If no valid RC signal is received within this time, motors are stopped.
 ******************************************************************************/
#define RC_SIGNAL_TIMEOUT_MS (500)

/*******************************************************************************
 * @brief Enable motor safety cutoff on signal loss
 *
 * When enabled, all motors stop if RC signal is lost.
 ******************************************************************************/
#define ENABLE_SIGNAL_LOSS_CUTOFF (1)

/*******************************************************************************
 * @brief Main loop minimum period in milliseconds
 *
 * Ensures consistent timing even if processing completes early.
 * Set to 0 to disable rate limiting.
 * 
 * @todo Consider microseconds to avoid rounding due to 1 MS period
 ******************************************************************************/
#define MAIN_LOOP_PERIOD_MS (1)

/*******************************************************************************
 * @brief Default PWM frequency divider
 *
 * Adjusts PWM frequency. Higher values = lower frequency.
 ******************************************************************************/
#define PWM_CLOCK_DIVIDER (4.0f)

/*******************************************************************************
 * @brief Motor settling time during calibration (ms)
 ******************************************************************************/
#define MOTOR_SETTLE_TIME_MS (500)

/*******************************************************************************
 * @brief Motor pulse counting time during calibration (ms)
 ******************************************************************************/
#define MOTOR_CALIBRATION_TIME_MS (2000)

/*******************************************************************************
 * @brief Enable encoder feedback for motor calibration
 ******************************************************************************/
#define ENABLE_ENCODER_CALIBRATION (0)

/*******************************************************************************
 * @brief Enable UART output for stdio
 ******************************************************************************/
#define ENABLE_STDIO_UART (1)

/*******************************************************************************
 * @brief Enable USB output for stdio
 ******************************************************************************/
#define ENABLE_STDIO_USB (1)

/*******************************************************************************
 * @brief Milliseconds to delay and allow USB to enumerate and work for stdio
 ******************************************************************************/
#define STDIO_USB_DELAY_MS (2000)


/* EOF -----------------------------------------------------------------------*/

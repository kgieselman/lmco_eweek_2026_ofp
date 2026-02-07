/*******************************************************************************
 * @file pinout.h
 * @brief GPIO pin definitions for the robot control system
 *
 * This file defines all GPIO pin assignments for the Raspberry Pi Pico.
 * Pin assignments are organized by function (drive train, mechanisms, etc.).
 *
 * @note All 30 GPIOs on the Pico are PWM capable (16 slices max in use).
 *
 * @section pwm_table PWM Slice/Channel Mapping
 *
 * | Slice | Chan A | Chan B | Alt Chan A | Alt Chan B |
 * |-------|--------|--------|------------|------------|
 * | 0     | 0      | 1      | 16         | 17         |
 * | 1     | 2      | 3      | 18         | 19         |
 * | 2     | 4      | 5      | 20         | 21         |
 * | 3     | 6      | 7      | 22         | 23         |
 * | 4     | 8      | 9      | 24         | 25         |
 * | 5     | 10     | 11     | 26         | 27         |
 * | 6     | 12     | 13     | 28         | 29         |
 * | 7     | 14     | 15     | n/a        | n/a        |
 ******************************************************************************/
#pragma once


/* Defines -------------------------------------------------------------------*/

/*******************************************************************************
 * @brief Marker for unassigned/invalid pins
 ******************************************************************************/
#define PIN_INVALID ((int)-1)

/*============================================================================*/
/* GPIO Pin Limits                                                            */
/*============================================================================*/

/*******************************************************************************
 * @brief Minimum valid GPIO pin number
 ******************************************************************************/
#define GPIO_PIN_MIN ((int)0)

/*******************************************************************************
 * @brief Maximum valid GPIO pin number
 ******************************************************************************/
#define GPIO_PIN_MAX ((int)29)


/* FlySky IBus Interface -----------------------------------------------------*/

/*******************************************************************************
 * @brief iBUS UART TX pin (Pico to receiver, not typically used)
 * @note Using UART1
 ******************************************************************************/
constexpr int PIN_IBUS_TX = 4;

/*******************************************************************************
 * @brief iBUS UART RX pin (receiver to Pico)
 * @note Using UART1
 ******************************************************************************/
constexpr int PIN_IBUS_RX = 5;


/* Drive Train - Differential ------------------------------------------------*/

/*******************************************************************************
 * @name Differential Left Motor
 *
 * Pin usage depends on motor driver wiring mode (see config.h):
 * - MODE_2PWM:      DIR_FWD = IN1 (PWM), DIR_REV = IN2 (PWM), ENABLE = unused
 * - MODE_1PWM_2DIR: ENABLE = PWM, DIR_FWD = digital, DIR_REV = digital
 * @{
 ******************************************************************************/
constexpr int PIN_DIFF_MOTOR_LEFT_ENABLE  = 21;  /**< PWM enable (Slice 2 Chan B) */
constexpr int PIN_DIFF_MOTOR_LEFT_DIR_FWD = 27;  /**< Forward direction / IN1 */
constexpr int PIN_DIFF_MOTOR_LEFT_DIR_REV = 26;  /**< Reverse direction / IN2 */
constexpr int PIN_DIFF_MOTOR_LEFT_ENC     = PIN_INVALID; /**< Encoder (Optional) */
/** @} */

/*******************************************************************************
 * @name Differential Right Motor
 *
 * Pin usage depends on motor driver wiring mode (see config.h):
 * - MODE_2PWM:      DIR_FWD = IN1 (PWM), DIR_REV = IN2 (PWM), ENABLE = unused
 * - MODE_1PWM_2DIR: ENABLE = PWM, DIR_FWD = digital, DIR_REV = digital
 * @{
 ******************************************************************************/
constexpr int PIN_DIFF_MOTOR_RIGHT_ENABLE  = 8;   /**< PWM enable (Slice 4 Chan A) */
constexpr int PIN_DIFF_MOTOR_RIGHT_DIR_FWD = 7;   /**< Forward direction / IN1 */
constexpr int PIN_DIFF_MOTOR_RIGHT_DIR_REV = 6;   /**< Reverse direction / IN2 */
constexpr int PIN_DIFF_MOTOR_RIGHT_ENC     = PIN_INVALID; /**< Encoder (Optional) */
/** @} */


/* Collection Mechanism ------------------------------------------------------*/

/*******************************************************************************
 * @name Collection Mechanism Pins
 * @note Assign pins based on hardware design
 * @{
 ******************************************************************************/
constexpr int PIN_COLLECT_MOTOR_PWM = PIN_INVALID;  /**< Collector motor PWM */
/** @} */


/* Deposit Mechanism ---------------------------------------------------------*/

/*******************************************************************************
 * @name Deposit Mechanism Pins
 * @note Assign pins based on hardware design
 * @{
 ******************************************************************************/
constexpr int PIN_DEPOSIT_SERVO     = PIN_INVALID;  /**< Deposit gate servo */
/** @} */


/* Launcher Mechanism --------------------------------------------------------*/

/*******************************************************************************
 * @name Launcher Mechanism Pins
 * @note Assign pins based on hardware design
 * @{
 ******************************************************************************/
constexpr int PIN_LAUNCHER_MOTOR_PWM     = PIN_INVALID;  /**< Launcher motor PWM */
constexpr int PIN_LAUNCHER_FEED_SERVO    = PIN_INVALID;  /**< Ball feed servo */
constexpr int PIN_LAUNCHER_TILT_SERVO    = PIN_INVALID;  /**< Angle adjustment servo */
/** @} */


/* Debug/Status --------------------------------------------------------------*/

/*******************************************************************************
 * @brief Onboard LED pin (built into Pico)
 ******************************************************************************/
constexpr int PIN_LED_ONBOARD = 25;

/*******************************************************************************
 * @brief External status LED (optional)
 ******************************************************************************/
constexpr int PIN_LED_STATUS = PIN_INVALID;


/* EOF -----------------------------------------------------------------------*/

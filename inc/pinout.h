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
 *
 * @section orientation Pico Orientation
 * - Pins GP0 - GP15 facing right
 * - Pins GP16 - GP28 facing left
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


/* Drive Train - Mecanum -----------------------------------------------------*/

/*******************************************************************************
 * @name Mecanum Front Left Motor
 * @{
 ******************************************************************************/
constexpr int PIN_MECANUM_MOTOR_FL_PWM     = 20;  /**< PWM Slice 2 Channel A */
constexpr int PIN_MECANUM_MOTOR_FL_DIR_FWD = 18;  /**< Direction forward */
constexpr int PIN_MECANUM_MOTOR_FL_DIR_REV = 19;  /**< Direction reverse */
/** @} */

/*******************************************************************************
 * @name Mecanum Front Right Motor
 * @{
 ******************************************************************************/
constexpr int PIN_MECANUM_MOTOR_FR_PWM     = 8;   /**< PWM Slice 4 Channel A */
constexpr int PIN_MECANUM_MOTOR_FR_DIR_FWD = 10;  /**< Direction forward */
constexpr int PIN_MECANUM_MOTOR_FR_DIR_REV = 11;  /**< Direction reverse */
/** @} */

/*******************************************************************************
 * @name Mecanum Rear Right Motor
 * @{
 ******************************************************************************/
constexpr int PIN_MECANUM_MOTOR_RR_PWM     = 9;   /**< PWM Slice 4 Channel B */
constexpr int PIN_MECANUM_MOTOR_RR_DIR_FWD = 7;   /**< Direction forward */
constexpr int PIN_MECANUM_MOTOR_RR_DIR_REV = 6;   /**< Direction reverse */
/** @} */

/*******************************************************************************
 * @name Mecanum Rear Left Motor
 * @{
 ******************************************************************************/
constexpr int PIN_MECANUM_MOTOR_RL_PWM     = 21;  /**< PWM Slice 2 Channel B */
constexpr int PIN_MECANUM_MOTOR_RL_DIR_FWD = 27;  /**< Direction forward */
constexpr int PIN_MECANUM_MOTOR_RL_DIR_REV = 26;  /**< Direction reverse */
/** @} */


/* Drive Train - Differential ------------------------------------------------*/

/*******************************************************************************
 * @name Differential Left Motor
 * @{
 ******************************************************************************/
constexpr int PIN_DIFF_MOTOR_LEFT_PWM     = 9;   /**< PWM Slice 4 Channel B */
constexpr int PIN_DIFF_MOTOR_LEFT_DIR_FWD = 7;   /**< Direction forward */
constexpr int PIN_DIFF_MOTOR_LEFT_DIR_REV = 6;   /**< Direction reverse */
constexpr int PIN_DIFF_MOTOR_LEFT_ENC     = PIN_INVALID; /**< Encoder (optional) */
/** @} */

/*******************************************************************************
 * @name Differential Right Motor
 * @{
 ******************************************************************************/
constexpr int PIN_DIFF_MOTOR_RIGHT_PWM     = 8;  /**< PWM Slice 4 Channel A */
constexpr int PIN_DIFF_MOTOR_RIGHT_DIR_FWD = 10; /**< Direction forward */
constexpr int PIN_DIFF_MOTOR_RIGHT_DIR_REV = 11; /**< Direction reverse */
constexpr int PIN_DIFF_MOTOR_RIGHT_ENC     = PIN_INVALID; /**< Encoder (optional) */
/** @} */


/* Collection Mechanism ------------------------------------------------------*/

/*******************************************************************************
 * @name Collection Mechanism Pins
 * @note Assign pins based on hardware design
 * @{
 ******************************************************************************/
constexpr int PIN_COLLECT_MOTOR_PWM = PIN_INVALID;  /**< Collector motor PWM */
constexpr int PIN_COLLECT_MOTOR_DIR = PIN_INVALID;  /**< Collector motor direction */
constexpr int PIN_COLLECT_SENSOR    = PIN_INVALID;  /**< Ball presence sensor */
/** @} */


/* Deposit Mechanism ---------------------------------------------------------*/

/*******************************************************************************
 * @name Deposit Mechanism Pins
 * @note Assign pins based on hardware design
 * @{
 ******************************************************************************/
constexpr int PIN_DEPOSIT_SERVO     = PIN_INVALID;  /**< Deposit gate servo */
constexpr int PIN_DEPOSIT_SENSOR    = PIN_INVALID;  /**< Ball count sensor */
/** @} */


/* Launcher Mechanism --------------------------------------------------------*/

/*******************************************************************************
 * @name Launcher Mechanism Pins
 * @note Assign pins based on hardware design
 * @{
 ******************************************************************************/
constexpr int PIN_LAUNCHER_MOTOR_PWM     = PIN_INVALID;  /**< Launcher motor PWM */
constexpr int PIN_LAUNCHER_MOTOR_DIR_FWD = PIN_INVALID;  /**< Launcher direction fwd */
constexpr int PIN_LAUNCHER_MOTOR_DIR_REV = PIN_INVALID;  /**< Launcher direction rev */
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

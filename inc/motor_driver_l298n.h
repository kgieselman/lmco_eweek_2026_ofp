/*******************************************************************************
 * @file motor_driver_l298n.h
 * @brief L298N dual H-bridge motor driver interface
 *
 * Provides a motor driver implementation for the L298N chip. This driver uses
 * a single PWM signal for speed control and two digital outputs for direction:
 *   - Forward:  EN = PWM duty cycle, IN1 = HIGH, IN2 = LOW
 *   - Reverse:  EN = PWM duty cycle, IN1 = LOW,  IN2 = HIGH
 *   - Brake:    EN = HIGH,           IN1 = HIGH, IN2 = HIGH
 *   - Coast:    EN = LOW,            IN1 = X,    IN2 = X
 *
 * @par L298N Specifications:
 *   - Operating voltage: 5V to 35V
 *   - Output current: 2A per channel (3A peak)
 *   - Logic voltage: 5V (can use 3.3V with level shifter)
 *   - Built-in flyback diodes on most modules
 *
 * @par Example Usage:
 * @code
 * MotorDriverL298N driver;
 * 
 * // Configure motor A with PWM pin 8, direction pins 9 and 10
 * if (!driver.configureMotor(MotorDriverL298N::MOTOR_A, 8, 9, 10)) {
 *   // Handle error
 * }
 * 
 * // Set motor to 75% forward
 * driver.setMotor(MotorDriverL298N::MOTOR_A, 375);  // 375/500 = 75%
 * 
 * // Set motor to 50% reverse
 * driver.setMotor(MotorDriverL298N::MOTOR_A, -250); // -250/500 = 50% reverse
 * 
 * // Stop with braking
 * driver.brake(MotorDriverL298N::MOTOR_A);
 * 
 * // Stop with coasting (free spin)
 * driver.coast(MotorDriverL298N::MOTOR_A);
 * @endcode
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include "pinout.h"

#include <stdint.h>
#include <stdbool.h>


/* Class Definition ----------------------------------------------------------*/

/*******************************************************************************
 * @class MotorDriverL298N
 * @brief Driver for L298N dual H-bridge motor controller
 *
 * This class provides an interface for controlling motors through the L298N
 * motor driver IC. Each L298N chip can control two DC motors independently.
 *
 * The L298N uses a PWM enable pin for speed control and two direction pins
 * to control the H-bridge state.
 ******************************************************************************/
class MotorDriverL298N
{
public:
  /* Public Types ------------------------------------------------------------*/

  /*****************************************************************************
   * @brief Motor channel identifier
   ****************************************************************************/
  enum MotorChannel {
    MOTOR_A = 0,    /**< Motor channel A (ENA/IN1/IN2) */
    MOTOR_B = 1,    /**< Motor channel B (ENB/IN3/IN4) */
    MOTOR_COUNT = 2 /**< Number of motor channels per driver */
  };

  /*****************************************************************************
   * @brief Motor stop mode
   ****************************************************************************/
  enum StopMode {
    STOP_COAST,  /**< Coast/free spin (EN = LOW) */
    STOP_BRAKE   /**< Active brake (EN = HIGH, IN1 = IN2 = HIGH) */
  };


  /* Public Constants --------------------------------------------------------*/

  /** @brief Minimum motor value (full reverse) */
  static constexpr int MOTOR_VALUE_MIN = -500;

  /** @brief Maximum motor value (full forward) */
  static constexpr int MOTOR_VALUE_MAX = 500;

  /** @brief Default PWM frequency in Hz */
  static constexpr int DEFAULT_PWM_FREQ_HZ = 20000;


  /* Public Function Declarations --------------------------------------------*/

  /*****************************************************************************
   * @brief Construct a L298N motor driver instance
   * 
   * @param pwmFreqHz PWM frequency in Hz (default 20kHz)
   ****************************************************************************/
  explicit MotorDriverL298N(int pwmFreqHz = DEFAULT_PWM_FREQ_HZ);

  /*****************************************************************************
   * @brief Destructor - stops all motors
   ****************************************************************************/
  ~MotorDriverL298N();

  /*****************************************************************************
   * @brief Configure a motor channel
   *
   * Initializes the PWM and GPIO outputs for a motor channel.
   *
   * @param channel Motor channel to configure (MOTOR_A or MOTOR_B)
   * @param pinPwm GPIO pin for PWM enable (ENA or ENB)
   * @param pinDirFwd GPIO pin for forward direction (IN1 or IN3)
   * @param pinDirRev GPIO pin for reverse direction (IN2 or IN4)
   * @param pinEncoder Optional encoder input pin
   * @return true if configuration successful, false on error
   *
   * @note If motor runs backwards, swap pinDirFwd and pinDirRev.
   ****************************************************************************/
  bool configureMotor(MotorChannel channel,
                      int pinPwm,
                      int pinDirFwd,
                      int pinDirRev,
                      int pinEncoder = PIN_INVALID);

  /*****************************************************************************
   * @brief Set motor speed and direction
   *
   * Sets the motor output to the specified value. The value is in the range
   * [-500, +500] where:
   *   - +500 = full speed forward
   *   - 0    = stopped (coast mode)
   *   - -500 = full speed reverse
   *
   * @param channel Motor channel to control
   * @param value Motor value in range [-500, +500]
   * @return true if value applied successfully
   *
   * @note Values outside the valid range will be clamped.
   ****************************************************************************/
  bool setMotor(MotorChannel channel, int value);

  /*****************************************************************************
   * @brief Set motor output with trim applied
   *
   * Sets the motor output with an additional trim multiplier for calibration.
   * The trim value scales the output to compensate for motor variations.
   *
   * @param channel Motor channel to control
   * @param value Motor value in range [-500, +500]
   * @param trim Trim multiplier (0.0 to 1.0, where 1.0 = no trim)
   * @return true if value applied successfully
   ****************************************************************************/
  bool setMotorWithTrim(MotorChannel channel, int value, float trim);

  /*****************************************************************************
   * @brief Stop motor with coasting (free spin)
   *
   * Sets PWM output to 0%, allowing the motor to coast to a stop.
   *
   * @param channel Motor channel to stop
   ****************************************************************************/
  void coast(MotorChannel channel);

  /*****************************************************************************
   * @brief Stop motor with active braking
   *
   * Sets PWM to 100% and both direction pins HIGH for active braking.
   *
   * @param channel Motor channel to brake
   ****************************************************************************/
  void brake(MotorChannel channel);

  /*****************************************************************************
   * @brief Stop motor using specified mode
   *
   * @param channel Motor channel to stop
   * @param mode Stop mode (STOP_COAST or STOP_BRAKE)
   ****************************************************************************/
  void stop(MotorChannel channel, StopMode mode = STOP_COAST);

  /*****************************************************************************
   * @brief Stop all motors
   *
   * @param mode Stop mode to use for all motors
   ****************************************************************************/
  void stopAll(StopMode mode = STOP_COAST);

  /*****************************************************************************
   * @brief Check if a motor channel is configured
   *
   * @param channel Motor channel to check
   * @return true if the channel has been configured
   ****************************************************************************/
  bool isConfigured(MotorChannel channel) const;

  /*****************************************************************************
   * @brief Check if all motor channels are configured
   *
   * @return true if all channels have been configured
   ****************************************************************************/
  bool isFullyConfigured() const;

  /*****************************************************************************
   * @brief Get the encoder pin for a motor channel
   *
   * @param channel Motor channel
   * @return Encoder pin number, or -1 if not configured
   ****************************************************************************/
  int getEncoderPin(MotorChannel channel) const;

  /*****************************************************************************
   * @brief Set the default stop mode
   *
   * @param mode Default stop mode for setMotor(0) calls
   ****************************************************************************/
  void setDefaultStopMode(StopMode mode) { m_defaultStopMode = mode; }

  /*****************************************************************************
   * @brief Get the current default stop mode
   *
   * @return Current default stop mode
   ****************************************************************************/
  StopMode getDefaultStopMode() const { return m_defaultStopMode; }


private:
  /* Private Types -----------------------------------------------------------*/

  /*****************************************************************************
   * @brief Motor channel configuration
   ****************************************************************************/
  struct MotorConfig {
    bool configured;  /**< Channel is configured and ready */
    int pinPwm;       /**< PWM enable pin */
    int pinDirFwd;    /**< Forward direction pin */
    int pinDirRev;    /**< Reverse direction pin */
    int pinEncoder;   /**< Encoder pin */
    int currentValue; /**< Current motor value for state tracking */
  };


  /* Private Constants -------------------------------------------------------*/

  /** @brief PWM counter top value (determines resolution) */
  static constexpr int PWM_TOP_COUNT = 1000;


  /* Private Variables -------------------------------------------------------*/
  MotorConfig m_motors[MOTOR_COUNT]; /**< Motor channel configurations */
  int m_pwmFreqHz;                   /**< Configured PWM frequency */
  float m_pwmClkDiv;                 /**< Calculated PWM clock divider */
  StopMode m_defaultStopMode;        /**< Default stop mode */


  /* Private Function Declarations -------------------------------------------*/

  /*****************************************************************************
   * @brief Validate GPIO pin number
   *
   * @param pin Pin number to validate
   * @return true if pin is in valid range
   ****************************************************************************/
  bool validatePin(int pin) const;

  /*****************************************************************************
   * @brief Initialize a GPIO pin for PWM output
   *
   * @param pin Pin number to configure
   * @return true if initialization successful
   ****************************************************************************/
  bool initPwmPin(int pin);

  /*****************************************************************************
   * @brief Initialize a GPIO pin for digital output
   *
   * @param pin Pin number to configure
   * @return true if initialization successful
   ****************************************************************************/
  bool initDirectionPin(int pin);

  /*****************************************************************************
   * @brief Set PWM duty cycle on a pin
   *
   * @param pin PWM pin
   * @param dutyCycle Duty cycle (0 to PWM_TOP_COUNT)
   ****************************************************************************/
  void setPwmDuty(int pin, uint16_t dutyCycle);

  /*****************************************************************************
   * @brief Set a digital output pin state
   *
   * @param pin GPIO pin
   * @param state Pin state (true = HIGH, false = LOW)
   ****************************************************************************/
  void setDigitalOut(int pin, bool state);

  /*****************************************************************************
   * @brief Calculate clock divider for desired PWM frequency
   *
   * @param freqHz Desired PWM frequency in Hz
   * @return Clock divider value
   ****************************************************************************/
  float calculateClockDivider(int freqHz) const;

  /*****************************************************************************
   * @brief Clamp a value to the valid motor range
   *
   * @param value Value to clamp
   * @return Clamped value in range [MOTOR_VALUE_MIN, MOTOR_VALUE_MAX]
   ****************************************************************************/
  static constexpr int clampMotorValue(int value)
  {
    return (value < MOTOR_VALUE_MIN) ? MOTOR_VALUE_MIN :
           (value > MOTOR_VALUE_MAX) ? MOTOR_VALUE_MAX : value;
  }
};


/* EOF -----------------------------------------------------------------------*/

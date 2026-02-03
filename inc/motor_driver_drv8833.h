/*******************************************************************************
 * @file motor_driver_drv8833.h
 * @brief DRV8833 dual H-bridge motor driver interface
 *
 * Provides a motor driver implementation for the TI DRV8833 chip. Unlike
 * traditional motor drivers that use a single PWM + direction pins, the
 * DRV8833 uses two PWM signals per motor channel (IN1/IN2) where:
 *   - Forward:  IN1 = PWM duty cycle, IN2 = LOW (0% duty)
 *   - Reverse:  IN1 = LOW (0% duty),  IN2 = PWM duty cycle
 *   - Brake:    IN1 = HIGH,           IN2 = HIGH
 *   - Coast:    IN1 = LOW,            IN2 = LOW
 *
 * @par DRV8833 Specifications:
 *   - Operating voltage: 2.7V to 10.8V
 *   - Output current: 1.5A per channel (2A peak)
 *   - PWM frequency: Up to 50 kHz recommended
 *   - Built-in current limiting and thermal shutdown
 *
 * @par Example Usage:
 * @code
 * MotorDriverDRV8833 driver;
 * 
 * // Configure motor A with PWM pins 8 and 9
 * if (!driver.configureMotor(MotorDriverDRV8833::MOTOR_A, 8, 9)) {
 *   // Handle error
 * }
 * 
 * // Set motor to 75% forward
 * driver.setMotor(MotorDriverDRV8833::MOTOR_A, 375);  // 375/500 = 75%
 * 
 * // Set motor to 50% reverse
 * driver.setMotor(MotorDriverDRV8833::MOTOR_A, -250); // -250/500 = 50% reverse
 * 
 * // Stop with braking
 * driver.brake(MotorDriverDRV8833::MOTOR_A);
 * 
 * // Stop with coasting (free spin)
 * driver.coast(MotorDriverDRV8833::MOTOR_A);
 * @endcode
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include "pinout.h"

#include <stdint.h>
#include <stdbool.h>


/* Class Definition ----------------------------------------------------------*/

/*******************************************************************************
 * @class MotorDriverDRV8833
 * @brief Driver for DRV8833 dual H-bridge motor controller
 *
 * This class provides an interface for controlling motors through the DRV8833
 * motor driver IC. Each DRV8833 chip can control two DC motors independently.
 *
 * The DRV8833 differs from traditional motor drivers in that it uses two PWM
 * inputs per channel instead of a PWM + direction approach. This allows for
 * more flexible control including active braking.
 ******************************************************************************/
class MotorDriverDRV8833
{
public:
  /* Public Types ------------------------------------------------------------*/

  /*****************************************************************************
   * @brief Motor channel identifier
   ****************************************************************************/
  enum MotorChannel {
    MOTOR_A = 0,    /**< Motor channel A (AIN1/AIN2) */
    MOTOR_B = 1,    /**< Motor channel B (BIN1/BIN2) */
    MOTOR_COUNT = 2 /**< Number of motor channels per driver */
  };

  /*****************************************************************************
   * @brief Motor stop mode
   ****************************************************************************/
  enum StopMode {
    STOP_COAST,  /**< Coast/free spin (both outputs LOW) */
    STOP_BRAKE   /**< Active brake (both outputs HIGH) */
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
   * @brief Construct a DRV8833 motor driver instance
   * 
   * @param pwmFreqHz PWM frequency in Hz (default 20kHz)
   ****************************************************************************/
  explicit MotorDriverDRV8833(int pwmFreqHz = DEFAULT_PWM_FREQ_HZ);

  /*****************************************************************************
   * @brief Destructor - stops all motors
   ****************************************************************************/
  ~MotorDriverDRV8833();

  /*****************************************************************************
   * @brief Configure a motor channel
   *
   * Initializes the PWM outputs for a motor channel. Both pins must be valid
   * GPIO pins capable of PWM output.
   *
   * @param channel Motor channel to configure (MOTOR_A or MOTOR_B)
   * @param pinIn1 GPIO pin for IN1 (forward PWM)
   * @param pinIn2 GPIO pin for IN2 (reverse PWM)
   * @param pinEncoder Optional encoder input pin
   * @return true if configuration successful, false on error
   *
   * @note For best results, use pins on the same PWM slice for synchronized
   *       output. See pinout.h for PWM slice mapping.
   * @note If motor runs backwards, swap pinIn1 and pinIn2.
   ****************************************************************************/
  [[nodiscard]] bool configureMotor(MotorChannel channel,
                                    int pinIn1,
                                    int pinIn2,
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
  [[nodiscard]] bool setMotor(MotorChannel channel, int value);

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
  [[nodiscard]] bool setMotorWithTrim(MotorChannel channel, int value, float trim);

  /*****************************************************************************
   * @brief Stop motor with coasting (free spin)
   *
   * Sets both PWM outputs to 0%, allowing the motor to coast to a stop.
   *
   * @param channel Motor channel to stop
   ****************************************************************************/
  void coast(MotorChannel channel);

  /*****************************************************************************
   * @brief Stop motor with active braking
   *
   * Sets both PWM outputs to 100%, actively braking the motor.
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
  [[nodiscard]] bool isConfigured(MotorChannel channel) const;

  /*****************************************************************************
   * @brief Check if all motor channels are configured
   *
   * @return true if all channels have been configured
   ****************************************************************************/
  [[nodiscard]] bool isFullyConfigured() const;

  /*****************************************************************************
   * @brief Get the encoder pin for a motor channel
   *
   * @param channel Motor channel
   * @return Encoder pin number, or -1 if not configured
   ****************************************************************************/
  [[nodiscard]] int getEncoderPin(MotorChannel channel) const;

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
  [[nodiscard]] StopMode getDefaultStopMode() const { return m_defaultStopMode; }


private:
  /* Private Types -----------------------------------------------------------*/

  /*****************************************************************************
   * @brief Motor channel configuration
   ****************************************************************************/
  struct MotorConfig {
    bool configured;  /**< Channel is configured and ready */
    int pinIn1;       /**< IN1 (forward) PWM pin */
    int pinIn2;       /**< IN2 (reverse) PWM pin */
    int pinEncoder;   /**< Encoder pin (Invalid if not used) */
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
  [[nodiscard]] bool validatePin(int pin) const;

  /*****************************************************************************
   * @brief Initialize a GPIO pin for PWM output
   *
   * @param pin Pin number to configure
   * @return true if initialization successful
   ****************************************************************************/
  [[nodiscard]] bool initPwmPin(int pin);

  /*****************************************************************************
   * @brief Set PWM duty cycle on a pin
   *
   * @param pin PWM pin
   * @param dutyCycle Duty cycle (0 to PWM_TOP_COUNT)
   ****************************************************************************/
  void setPwmDuty(int pin, uint16_t dutyCycle);

  /*****************************************************************************
   * @brief Calculate clock divider for desired PWM frequency
   *
   * @param freqHz Desired PWM frequency in Hz
   * @return Clock divider value
   ****************************************************************************/
  [[nodiscard]] float calculateClockDivider(int freqHz) const;

  /*****************************************************************************
   * @brief Clamp a value to the valid motor range
   *
   * @param value Value to clamp
   * @return Clamped value in range [MOTOR_VALUE_MIN, MOTOR_VALUE_MAX]
   ****************************************************************************/
  [[nodiscard]] static constexpr int clampMotorValue(int value)
  {
    return (value < MOTOR_VALUE_MIN) ? MOTOR_VALUE_MIN :
           (value > MOTOR_VALUE_MAX) ? MOTOR_VALUE_MAX : value;
  }
};


/* EOF -----------------------------------------------------------------------*/

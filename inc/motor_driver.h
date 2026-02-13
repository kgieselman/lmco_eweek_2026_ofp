/*******************************************************************************
 * @file motor_driver.h
 * @brief Generic dual H-bridge motor driver interface
 *
 * Provides a single motor driver implementation that supports two common
 * wiring configurations, selected at construction time:
 *
 * @par MODE_2PWM (e.g. DRV8833, TB6612FNG):
 * Two PWM signals per motor channel (IN1/IN2):
 *   - Forward:  IN1 = PWM duty cycle, IN2 = LOW (0% duty)
 *   - Reverse:  IN1 = LOW (0% duty),  IN2 = PWM duty cycle
 *   - Brake:    IN1 = HIGH,           IN2 = HIGH
 *   - Coast:    IN1 = LOW,            IN2 = LOW
 *
 * @par MODE_1PWM_2DIR (e.g. L298N, BTS7960):
 * One PWM signal for speed, two digital pins for direction:
 *   - Forward:  EN = PWM duty cycle, IN1 = HIGH, IN2 = LOW
 *   - Reverse:  EN = PWM duty cycle, IN1 = LOW,  IN2 = HIGH
 *   - Brake:    EN = HIGH,           IN1 = HIGH, IN2 = HIGH
 *   - Coast:    EN = LOW,            IN1 = LOW,  IN2 = LOW
 *
 * @par Example Usage (2-PWM mode):
 * @code
 * MotorDriver driver(MotorDriver::MODE_2PWM);
 * driver.configureMotor(MotorDriver::MOTOR_A, 8, 9);
 * driver.setMotor(MotorDriver::MOTOR_A, 375);  // 75% forward
 * @endcode
 *
 * @par Example Usage (1-PWM + 2-DIR mode):
 * @code
 * MotorDriver driver(MotorDriver::MODE_1PWM_2DIR);
 * driver.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10);
 * driver.setMotor(MotorDriver::MOTOR_A, -250); // 50% reverse
 * @endcode
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include "pinout.h"

#include <stdint.h>
#include <stdbool.h>


/* Class Definition ----------------------------------------------------------*/

/*******************************************************************************
 * @class MotorDriver
 * @brief Generic driver for dual H-bridge motor controllers
 *
 * This class provides a unified interface for controlling motors through
 * various H-bridge motor driver ICs. The wiring mode is selected at
 * construction time and applies to all channels on the driver instance.
 ******************************************************************************/
class MotorDriver
{
public:
  /* Public Types ------------------------------------------------------------*/

  /*****************************************************************************
   * @brief Wiring mode enumeration
   *
   * Determines how the motor driver IC is wired to the microcontroller.
   ****************************************************************************/
  enum Mode_e {
    MODE_2PWM,       /**< Two PWM pins per channel (e.g. DRV8833) */
    MODE_1PWM_2DIR   /**< One PWM + two direction pins per channel (e.g. L298N) */
  };

  /*****************************************************************************
   * @brief Motor channel identifier
   ****************************************************************************/
  enum MotorChannel_e {
    MOTOR_A     = 0, /**< Motor channel A */
    MOTOR_B     = 1, /**< Motor channel B */
    MOTOR_COUNT = 2  /**< Number of motor channels per driver */
  };

  /*****************************************************************************
   * @brief Motor stop mode
   ****************************************************************************/
  enum StopMode_e {
    STOP_COAST, /**< Coast/free spin (outputs floating/low) */
    STOP_BRAKE  /**< Active brake (outputs shorted/high) */
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
   * @brief Construct a motor driver instance
   *
   * @param mode Wiring mode for all channels on this driver
   * @param pwmFreqHz PWM frequency in Hz (default 20kHz)
   ****************************************************************************/
  explicit MotorDriver(Mode_e mode = MODE_2PWM,
                       int    pwmFreqHz = DEFAULT_PWM_FREQ_HZ);

  /*****************************************************************************
   * @brief Destructor - stops all motors
   ****************************************************************************/
  ~MotorDriver();

  /*****************************************************************************
   * @brief Configure a motor channel (2-PWM mode)
   *
   * Initializes the PWM outputs for a motor channel. Both pins must be valid
   * GPIO pins capable of PWM output.
   *
   * @param channel Motor channel to configure (MOTOR_A or MOTOR_B)
   * @param pinIn1 PWM pin for forward direction
   * @param pinIn2 PWM pin for reverse direction
   * @param pinEncoder Optional encoder input pin
   * @return true if configuration successful, false on error
   *
   * @note Only valid when driver is constructed with MODE_2PWM.
   * @note If motor runs backwards, swap pinIn1 and pinIn2.
   ****************************************************************************/
  bool configureMotor(MotorChannel_e channel,
                      int            pinIn1,
                      int            pinIn2,
                      int            pinEncoder = PIN_INVALID);

  /*****************************************************************************
   * @brief Configure a motor channel (1-PWM + 2-DIR mode)
   *
   * Initializes the PWM and GPIO outputs for a motor channel.
   *
   * @param channel Motor channel to configure (MOTOR_A or MOTOR_B)
   * @param pinPwm GPIO pin for PWM speed control
   * @param pinDirFwd GPIO pin for forward direction
   * @param pinDirRev GPIO pin for reverse direction
   * @param pinEncoder Optional encoder input pin
   * @return true if configuration successful, false on error
   *
   * @note Only valid when driver is constructed with MODE_1PWM_2DIR.
   * @note If motor runs backwards, swap pinDirFwd and pinDirRev.
   ****************************************************************************/
  bool configureMotor(MotorChannel_e channel,
                      int            pinPwm,
                      int            pinDirFwd,
                      int            pinDirRev,
                      int            pinEncoder);

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
  bool setMotor(MotorChannel_e channel, int value);

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
  bool setMotorWithTrim(MotorChannel_e channel, int value, float trim);

  /*****************************************************************************
   * @brief Stop motor with coasting (free spin)
   *
   * @param channel Motor channel to stop
   ****************************************************************************/
  void coast(MotorChannel_e channel);

  /*****************************************************************************
   * @brief Stop motor with active braking
   *
   * @param channel Motor channel to brake
   ****************************************************************************/
  void brake(MotorChannel_e channel);

  /*****************************************************************************
   * @brief Stop motor using specified mode
   *
   * @param channel Motor channel to stop
   * @param mode Stop mode (STOP_COAST or STOP_BRAKE)
   ****************************************************************************/
  void stop(MotorChannel_e channel, StopMode_e mode = STOP_COAST);

  /*****************************************************************************
   * @brief Stop all motors
   *
   * @param mode Stop mode to use for all motors
   ****************************************************************************/
  void stopAll(StopMode_e mode = STOP_COAST);

  /*****************************************************************************
   * @brief Check if a motor channel is configured
   *
   * @param channel Motor channel to check
   * @return true if the channel has been configured
   ****************************************************************************/
  bool isConfigured(MotorChannel_e channel) const;

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
   * @return Encoder pin number, or PIN_INVALID if not configured
   ****************************************************************************/
  int getEncoderPin(MotorChannel_e channel) const;

  /*****************************************************************************
   * @brief Get the wiring mode
   *
   * @return Wiring mode set at construction
   ****************************************************************************/
  Mode_e getMode() const { return m_mode; }

  /*****************************************************************************
   * @brief Set the default stop mode
   *
   * @param mode Default stop mode for setMotor(0) calls
   ****************************************************************************/
  void setDefaultStopMode(StopMode_e mode) { m_defaultStopMode = mode; }

  /*****************************************************************************
   * @brief Get the current default stop mode
   *
   * @return Current default stop mode
   ****************************************************************************/
  StopMode_e getDefaultStopMode() const { return m_defaultStopMode; }

  /*****************************************************************************
   * @brief Get the current motor setpoint value (before trim)
   *
   * Returns the last value passed to setMotor()/setMotorWithTrim() for the
   * specified channel. This is the pre-trim value in [-500, +500].
   *
   * @param channel Motor channel to query
   * @return Current motor value, or 0 if channel is invalid
   ****************************************************************************/
  int getCurrentValue(MotorChannel_e channel) const;


private:
  /* Private Types -----------------------------------------------------------*/

  /*****************************************************************************
   * @brief Motor channel configuration
   *
   * Pin usage depends on the wiring mode:
   * - MODE_2PWM:      pin1 = IN1 (fwd PWM), pin2 = IN2 (rev PWM), pin3 = unused
   * - MODE_1PWM_2DIR: pin1 = PWM enable,    pin2 = DIR fwd,       pin3 = DIR rev
   ****************************************************************************/
  struct MotorConfig {
    bool configured;  /**< Channel is configured and ready */
    int pin1;         /**< Primary pin (IN1 or PWM) */
    int pin2;         /**< Secondary pin (IN2 or DIR_FWD) */
    int pin3;         /**< Tertiary pin (unused or DIR_REV) */
    int pinEncoder;   /**< Encoder pin (PIN_INVALID if not used) */
    int currentValue; /**< Current motor value for state tracking */
  };


  /* Private Constants -------------------------------------------------------*/

  /** @brief PWM counter top value (determines resolution) */
  static constexpr int PWM_TOP_COUNT = 1000;


  /* Private Variables -------------------------------------------------------*/

  Mode_e     m_mode;                     /**< Wiring mode for all channels */
  MotorConfig m_motors[MOTOR_COUNT];     /**< Motor channel configurations */
  int         m_pwmFreqHz;               /**< Configured PWM frequency */
  float       m_pwmClkDiv;               /**< Calculated PWM clock divider */
  StopMode_e  m_defaultStopMode;         /**< Default stop mode */


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
   * @brief Apply motor output in 2-PWM mode
   *
   * @param motor Motor configuration
   * @param dutyCycle Calculated duty cycle
   * @param forward true if forward direction
   ****************************************************************************/
  void applyOutput2Pwm(const MotorConfig& motor, uint16_t dutyCycle, bool forward);

  /*****************************************************************************
   * @brief Apply motor output in 1-PWM + 2-DIR mode
   *
   * @param motor Motor configuration
   * @param dutyCycle Calculated duty cycle
   * @param forward true if forward direction
   ****************************************************************************/
  void applyOutput1Pwm2Dir(const MotorConfig& motor, uint16_t dutyCycle, bool forward);

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

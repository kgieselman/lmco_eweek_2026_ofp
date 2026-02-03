/*******************************************************************************
 * @file drive_train.cpp
 * @brief Implementation of base drive train class
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "drive_train.h"
#include "error_handler.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"


/* Method Definitions --------------------------------------------------------*/
DriveTrain::DriveTrain()
  : m_speed(0)
  , m_turn(0)
{
}

DriveTrain::~DriveTrain()
{
  /* Base destructor - derived classes handle motor cleanup */
}

bool DriveTrain::setSpeed(int speed)
{
  if (!validateUserInput(speed))
  {
    ERROR_REPORT(ERROR_OUT_OF_RANGE);
    return false;
  }

  m_speed = speed;
  return true;
}

bool DriveTrain::setTurn(int turn)
{
  if (!validateUserInput(turn))
  {
    ERROR_REPORT(ERROR_OUT_OF_RANGE);
    return false;
  }

  m_turn = turn;
  return true;
}

bool DriveTrain::validateUserInput(int value) const
{
  return (value >= USER_INPUT_MIN) && (value <= USER_INPUT_MAX);
}

bool DriveTrain::validatePin(int pin) const
{
  return (pin >= GPIO_PIN_MIN) && (pin <= GPIO_PIN_MAX);
}

bool DriveTrain::initPwmPin(int pinPwm, int topCount, float clkDiv)
{
  if (!validatePin(pinPwm))
  {
    ERROR_REPORT(ERROR_DT_INVALID_PIN);
    return false;
  }

  /* Configure pin for PWM function */
  gpio_set_function(pinPwm, GPIO_FUNC_PWM);

  /* Get PWM slice and channel for this pin */
  uint sliceNum = pwm_gpio_to_slice_num(pinPwm);
  uint chanNum  = pwm_gpio_to_channel(pinPwm);

  /* Configure PWM with specified parameters */
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, clkDiv);
  pwm_init(sliceNum, &config, true);
  pwm_set_wrap(sliceNum, topCount);
  pwm_set_chan_level(sliceNum, chanNum, 0);  /* Start with output off */
  pwm_set_enabled(sliceNum, true);

  return true;
}

bool DriveTrain::initDirectionPin(int pinDir)
{
  if (!validatePin(pinDir))
  {
    ERROR_REPORT(ERROR_DT_INVALID_PIN);
    return false;
  }

  /* Configure pin as digital output */
  gpio_init(pinDir);
  gpio_set_dir(pinDir, GPIO_OUT);
  gpio_put(pinDir, 0);  /* Start with output low */

  return true;
}

void DriveTrain::setMotorOutput(const Motor& motor, int value, float multiplier)
{
  if (!motor.initialized)
  {
    return;
  }

  /* Calculate PWM value */
  int absValue = (value < 0) ? -value : value;
  uint16_t pwmValue = static_cast<uint16_t>(absValue * multiplier);

  /* Set PWM level */
  pwm_set_gpio_level(motor.pinPwm, pwmValue);

  /* Set direction pins */
  gpio_put(motor.pinDirFwd, value > 0);
  gpio_put(motor.pinDirRev, value < 0);
}

/* EOF -----------------------------------------------------------------------*/

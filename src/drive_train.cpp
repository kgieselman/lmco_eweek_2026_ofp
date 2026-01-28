/******************************************************************************
 * @file drive_train.cpp
 * @brief Implementation of base class for drive train controllers
 *****************************************************************************/

/* Includes -----------------------------------------------------------------*/
#include "drive_train.h"
#include "hardware/pwm.h"


/* Function Definitions -----------------------------------------------------*/

drive_train::drive_train() : m_speed(0), m_turn(0)
{
  // Initialize member variables
}

drive_train::~drive_train()
{
  // Base destructor - derived classes will handle their own cleanup
}

bool drive_train::set_speed(int speed)
{
  if (!validate_user_input(speed))
  {
    return false;
  }

  m_speed = speed;
  return true;
}

bool drive_train::set_turn(int turn)
{
  if (!validate_user_input(turn))
  {
    return false;
  }

  m_turn = turn;
  return true;
}

bool drive_train::validate_user_input(int value)
{
  return (value >= USER_INPUT_MIN) && (value <= USER_INPUT_MAX);
}

bool drive_train::validate_pin(int pin)
{
  return (pin >= PICO_GPIO_PIN_MIN) && (pin <= PICO_GPIO_PIN_MAX);
}

bool drive_train::init_pwm_pin(int pinPWM, int pwmTopCount, float pwmClkDiv)
{
  if (!validate_pin(pinPWM))
  {
    return false;
  }

  // Configure the pin for PWM function
  gpio_set_function(pinPWM, GPIO_FUNC_PWM);
  
  // Get PWM slice and channel for this pin
  uint slice_num = pwm_gpio_to_slice_num(pinPWM);
  uint chan_num  = pwm_gpio_to_channel(pinPWM);
  
  // Configure PWM
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, pwmClkDiv);
  pwm_init(slice_num, &config, true);
  pwm_set_wrap(slice_num, pwmTopCount);
  pwm_set_chan_level(slice_num, chan_num, 0); // Default to OFF
  pwm_set_enabled(slice_num, true);

  return true;
}

bool drive_train::init_direction_pin(int pinDir)
{
  if (!validate_pin(pinDir))
  {
    return false;
  }

  // Configure direction pin as output
  gpio_init(pinDir);
  gpio_set_dir(pinDir, GPIO_OUT);
  gpio_put(pinDir, 0); // Initialize to OFF

  return true;
}


/* EOF ----------------------------------------------------------------------*/

/******************************************************************************
 * @file drive_train_differential.cpp
 * @brief Implementation of a differential drive train
 *****************************************************************************/

/* Includes -----------------------------------------------------------------*/
#include "drive_train_differential.h"
#include "hardware/pwm.h"
#include <algorithm>
#include <stdio.h>


/* Function Definitions -----------------------------------------------------*/
drive_train_differential::drive_train_differential() : m_debugUpdate(0)
{
// Clear motors
  for (int i=0; i<MOTOR_COUNT; i++)
  {
    m_motors[i] = {0};
    m_motors[i].valTrimFwd = 1.0;
    m_motors[i].valTrimRev = 1.0;
  }
}

drive_train_differential::~drive_train_differential()
{

}

bool drive_train_differential::add_motor(motor_e motor,
                                         int     pinPWM,
                                         int     pinDirFwd,
                                         int     pinDirRev)
{
  if ((motor < MOTOR_LEFT) || (motor >= MOTOR_COUNT))
  {
    // Invalid motor
    return false;
  }

  const int PICO_GPIO_PIN_MIN = 0;
  const int PICO_GPIO_PIN_MAX = 29;
  if ((pinPWM    < PICO_GPIO_PIN_MIN) || (pinPWM    > PICO_GPIO_PIN_MAX) ||
      (pinDirFwd < PICO_GPIO_PIN_MIN) || (pinDirFwd > PICO_GPIO_PIN_MAX) ||
      (pinDirRev < PICO_GPIO_PIN_MIN) || (pinDirRev > PICO_GPIO_PIN_MAX))
  {
    // Invalid pin
    return false;
  }

  m_motors[motor].pinPWM     = pinPWM;
  m_motors[motor].pinDirFwd  = pinDirFwd;
  m_motors[motor].pinDirRev  = pinDirRev;

  // Configure the pins
  gpio_set_function(pinPWM, GPIO_FUNC_PWM);
  uint slice_num    = pwm_gpio_to_slice_num(pinPWM);
  uint chan_num     = pwm_gpio_to_channel(pinPWM);
  pwm_config config = pwm_get_default_config();

  pwm_config_set_clkdiv(&config, PWM_SYS_CLK_DIV);
  pwm_init(slice_num, &config, true);
  pwm_set_wrap(slice_num, PWM_TOP_COUNT);
  pwm_set_chan_level(slice_num, chan_num, 0); // Default to OFF
  pwm_set_enabled(slice_num, true);


  // Configure Discrete Direction Pins (Outputs)
  gpio_init(pinDirFwd);
  gpio_set_dir(pinDirFwd, GPIO_OUT);
  gpio_put(pinDirFwd, 0); // Init to OFF

  gpio_init(pinDirRev);
  gpio_set_dir(pinDirRev, GPIO_OUT);
  gpio_put(pinDirRev, 0); // Init to OFF


  // Since motor was just added, remove any trims
  m_motors[motor].valTrimFwd = 1.0;
  m_motors[motor].valTrimRev = 1.0;

  m_motors[motor].initialized = true;

  return true;
}

bool drive_train_differential::set_speed(int speed)
{
  if ((speed < USER_INPUT_MIN) || (speed > USER_INPUT_MAX))
  {
    return false;
  }

  m_speed = speed;

  return true;
}

bool drive_train_differential::set_turn(int turn)
{
  if ((turn < USER_INPUT_MIN) || (turn > USER_INPUT_MAX))
  {
    return false;
  }

  m_turn = turn;

  return true;
}

void drive_train_differential::update(void)
{
  // Calculate raw Mecanum values (expected range [-1500..1500])
  int calcVal[MOTOR_COUNT  ] = {0};
  calcVal[MOTOR_LEFT ] = m_speed +  m_turn;
  calcVal[MOTOR_RIGHT] = m_speed - m_turn;

  // Apply Trim as needed (default value is 1.0)
  for (int i=0; i<MOTOR_COUNT; i++)
  {
    if (calcVal[i] > 0)
    {
      float trimmedVal = static_cast<float>(calcVal[i]) * m_motors[i].valTrimFwd;
      calcVal[i] = static_cast<int>(trimmedVal);
    }
    else if (calcVal)
    {
      float trimmedVal = static_cast<float>(calcVal[i]) * m_motors[i].valTrimRev;
      calcVal[i] = static_cast<int>(trimmedVal);
    }
  }

  if (m_debugUpdate)
  {
    --m_debugUpdate;
    printf("calcVal[0] %d, calcVCal[1] %d\n\n", calcVal[0], calcVal[1]);
  }

  // Determine Scaling
  // 1. Find the strongest intended user input to determine the overall "throttle" percentage.
  // 2. Find the highest calculated wheel value to use as a normalization base.
  // 3. Create a multiplier that scales the wheels so that the fastest motor matches the 
  //    user's intended throttle, preventing clipping while maintaining the drive vector.
  int inputMax = std::max({std::abs(m_speed), std::abs(m_turn)}); // [0..500]

  int calcMax  = std::max({std::abs(calcVal[MOTOR_LEFT ]), 
                           std::abs(calcVal[MOTOR_RIGHT])});

  float inputMaxPercent = static_cast<float>(inputMax) / USER_INPUT_MAX;

  float motorMultiplier = 0.0;
  if (calcMax > 0) // Avoid divide by 0
  {
    motorMultiplier = (inputMaxPercent * PWM_TOP_COUNT) / static_cast<float>(calcMax);
  }

  for (int i=0; i<MOTOR_COUNT; i++)
  {
    // Update speed
    uint16_t pwmValue = std::abs(calcVal[i]) * motorMultiplier;

    pwm_set_gpio_level(m_motors[i].pinPWM, pwmValue);
    gpio_put(m_motors[i].pinDirFwd, calcVal[i] > 0);
    gpio_put(m_motors[i].pinDirRev, calcVal[i] < 0);
  }
}

void drive_train_differential::print_update(void)
{
  ++m_debugUpdate;
}

// Value will be used as a multiplier to weaken the stronger motor so that
  // they both are normalized. Without this, the car will not drive straight
// TODO: How to write this? Polling? Enable interrupts on encoder and take
  // a nap? See how many interrupts occured during nap?
void drive_train_differential::calibrate(void)
{
  for (int i=0; i<MOTOR_COUNT; i++)
  {
    m_motors[i].valTrimFwd = 1.0;
    m_motors[i].valTrimRev = 1.0;
  }
}


/* EOF ----------------------------------------------------------------------*/

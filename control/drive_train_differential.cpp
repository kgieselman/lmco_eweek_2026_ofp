/******************************************************************************
 * @file drive_train_differential.cpp
 * @brief Implementation of a differential drive train
 *****************************************************************************/

/* Libraries ----------------------------------------------------------------*/
#include "drive_train_differential.h"
#include "hardware/pwm.h"
#include <algorithm>


/* Function Definitions -----------------------------------------------------*/
drive_train_differential::drive_train_differential()
{
// Clear motors
  for (int i=0; i<MOTOR_COUNT; i++)
  {
    motorArr[i] = {0};
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

  const int GPIO_PIN_MIN = 0;
  const int GPIO_PIN_MAX = 29;
  if ((pinPWM    < GPIO_PIN_MIN) || (pinPWM    > GPIO_PIN_MAX) ||
      (pinDirFwd < GPIO_PIN_MIN) || (pinDirFwd > GPIO_PIN_MAX) ||
      (pinDirRev < GPIO_PIN_MIN) || (pinDirRev > GPIO_PIN_MAX))
  {
    // Invalid pin
    return false;
  }

  motorArr[motor].pinPWM     = pinPWM;
  motorArr[motor].pinDirFwd  = pinDirFwd;
  motorArr[motor].pinDirRev  = pinDirRev;

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
  motorArr[motor].valTrimFwd = 0;
  motorArr[motor].valTrimRev = 0;

  motorArr[motor].initialized = true;

  return true;
}

bool drive_train_differential::set_speed(int speed)
{
  if ((speed < USER_INPUT_MIN) || (speed > USER_INPUT_MAX))
  {
    return false;
  }

  inputSpeed = speed;

  return true;
}

bool drive_train_differential::set_turn(int turn)
{
  if ((turn < USER_INPUT_MIN) || (turn > USER_INPUT_MAX))
  {
    return false;
  }

  inputTurn = turn;

  return true;
}

void drive_train_differential::update(void)
{
  // Calculate raw Mecanum values (expected range [-1500..1500])
  int mecanumVal[MOTOR_COUNT  ] = {0};
  mecanumVal[MOTOR_LEFT ] = inputSpeed +  inputTurn;
  mecanumVal[MOTOR_RIGHT] = inputSpeed - inputTurn;

  // Determine Scaling
  // 1. Find the strongest intended user input to determine the overall "throttle" percentage.
  // 2. Find the highest calculated wheel value to use as a normalization base.
  // 3. Create a multiplier that scales the wheels so that the fastest motor matches the 
  //    user's intended throttle, preventing clipping while maintaining the drive vector.
  int inputMax = std::max({std::abs(inputSpeed), std::abs(inputTurn)}); // [0..500]

  int calcMax  = std::max({std::abs(mecanumVal[MOTOR_LEFT ]), 
                           std::abs(mecanumVal[MOTOR_RIGHT])});

  float inputMaxPercent = static_cast<float>(inputMax) / USER_INPUT_MAX;

  float motorMultiplier = 0.0;
  if (calcMax > 0) // Avoid divide by 0
  {
    motorMultiplier = (inputMaxPercent * PWM_TOP_COUNT) / static_cast<float>(calcMax);
  }

  for (int i=0; i<MOTOR_COUNT; i++)
  {
    // Update speed
    uint16_t pwmValue = std::abs(mecanumVal[i]) * motorMultiplier;

    pwm_set_gpio_level(motorArr[i].pinPWM, pwmValue);
    gpio_put(motorArr[i].pinDirFwd, mecanumVal[i] > 0);
    gpio_put(motorArr[i].pinDirRev, mecanumVal[i] < 0);
  }
}

void drive_train_differential::calibrate(void)
{
  for (int i=0; i<MOTOR_COUNT; i++)
  {
    motorArr[i].valTrimFwd = 0;
    motorArr[i].valTrimRev = 0;
  }
}


/* EOF ----------------------------------------------------------------------*/

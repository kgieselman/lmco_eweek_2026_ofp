/******************************************************************************
 * @file control_drive_train.cpp
 * @brief Implementation of drive train control
 *****************************************************************************/

/** Includes ----------------------------------------------------------------*/
#include "control_drive_train.h"
#include "hardware/pwm.h"
#include <algorithm>
#include <stdio.h>


/** Class Function Definitions ----------------------------------------------*/
drive_train::drive_train() : debug_update(0)
{
  // Clear motors
  for (int i=0; i<MOTOR_COUNT; i++)
  {
    motorArr[i] = {0};
  }
}

bool drive_train::add_motor(motor_e motor,
                            int     pinPWM,
                            int     pinDirFwd,
                            int     pinDirRev)
{
  if ((motor < MOTOR_FRONT_LEFT) || (motor >= MOTOR_COUNT))
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

void drive_train::update(int speed, int turn, int strafe)
{
  const float INPUT_PARAMETER_MAX = 500.0;
  const float MOTOR_PWM_MAX       = 1500.0;

  // Calculate raw Mecanum values (expected range [-1500..1500])
  int mecanumVal[MOTOR_COUNT  ] = {0};
  mecanumVal[MOTOR_FRONT_LEFT ] = speed + strafe + turn;
  mecanumVal[MOTOR_FRONT_RIGHT] = speed - strafe - turn;
  mecanumVal[MOTOR_REAR_RIGHT ] = speed + strafe - turn;
  mecanumVal[MOTOR_REAR_LEFT  ] = speed - strafe + turn;

  // Determine Scaling
  // 1. Find the strongest intended user input to determine the overall "throttle" percentage.
  // 2. Find the highest calculated wheel value to use as a normalization base.
  // 3. Create a multiplier that scales the wheels so that the fastest motor matches the 
  //    user's intended throttle, preventing clipping while maintaining the drive vector.
  int inputMax = std::max({std::abs(speed), std::abs(strafe), std::abs(turn)}); // [0..500]

  int calcMax  = std::max({std::abs(mecanumVal[MOTOR_FRONT_LEFT ]), 
                           std::abs(mecanumVal[MOTOR_FRONT_RIGHT]),
                           std::abs(mecanumVal[MOTOR_REAR_RIGHT ]),
                           std::abs(mecanumVal[MOTOR_REAR_LEFT  ])});

  float inputMaxPercent = static_cast<float>(inputMax) / INPUT_PARAMETER_MAX;

  float motorMultiplier = 0.0;
  if (calcMax > 0) // Avoid divide by 0
  {
    motorMultiplier = (inputMaxPercent * MOTOR_PWM_MAX) / static_cast<float>(calcMax);
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

// TODO: Stretch goal - also requires update to the update() function
void drive_train::calibrate(void)
{
  for (int i=0; i<MOTOR_COUNT; i++)
  {
    motorArr[i].valTrimFwd = 0;
    motorArr[i].valTrimRev = 0;

    // Force all motors to move forward
    pwm_set_gpio_level(std::abs(motorArr[i].pinPWM), 250); // Max speed is currently 1000
    gpio_put(motorArr[i].pinDirFwd, true);
    gpio_put(motorArr[i].pinDirRev, false);
  }
}

void drive_train::debug_print(void)
{
  ++debug_update;
}


/* EOF ----------------------------------------------------------------------*/

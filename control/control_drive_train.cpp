/******************************************************************************
 * @file control_drive_train.cpp
 * @brief Implementation of drive train control
 *****************************************************************************/

/** Includes ----------------------------------------------------------------*/
#include "control_drive_train.h"
#include "hardware/pwm.h"
#include <algorithm>


//TODO: Public ISRs?

/** Class Function Definitions ----------------------------------------------*/
drive_train::drive_train()
{
  // Clear motors
  for (int i=0; i<MOTOR_COUNT; i++)
  {
    motorArr[i] = {0};
  }
}

// TODO: Update to use driver_motor_l298n?
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

  pwm_config_set_clkdiv(&config, SYS_CLK_DIV_19KHZ);
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


  // Configure Encoder Pins (Interrupts???)
  //TODO: Stretch





  // Since motor was just added, remove any trims
  motorArr[motor].valTrimFwd = 0;
  motorArr[motor].valTrimRev = 0;

  motorArr[motor].initialized = true;

  return true;
}

bool drive_train::motor_initialized(motor_e motor)
{
  if ((motor < MOTOR_FRONT_LEFT) || (motor >= MOTOR_COUNT))
  {
    return false;
  }

  return motorArr[motor].initialized;
}

// TODO: Update to use trims from calibration routine
void drive_train::update(int speed, int turn, int strafe)
{ 
  int mecanumVal[MOTOR_COUNT]   = {0}; // value range [-1000..1000]
  mecanumVal[MOTOR_FRONT_LEFT]  = speed + strafe - turn;
  mecanumVal[MOTOR_FRONT_RIGHT] = speed - strafe - turn;
  mecanumVal[MOTOR_REAR_RIGHT]  = speed + strafe - turn;
  mecanumVal[MOTOR_REAR_LEFT]   = speed - strafe + turn;


  // Apply trim and normalize output so no motors are set over max PWM






  

  float motorValue[MOTOR_COUNT] = {0}; // value range [-1000.0, 1000.0]
  for (int i=0; i<MOTOR_COUNT; i++)
  {
    // Apply trim
    motorValue[i] = convert_motor_value(&motorArr[i], mecanumVal[i]);
  }

  // Convert to PWM [0..255]

  // Use value to determine 




  // normalize the values [-1.0, 1.0]
  float valMax = std::max({std::abs(motorValue[MOTOR_FRONT_LEFT]),
                           std::abs(motorValue[MOTOR_FRONT_RIGHT]),
                           std::abs(motorValue[MOTOR_REAR_RIGHT]),
                           std::abs(motorValue[MOTOR_REAR_LEFT])});
  if (valMax > 1.0)
  {
    for (int i=0; i<MOTOR_COUNT; i++)
    {
      motorValue[i] /= valMax;
    }
  }

  for (int i=0; i<MOTOR_COUNT; i++)
  {
    // Update speed
    //motorArr[i].pinPWM;

    // Update direction
    //motorArr[i].pinDirA
    //motorArr[i].pinDirB
  }
}

void drive_train::calibrate(void)
{
  for (int i=0; i<MOTOR_COUNT; i++)
  {
    motorArr[i].valTrimFwd = 0;
    motorArr[i].valTrimRev = 0;
  }
}


/* Private Function Definitions ---------------------------------------------*/
float drive_train::convert_motor_value(motor_t* pMotor, int currentValue)
{
  if (pMotor == nullptr)
  {
    return 0.0;
  }

  // currentValue is a float [-1000,1000]
  
  // TODO: Should trim be a percentage?
    // motor a is 95% power of motor b when going forward so weaken b to match a?
    // handle the math here to normalize the motors

  // Check if forward trim is needed
  if (currentValue > pMotor->valTrimFwd)
  {
    currentValue -= pMotor->valTrimFwd;
  }
  // Check if reverse trim is needed
  else if (std::abs(currentValue) > pMotor->valTrimRev)
  {
    currentValue += pMotor->valTrimRev;
  }
  else
  {
    currentValue = 0.0;
  }

  return static_cast<float>(currentValue);
}


/* EOF ----------------------------------------------------------------------*/

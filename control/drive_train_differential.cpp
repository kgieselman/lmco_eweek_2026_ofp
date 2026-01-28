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
volatile uint g_encoderLeftCounter = 0;
void __ISR_encoder_left(uint gpio, uint32_t events)
{
  ++g_encoderLeftCounter;
}

volatile uint g_encoderRightCounter = 0;
void __ISR_encoder_right(uint gpio, uint32_t events)
{
 ++g_encoderRightCounter;
}

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
                                         int     pinDirRev,
                                         int     pinEnc)
{
  if ((motor < MOTOR_LEFT) || (motor >= MOTOR_COUNT))
  {
    // Invalid motor
    return false;
  }

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

  // Configure optional parameters
  if ((pinEnc >= PICO_GPIO_PIN_MIN) && (pinEnc <= PICO_GPIO_PIN_MAX))
  {
    gpio_init(pinEnc);
    gpio_set_dir(pinEnc, GPIO_IN);
    gpio_pull_up(pinEnc); // Ensure the pin is high by default // TODO: Verify this is needed

    switch (motor)
    {
      case MOTOR_LEFT:
      {
        gpio_set_irq_enabled_with_callback(pinEnc,
                                           GPIO_IRQ_EDGE_RISE,
                                           false,
                                           &__ISR_encoder_left);
        break;
      }
      case MOTOR_RIGHT:
      {
        gpio_set_irq_enabled_with_callback(pinEnc,
                                           GPIO_IRQ_EDGE_RISE,
                                           false,
                                           &__ISR_encoder_right);
        break;
      }
      default:
      {
        // ERROR: Invalid motor
        break;
      }
    }
  }

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
  // Verify that motors have been initialized before proceeding
  bool motorsVerified = true;
  for (int i=0; i<MOTOR_COUNT; i++)
  {
    motorsVerified &= m_motors[i].initialized;
  }

  if (motorsVerified)
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
      else if (calcVal[i] < 0)
      {
        float trimmedVal = static_cast<float>(calcVal[i]) * m_motors[i].valTrimRev;
        calcVal[i] = static_cast<int>(trimmedVal);
      }
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
  else
  {
    printf("ERROR: Motor not intialize\n");
  }
}

void drive_train_differential::print_update(void)
{
  ++m_debugUpdate;
}

void drive_train_differential::stop_motors(void)
{
  for (int i=0; i<MOTOR_COUNT; i++)
  {
    gpio_put(m_motors[i].pinDirFwd, 0);
    gpio_put(m_motors[i].pinDirRev, 0);
    pwm_set_gpio_level(m_motors[i].pinPWM, 0);
  }
}

void drive_train_differential::calibrate_action(bool forward, int pwmVal, int* pArrPulses)
{
  if (pArrPulses == nullptr)
  {
    printf("ERROR: NULL Pointer\n");
    return;
  }

  // Disable interrupts/clear counters/set motors to spin forward
  for (int i=0; i<MOTOR_COUNT; i++)
  {
    // Disable the Encoder interrupts
    gpio_set_irq_enabled(m_motors[i].pinEncoder, GPIO_IRQ_EDGE_RISE, false);

    // Set motor to spin forward at half speed
    pwm_set_gpio_level(m_motors[i].pinPWM, pwmVal);
    gpio_put(m_motors[i].pinDirFwd, forward);
    gpio_put(m_motors[i].pinDirRev, !forward);
  }
  sleep_ms(MOTOR_SETTLE_TIME_MS);

  // Clear counter and enable interrupts
  g_encoderLeftCounter  = 0;
  g_encoderRightCounter = 0;
  for (int i=0; i<MOTOR_COUNT; i++)
  {
    // Clear any phantom interrupts
    gpio_acknowledge_irq(m_motors[i].pinEncoder, GPIO_IRQ_EDGE_RISE);
    gpio_set_irq_enabled(m_motors[i].pinEncoder, GPIO_IRQ_EDGE_RISE, true);
  }

  sleep_ms(CAL_MOTOR_COUNT_TIME_MS);

  // Loop in opposite direction for equality
  for (int i=(MOTOR_COUNT - 1); i>=0; i--)
  {
    gpio_set_irq_enabled(m_motors[i].pinEncoder, GPIO_IRQ_EDGE_RISE, false);
  }

  pArrPulses[MOTOR_LEFT] = g_encoderLeftCounter;
  pArrPulses[MOTOR_RIGHT] = g_encoderRightCounter;
}

void drive_train_differential::calibrate(void)
{
  // Verify that encoder has been setup
  bool readyToCal = true;
  for (int i=0; i<MOTOR_COUNT; i++)
  {
    // TODO: Define these constants somewhere universal to the project
    bool pinValid = (m_motors[i].pinEncoder >= PICO_GPIO_PIN_MIN) &&
                    (m_motors[i].pinEncoder <= PICO_GPIO_PIN_MAX);
    readyToCal &= pinValid;
  }

  if (readyToCal)
  {
    // Create arrays to hold pulse counts
    int fwdPulses[MOTOR_COUNT] = {0};
    int revPulses[MOTOR_COUNT] = {0};

    stop_motors();
    sleep_ms(MOTOR_SETTLE_TIME_MS);

    calibrate_action(true, PWM_TOP_COUNT/2, fwdPulses);

    stop_motors();
    sleep_ms(MOTOR_SETTLE_TIME_MS);

    calibrate_action(false, PWM_TOP_COUNT/2, revPulses);

    stop_motors();

    // Determine the weaker motor and store that value
    int minFwdPulses = std::min({fwdPulses[MOTOR_LEFT], fwdPulses[MOTOR_RIGHT]});
    int minRevPulses = std::min({revPulses[MOTOR_LEFT], revPulses[MOTOR_RIGHT]});

    // Check if both motors were spinning
    if (minFwdPulses > 0)
    {
      m_motors[MOTOR_LEFT].valTrimFwd  = static_cast<float>(minFwdPulses) /
                                         static_cast<float>(fwdPulses[MOTOR_LEFT]);
      m_motors[MOTOR_RIGHT].valTrimFwd = static_cast<float>(minFwdPulses) /
                                         static_cast<float>(fwdPulses[MOTOR_RIGHT]);
    }
    else
    {
      // At least one motor was stopped, apply default trim
      m_motors[MOTOR_LEFT].valTrimFwd  = DEFAULT_TRIM;
      m_motors[MOTOR_RIGHT].valTrimFwd = DEFAULT_TRIM;
    }

    // Check if both motors were spinning
    if (minRevPulses > 0)
    {
      m_motors[MOTOR_LEFT].valTrimRev  = static_cast<float>(minRevPulses) /
                                         static_cast<float>(revPulses[MOTOR_LEFT]);
      m_motors[MOTOR_RIGHT].valTrimRev = static_cast<float>(minRevPulses) /
                                         static_cast<float>(revPulses[MOTOR_RIGHT]);
    }
    else
    {
      // At least one motor was stopped, apply default trim
      m_motors[MOTOR_LEFT].valTrimRev  = DEFAULT_TRIM;
      m_motors[MOTOR_RIGHT].valTrimRev = DEFAULT_TRIM;
    }
  }
  else
  {
    // No encoder pin configured, apply default trim
    for (int i=0; i<MOTOR_COUNT; i++)
    {
      m_motors[i].valTrimFwd = DEFAULT_TRIM;
      m_motors[i].valTrimRev = DEFAULT_TRIM;
    }
  }
}


/* EOF ----------------------------------------------------------------------*/

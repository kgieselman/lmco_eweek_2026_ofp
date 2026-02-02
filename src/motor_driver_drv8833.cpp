/*******************************************************************************
 * @file motor_driver_drv8833.cpp
 * @brief Implementation of DRV8833 motor driver interface
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "motor_driver_drv8833.h"
#include "error_handler.h"

#ifndef UNIT_TEST
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#else
/* Mock implementations for unit testing */
#include <cstdio>
static inline void gpio_set_function(int pin, int func) { (void)pin; (void)func; }
static inline unsigned int pwm_gpio_to_slice_num(int pin) { return pin / 2; }
static inline unsigned int pwm_gpio_to_channel(int pin) { return pin % 2; }
struct pwm_config { float div; };
static inline pwm_config pwm_get_default_config() { return {1.0f}; }
static inline void pwm_config_set_clkdiv(pwm_config* c, float d) { c->div = d; }
static inline void pwm_init(unsigned int s, pwm_config* c, bool e) { (void)s; (void)c; (void)e; }
static inline void pwm_set_wrap(unsigned int s, int t) { (void)s; (void)t; }
static inline void pwm_set_chan_level(unsigned int s, unsigned int c, int l) { (void)s; (void)c; (void)l; }
static inline void pwm_set_enabled(unsigned int s, bool e) { (void)s; (void)e; }
static inline void pwm_set_gpio_level(int p, int l) { (void)p; (void)l; }
#define GPIO_FUNC_PWM 0
#endif


/* Method Definitions --------------------------------------------------------*/

MotorDriverDRV8833::MotorDriverDRV8833(int pwmFreqHz)
  : m_pwmFreqHz(pwmFreqHz)
  , m_defaultStopMode(STOP_COAST)
{
  /* Calculate clock divider for desired frequency */
  m_pwmClkDiv = calculateClockDivider(pwmFreqHz);

  /* Initialize motor configurations */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    m_motors[i].configured = false;
    m_motors[i].pinIn1 = -1;
    m_motors[i].pinIn2 = -1;
    m_motors[i].pinEncoder = -1;
    m_motors[i].currentValue = 0;
  }
}

MotorDriverDRV8833::~MotorDriverDRV8833()
{
  /* Stop all motors on destruction */
  stopAll(STOP_COAST);
}

bool MotorDriverDRV8833::configureMotor(MotorChannel channel,
                                         int pinIn1,
                                         int pinIn2,
                                         int pinEncoder)
{
  /* Validate channel */
  if (channel >= MOTOR_COUNT)
  {
    ERROR_REPORT(ERROR_OUT_OF_RANGE);
    return false;
  }

  /* Validate pins */
  if (!validatePin(pinIn1) || !validatePin(pinIn2))
  {
    ERROR_REPORT(ERROR_DT_INVALID_PIN);
    return false;
  }

  /* Validate encoder pin if specified */
  if (pinEncoder != -1 && !validatePin(pinEncoder))
  {
    ERROR_REPORT(ERROR_DT_INVALID_PIN);
    return false;
  }

  /* Initialize PWM pins */
  if (!initPwmPin(pinIn1) || !initPwmPin(pinIn2))
  {
    return false;
  }

  /* Store configuration */
  m_motors[channel].pinIn1 = pinIn1;
  m_motors[channel].pinIn2 = pinIn2;
  m_motors[channel].pinEncoder = pinEncoder;
  m_motors[channel].currentValue = 0;
  m_motors[channel].configured = true;

  return true;
}

bool MotorDriverDRV8833::setMotor(MotorChannel channel, int value)
{
  return setMotorWithTrim(channel, value, 1.0f);
}

bool MotorDriverDRV8833::setMotorWithTrim(MotorChannel channel, int value, float trim)
{
  /* Validate channel */
  if (channel >= MOTOR_COUNT)
  {
    ERROR_REPORT(ERROR_OUT_OF_RANGE);
    return false;
  }

  /* Check if channel is configured */
  if (!m_motors[channel].configured)
  {
    ERROR_REPORT(ERROR_DT_NOT_INIT);
    return false;
  }

  /* Clamp value to valid range */
  value = clampMotorValue(value);

  /* Apply trim (clamp trim to valid range) */
  if (trim < 0.0f) trim = 0.0f;
  if (trim > 1.0f) trim = 1.0f;

  /* Store current value */
  m_motors[channel].currentValue = value;

  /* Handle zero value (stop) */
  if (value == 0)
  {
    stop(channel, m_defaultStopMode);
    return true;
  }

  /* Calculate PWM duty cycle */
  /* Map [-500, +500] to [0, PWM_TOP_COUNT] */
  int absValue = (value < 0) ? -value : value;
  uint16_t dutyCycle = static_cast<uint16_t>((absValue * PWM_TOP_COUNT * trim) / MOTOR_VALUE_MAX);

  /* Set PWM outputs based on direction */
  if (value > 0)
  {
    /* Forward: IN1 = PWM, IN2 = 0 */
    setPwmDuty(m_motors[channel].pinIn1, dutyCycle);
    setPwmDuty(m_motors[channel].pinIn2, 0);
  }
  else
  {
    /* Reverse: IN1 = 0, IN2 = PWM */
    setPwmDuty(m_motors[channel].pinIn1, 0);
    setPwmDuty(m_motors[channel].pinIn2, dutyCycle);
  }

  return true;
}

void MotorDriverDRV8833::coast(MotorChannel channel)
{
  if (channel >= MOTOR_COUNT || !m_motors[channel].configured)
  {
    return;
  }

  /* Coast: both outputs LOW */
  setPwmDuty(m_motors[channel].pinIn1, 0);
  setPwmDuty(m_motors[channel].pinIn2, 0);
  m_motors[channel].currentValue = 0;
}

void MotorDriverDRV8833::brake(MotorChannel channel)
{
  if (channel >= MOTOR_COUNT || !m_motors[channel].configured)
  {
    return;
  }

  /* Brake: both outputs HIGH */
  setPwmDuty(m_motors[channel].pinIn1, PWM_TOP_COUNT);
  setPwmDuty(m_motors[channel].pinIn2, PWM_TOP_COUNT);
  m_motors[channel].currentValue = 0;
}

void MotorDriverDRV8833::stop(MotorChannel channel, StopMode mode)
{
  if (mode == STOP_BRAKE)
  {
    brake(channel);
  }
  else
  {
    coast(channel);
  }
}

void MotorDriverDRV8833::stopAll(StopMode mode)
{
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    stop(static_cast<MotorChannel>(i), mode);
  }
}

bool MotorDriverDRV8833::isConfigured(MotorChannel channel) const
{
  if (channel >= MOTOR_COUNT)
  {
    return false;
  }
  return m_motors[channel].configured;
}

bool MotorDriverDRV8833::isFullyConfigured(void) const
{
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    if (!m_motors[i].configured)
    {
      return false;
    }
  }
  return true;
}

int MotorDriverDRV8833::getEncoderPin(MotorChannel channel) const
{
  if (channel >= MOTOR_COUNT)
  {
    return -1;
  }
  return m_motors[channel].pinEncoder;
}

bool MotorDriverDRV8833::validatePin(int pin) const
{
  return (pin >= GPIO_PIN_MIN) && (pin <= GPIO_PIN_MAX);
}

bool MotorDriverDRV8833::initPwmPin(int pin)
{
  if (!validatePin(pin))
  {
    ERROR_REPORT(ERROR_DT_INVALID_PIN);
    return false;
  }

  /* Configure pin for PWM function */
  gpio_set_function(pin, GPIO_FUNC_PWM);

  /* Get PWM slice and channel for this pin */
  uint sliceNum = pwm_gpio_to_slice_num(pin);
  uint chanNum  = pwm_gpio_to_channel(pin);

  /* Configure PWM with calculated parameters */
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, m_pwmClkDiv);
  pwm_init(sliceNum, &config, true);
  pwm_set_wrap(sliceNum, PWM_TOP_COUNT);
  pwm_set_chan_level(sliceNum, chanNum, 0);  /* Start with output off */
  pwm_set_enabled(sliceNum, true);

  return true;
}

void MotorDriverDRV8833::setPwmDuty(int pin, uint16_t dutyCycle)
{
  pwm_set_gpio_level(pin, dutyCycle);
}

float MotorDriverDRV8833::calculateClockDivider(int freqHz) const
{
  /* 
   * PWM frequency = SYS_CLK / (divider * (wrap + 1))
   * divider = SYS_CLK / (freqHz * (wrap + 1))
   */
  float divider = static_cast<float>(SYS_CLK_HZ) / 
                  (static_cast<float>(freqHz) * (PWM_TOP_COUNT + 1));
  
  /* Clamp to valid range (1.0 to 255.9375) */
  if (divider < 1.0f) divider = 1.0f;
  if (divider > 255.0f) divider = 255.0f;
  
  return divider;
}

int MotorDriverDRV8833::clampMotorValue(int value) const
{
  if (value < MOTOR_VALUE_MIN) return MOTOR_VALUE_MIN;
  if (value > MOTOR_VALUE_MAX) return MOTOR_VALUE_MAX;
  return value;
}


/* EOF -----------------------------------------------------------------------*/

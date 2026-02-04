/*******************************************************************************
 * @file motor_driver_l298n.cpp
 * @brief Implementation of L298N motor driver interface
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "motor_driver_l298n.h"
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
static inline void gpio_init(int p) { (void)p; }
static inline void gpio_set_dir(int p, bool d) { (void)p; (void)d; }
static inline void gpio_put(int p, bool v) { (void)p; (void)v; }
#define GPIO_FUNC_PWM 0
#define GPIO_OUT true
#endif


/* Method Definitions --------------------------------------------------------*/

MotorDriverL298N::MotorDriverL298N(int pwmFreqHz)
  : m_pwmFreqHz(pwmFreqHz)
  , m_defaultStopMode(STOP_COAST)
{
  /* Calculate clock divider for desired frequency */
  m_pwmClkDiv = calculateClockDivider(pwmFreqHz);

  /* Initialize motor configurations */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    m_motors[i].configured   = false;
    m_motors[i].pinPwm       = PIN_INVALID;
    m_motors[i].pinDirFwd    = PIN_INVALID;
    m_motors[i].pinDirRev    = PIN_INVALID;
    m_motors[i].pinEncoder   = PIN_INVALID;
    m_motors[i].currentValue = 0;
  }
}

MotorDriverL298N::~MotorDriverL298N()
{
  /* Stop all motors on destruction */
  stopAll(STOP_COAST);
}

bool MotorDriverL298N::configureMotor(MotorChannel_e channel,
                                      int            pinPwm,
                                      int            pinDirFwd,
                                      int            pinDirRev,
                                      int            pinEncoder)
{
  /* Validate channel */
  if (channel >= MOTOR_COUNT)
  {
    ERROR_REPORT(ERROR_OUT_OF_RANGE);
    return false;
  }

  /* Validate pins */
  if (!validatePin(pinPwm) || !validatePin(pinDirFwd) || !validatePin(pinDirRev))
  {
    ERROR_REPORT(ERROR_DT_INVALID_PIN);
    return false;
  }

  /* Validate encoder pin if specified */
  bool encoderPinSpecified = (pinEncoder != PIN_INVALID);
  if (encoderPinSpecified && !validatePin(pinEncoder))
  {
    ERROR_REPORT(ERROR_DT_INVALID_PIN);
    return false;
  }

  /* Initialize PWM pin */
  if (!initPwmPin(pinPwm))
  {
    return false;
  }

  /* Initialize direction pins */
  if (!initDirectionPin(pinDirFwd) || !initDirectionPin(pinDirRev))
  {
    return false;
  }

  /* Store configuration */
  m_motors[channel].pinPwm = pinPwm;
  m_motors[channel].pinDirFwd = pinDirFwd;
  m_motors[channel].pinDirRev = pinDirRev;
  m_motors[channel].pinEncoder = pinEncoder;
  m_motors[channel].currentValue = 0;
  m_motors[channel].configured = true;

  return true;
}

bool MotorDriverL298N::setMotor(MotorChannel_e channel, int value)
{
  return setMotorWithTrim(channel, value, 1.0f);
}

bool MotorDriverL298N::setMotorWithTrim(MotorChannel_e channel, int value, float trim)
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

  /* Set direction pins based on direction */
  if (value > 0)
  {
    /* Forward: IN1 = HIGH, IN2 = LOW */
    setDigitalOut(m_motors[channel].pinDirFwd, true);
    setDigitalOut(m_motors[channel].pinDirRev, false);
  }
  else
  {
    /* Reverse: IN1 = LOW, IN2 = HIGH */
    setDigitalOut(m_motors[channel].pinDirFwd, false);
    setDigitalOut(m_motors[channel].pinDirRev, true);
  }

  /* Set PWM duty cycle */
  setPwmDuty(m_motors[channel].pinPwm, dutyCycle);

  return true;
}

void MotorDriverL298N::coast(MotorChannel_e channel)
{
  if (channel >= MOTOR_COUNT || !m_motors[channel].configured)
  {
    return;
  }

  /* Coast: PWM = 0 (direction pins don't matter) */
  setPwmDuty(m_motors[channel].pinPwm, 0);
  setDigitalOut(m_motors[channel].pinDirFwd, false);
  setDigitalOut(m_motors[channel].pinDirRev, false);
  m_motors[channel].currentValue = 0;
}

void MotorDriverL298N::brake(MotorChannel_e channel)
{
  if (channel >= MOTOR_COUNT || !m_motors[channel].configured)
  {
    return;
  }

  /* Brake: PWM = 100%, IN1 = HIGH, IN2 = HIGH */
  setPwmDuty(m_motors[channel].pinPwm, PWM_TOP_COUNT);
  setDigitalOut(m_motors[channel].pinDirFwd, true);
  setDigitalOut(m_motors[channel].pinDirRev, true);
  m_motors[channel].currentValue = 0;
}

void MotorDriverL298N::stop(MotorChannel_e channel, StopMode_e mode)
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

void MotorDriverL298N::stopAll(StopMode_e mode)
{
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    stop(static_cast<MotorChannel_e>(i), mode);
  }
}

bool MotorDriverL298N::isConfigured(MotorChannel_e channel) const
{
  if (channel >= MOTOR_COUNT)
  {
    return false;
  }
  return m_motors[channel].configured;
}

bool MotorDriverL298N::isFullyConfigured(void) const
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

int MotorDriverL298N::getEncoderPin(MotorChannel_e channel) const
{
  if (channel >= MOTOR_COUNT)
  {
    return PIN_INVALID;
  }
  return m_motors[channel].pinEncoder;
}

bool MotorDriverL298N::validatePin(int pin) const
{
  return (pin >= GPIO_PIN_MIN) && (pin <= GPIO_PIN_MAX);
}

bool MotorDriverL298N::initPwmPin(int pin)
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

bool MotorDriverL298N::initDirectionPin(int pin)
{
  if (!validatePin(pin))
  {
    ERROR_REPORT(ERROR_DT_INVALID_PIN);
    return false;
  }

  /* Configure pin as digital output */
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_OUT);
  gpio_put(pin, false);  /* Start with output low */

  return true;
}

void MotorDriverL298N::setPwmDuty(int pin, uint16_t dutyCycle)
{
  pwm_set_gpio_level(pin, dutyCycle);
}

void MotorDriverL298N::setDigitalOut(int pin, bool state)
{
  gpio_put(pin, state);
}

float MotorDriverL298N::calculateClockDivider(int freqHz) const
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


/* EOF -----------------------------------------------------------------------*/

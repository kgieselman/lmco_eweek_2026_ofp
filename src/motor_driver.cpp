/*******************************************************************************
 * @file motor_driver.cpp
 * @brief Implementation of generic motor driver interface
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "motor_driver.h"
#include "error_handler.h"
#include "pinout.h"

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

MotorDriver::MotorDriver(Mode_e mode, int pwmFreqHz)
  : m_mode(mode)
  , m_pwmFreqHz(pwmFreqHz)
  , m_defaultStopMode(STOP_COAST)
{
  /* Calculate clock divider for desired frequency */
  m_pwmClkDiv = calculateClockDivider(pwmFreqHz);

  /* Initialize motor configurations */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    m_motors[i].configured   = false;
    m_motors[i].pin1         = PIN_INVALID;
    m_motors[i].pin2         = PIN_INVALID;
    m_motors[i].pin3         = PIN_INVALID;
    m_motors[i].pinEncoder   = PIN_INVALID;
    m_motors[i].currentValue = 0;
  }
}

MotorDriver::~MotorDriver()
{
  /* Stop all motors on destruction */
  stopAll(STOP_COAST);
}

bool MotorDriver::configureMotor(MotorChannel_e channel,
                                 int            pinIn1,
                                 int            pinIn2,
                                 int            pinEncoder)
{
  /* This overload is only valid for MODE_2PWM */
  if (m_mode != MODE_2PWM)
  {
    ERROR_REPORT(ERROR_INVALID_PARAM);
    return false;
  }

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
  if ((pinEncoder != PIN_INVALID) && !validatePin(pinEncoder))
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
  m_motors[channel].pin1         = pinIn1;
  m_motors[channel].pin2         = pinIn2;
  m_motors[channel].pin3         = PIN_INVALID;
  m_motors[channel].pinEncoder   = pinEncoder;
  m_motors[channel].currentValue = 0;
  m_motors[channel].configured   = true;

  return true;
}

bool MotorDriver::configureMotor(MotorChannel_e channel,
                                 int            pinPwm,
                                 int            pinDirFwd,
                                 int            pinDirRev,
                                 int            pinEncoder)
{
  /* This overload is only valid for MODE_1PWM_2DIR */
  if (m_mode != MODE_1PWM_2DIR)
  {
    ERROR_REPORT(ERROR_INVALID_PARAM);
    return false;
  }

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
  if ((pinEncoder != PIN_INVALID) && !validatePin(pinEncoder))
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
  m_motors[channel].pin1         = pinPwm;
  m_motors[channel].pin2         = pinDirFwd;
  m_motors[channel].pin3         = pinDirRev;
  m_motors[channel].pinEncoder   = pinEncoder;
  m_motors[channel].currentValue = 0;
  m_motors[channel].configured   = true;

  return true;
}

bool MotorDriver::setMotor(MotorChannel_e channel, int value)
{
  return setMotorWithTrim(channel, value, 1.0f);
}

bool MotorDriver::setMotorWithTrim(MotorChannel_e channel, int value, float trim)
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

  /* Clamp trim to valid range */
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
  int absValue = (value < 0) ? -value : value;
  uint16_t dutyCycle = static_cast<uint16_t>(
    (absValue * PWM_TOP_COUNT * trim) / MOTOR_VALUE_MAX);
  bool forward = (value > 0);

  /* Apply output based on wiring mode */
  if (m_mode == MODE_2PWM)
  {
    applyOutput2Pwm(m_motors[channel], dutyCycle, forward);
  }
  else
  {
    applyOutput1Pwm2Dir(m_motors[channel], dutyCycle, forward);
  }

  return true;
}

void MotorDriver::coast(MotorChannel_e channel)
{
  if (channel >= MOTOR_COUNT || !m_motors[channel].configured)
  {
    return;
  }

  if (m_mode == MODE_2PWM)
  {
    /* Coast: both PWM outputs LOW */
    setPwmDuty(m_motors[channel].pin1, 0);
    setPwmDuty(m_motors[channel].pin2, 0);
  }
  else
  {
    /* Coast: PWM = 0, direction pins LOW */
    setPwmDuty(m_motors[channel].pin1, 0);
    setDigitalOut(m_motors[channel].pin2, false);
    setDigitalOut(m_motors[channel].pin3, false);
  }

  m_motors[channel].currentValue = 0;
}

void MotorDriver::brake(MotorChannel_e channel)
{
  if (channel >= MOTOR_COUNT || !m_motors[channel].configured)
  {
    return;
  }

  if (m_mode == MODE_2PWM)
  {
    /* Brake: both PWM outputs HIGH */
    setPwmDuty(m_motors[channel].pin1, PWM_TOP_COUNT);
    setPwmDuty(m_motors[channel].pin2, PWM_TOP_COUNT);
  }
  else
  {
    /* Brake: PWM = 100%, both direction pins HIGH */
    setPwmDuty(m_motors[channel].pin1, PWM_TOP_COUNT);
    setDigitalOut(m_motors[channel].pin2, true);
    setDigitalOut(m_motors[channel].pin3, true);
  }

  m_motors[channel].currentValue = 0;
}

void MotorDriver::stop(MotorChannel_e channel, StopMode_e mode)
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

void MotorDriver::stopAll(StopMode_e mode)
{
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    stop(static_cast<MotorChannel_e>(i), mode);
  }
}

bool MotorDriver::isConfigured(MotorChannel_e channel) const
{
  if (channel >= MOTOR_COUNT)
  {
    return false;
  }
  return m_motors[channel].configured;
}

bool MotorDriver::isFullyConfigured() const
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

int MotorDriver::getEncoderPin(MotorChannel_e channel) const
{
  if (channel >= MOTOR_COUNT)
  {
    return PIN_INVALID;
  }
  return m_motors[channel].pinEncoder;
}

bool MotorDriver::validatePin(int pin) const
{
  return (pin >= GPIO_PIN_MIN) && (pin <= GPIO_PIN_MAX);
}

bool MotorDriver::initPwmPin(int pin)
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

bool MotorDriver::initDirectionPin(int pin)
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

void MotorDriver::setPwmDuty(int pin, uint16_t dutyCycle)
{
  pwm_set_gpio_level(pin, dutyCycle);
}

void MotorDriver::setDigitalOut(int pin, bool state)
{
  gpio_put(pin, state);
}

void MotorDriver::applyOutput2Pwm(const MotorConfig& motor,
                                   uint16_t dutyCycle,
                                   bool forward)
{
  if (forward)
  {
    /* Forward: IN1 = PWM, IN2 = 0 */
    setPwmDuty(motor.pin1, dutyCycle);
    setPwmDuty(motor.pin2, 0);
  }
  else
  {
    /* Reverse: IN1 = 0, IN2 = PWM */
    setPwmDuty(motor.pin1, 0);
    setPwmDuty(motor.pin2, dutyCycle);
  }
}

void MotorDriver::applyOutput1Pwm2Dir(const MotorConfig& motor,
                                       uint16_t dutyCycle,
                                       bool forward)
{
  /* Set direction pins first */
  if (forward)
  {
    setDigitalOut(motor.pin2, true);   /* DIR_FWD = HIGH */
    setDigitalOut(motor.pin3, false);  /* DIR_REV = LOW */
  }
  else
  {
    setDigitalOut(motor.pin2, false);  /* DIR_FWD = LOW */
    setDigitalOut(motor.pin3, true);   /* DIR_REV = HIGH */
  }

  /* Set PWM duty cycle */
  setPwmDuty(motor.pin1, dutyCycle);
}

float MotorDriver::calculateClockDivider(int freqHz) const
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

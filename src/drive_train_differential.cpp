/*******************************************************************************
 * @file drive_train_differential.cpp
 * @brief Implementation of differential drive train controller
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "drive_train_differential.h"
#include "config.h"
#include "error_handler.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <algorithm>
#include <cstdlib>

#if ENABLE_DEBUG
#include <stdio.h>
#endif


/* Interrupt Service Routines ------------------------------------------------*/
#if ENABLE_ENCODER_CALIBRATION
namespace {
  volatile uint32_t g_encoderLeftCount = 0;
  volatile uint32_t g_encoderRightCount = 0;
}

/*******************************************************************************
 * @brief Left encoder interrupt handler
 ******************************************************************************/
void __isr_encoder_left(uint gpio, uint32_t events)
{
  (void)gpio;
  (void)events;
  g_encoderLeftCount++;
}

/*******************************************************************************
 * @brief Right encoder interrupt handler
 ******************************************************************************/
void __isr_encoder_right(uint gpio, uint32_t events)
{
  (void)gpio;
  (void)events;
  g_encoderRightCount++;
}
#endif /* ENABLE_ENCODER_CALIBRATION */


/* Method Definitions --------------------------------------------------------*/
DriveTrainDifferential::DriveTrainDifferential()
  : DriveTrain()
{
  /* Initialize all motors to default state */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    m_motors[i].initialized = false;
    m_motors[i].pinPwm = -1;
    m_motors[i].pinDirFwd = -1;
    m_motors[i].pinDirRev = -1;
    m_motors[i].pinEncoder = -1;
    m_motors[i].trimFwd = DEFAULT_TRIM;
    m_motors[i].trimRev = DEFAULT_TRIM;
  }
}

DriveTrainDifferential::~DriveTrainDifferential()
{
  stop();
}

bool DriveTrainDifferential::addMotor(MotorId motor,
                                       int pinPwm,
                                       int pinDirFwd,
                                       int pinDirRev,
                                       int pinEncoder)
{
  /* Validate motor ID */
  if (motor < MOTOR_LEFT || motor >= MOTOR_COUNT)
  {
    ERROR_REPORT(ERROR_DT_INVALID_MOTOR);
    return false;
  }

  /* Validate pins */
  if (!validatePin(pinPwm) || !validatePin(pinDirFwd) || !validatePin(pinDirRev))
  {
    ERROR_REPORT(ERROR_DT_INVALID_PIN);
    return false;
  }

  /* Store pin assignments */
  m_motors[motor].pinPwm = pinPwm;
  m_motors[motor].pinDirFwd = pinDirFwd;
  m_motors[motor].pinDirRev = pinDirRev;
  m_motors[motor].pinEncoder = pinEncoder;

  /* Initialize PWM pin */
  if (!initPwmPin(pinPwm, PWM_TOP_COUNT, PWM_CLK_DIV))
  {
    ERROR_REPORT(ERROR_DT_PWM_FAILED);
    return false;
  }

  /* Initialize direction pins */
  if (!initDirectionPin(pinDirFwd) || !initDirectionPin(pinDirRev))
  {
    return false;
  }

  /* Reset trim values */
  m_motors[motor].trimFwd = DEFAULT_TRIM;
  m_motors[motor].trimRev = DEFAULT_TRIM;

  /* Configure encoder if provided */
#if ENABLE_ENCODER_CALIBRATION
  if (validatePin(pinEncoder))
  {
    gpio_init(pinEncoder);
    gpio_set_dir(pinEncoder, GPIO_IN);
    gpio_pull_up(pinEncoder);

    /* Set up encoder interrupt based on motor */
    gpio_irq_callback_t callback = nullptr;
    if (motor == MOTOR_LEFT)
    {
      callback = __isr_encoder_left;
    }
    else if (motor == MOTOR_RIGHT)
    {
      callback = __isr_encoder_right;
    }

    if (callback != nullptr)
    {
      gpio_set_irq_enabled_with_callback(pinEncoder,
                                         GPIO_IRQ_EDGE_RISE,
                                         false,  /* Initially disabled */
                                         callback);
    }
  }
#else
  (void)pinEncoder;  /* Suppress unused warning */
#endif

  m_motors[motor].initialized = true;

#if ENABLE_DEBUG
  printf("[DriveTrain] Motor %d configured: PWM=%d, FWD=%d, REV=%d\n",
         motor, pinPwm, pinDirFwd, pinDirRev);
#endif

  return true;
}

void DriveTrainDifferential::update(void)
{
  /* Verify all motors are initialized */
  if (!isInitialized())
  {
    ERROR_REPORT(ERROR_DT_NOT_INIT);
    return;
  }

  /* Calculate raw motor values */
  int motorValues[MOTOR_COUNT];
  motorValues[MOTOR_LEFT]  = m_speed + m_turn;
  motorValues[MOTOR_RIGHT] = m_speed - m_turn;

  /* Apply trim values */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    float trim = (motorValues[i] > 0) ? m_motors[i].trimFwd : m_motors[i].trimRev;
    motorValues[i] = static_cast<int>(motorValues[i] * trim);
  }

  /* Determine scaling to prevent clipping */
  int inputMax = std::max(std::abs(m_speed), std::abs(m_turn));
  int calcMax = std::max(std::abs(motorValues[MOTOR_LEFT]),
                         std::abs(motorValues[MOTOR_RIGHT]));

  float multiplier = 0.0f;
  if (calcMax > 0)
  {
    float inputPercent = static_cast<float>(inputMax) / USER_INPUT_MAX;
    multiplier = (inputPercent * PWM_TOP_COUNT) / static_cast<float>(calcMax);
  }

  /* Apply to motors */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    setMotorOutput(m_motors[i], motorValues[i], multiplier);
  }
}

void DriveTrainDifferential::stop(void)
{
  m_speed = 0;
  m_turn = 0;

  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    if (m_motors[i].initialized)
    {
      pwm_set_gpio_level(m_motors[i].pinPwm, 0);
      gpio_put(m_motors[i].pinDirFwd, 0);
      gpio_put(m_motors[i].pinDirRev, 0);
    }
  }
}

void DriveTrainDifferential::calibrate(void)
{
#if ENABLE_ENCODER_CALIBRATION
  /* Verify encoders are configured */
  bool encodersReady = true;
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    if (!validatePin(m_motors[i].pinEncoder))
    {
      encodersReady = false;
      break;
    }
  }

  if (!encodersReady)
  {
    /* No encoders - use default trim */
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
      m_motors[i].trimFwd = DEFAULT_TRIM;
      m_motors[i].trimRev = DEFAULT_TRIM;
    }
    return;
  }

  /* Stop motors and wait for settle */
  stop();
  sleep_ms(MOTOR_SETTLE_TIME_MS);

  /* Measure forward direction */
  int fwdPulses[MOTOR_COUNT] = {0};
  measureMotorPulses(true, PWM_TOP_COUNT / 2, fwdPulses);

  stop();
  sleep_ms(MOTOR_SETTLE_TIME_MS);

  /* Measure reverse direction */
  int revPulses[MOTOR_COUNT] = {0};
  measureMotorPulses(false, PWM_TOP_COUNT / 2, revPulses);

  stop();

  /* Calculate trim values based on slower motor */
  int minFwd = std::min(fwdPulses[MOTOR_LEFT], fwdPulses[MOTOR_RIGHT]);
  int minRev = std::min(revPulses[MOTOR_LEFT], revPulses[MOTOR_RIGHT]);

  /* Apply forward trim */
  if (minFwd > 0)
  {
    m_motors[MOTOR_LEFT].trimFwd = 
      static_cast<float>(minFwd) / static_cast<float>(fwdPulses[MOTOR_LEFT]);
    m_motors[MOTOR_RIGHT].trimFwd = 
      static_cast<float>(minFwd) / static_cast<float>(fwdPulses[MOTOR_RIGHT]);
  }
  else
  {
    m_motors[MOTOR_LEFT].trimFwd  = DEFAULT_TRIM;
    m_motors[MOTOR_RIGHT].trimFwd = DEFAULT_TRIM;
  }

  /* Apply reverse trim */
  if (minRev > 0)
  {
    m_motors[MOTOR_LEFT].trimRev = 
      static_cast<float>(minRev) / static_cast<float>(revPulses[MOTOR_LEFT]);
    m_motors[MOTOR_RIGHT].trimRev = 
      static_cast<float>(minRev) / static_cast<float>(revPulses[MOTOR_RIGHT]);
  }
  else
  {
    m_motors[MOTOR_LEFT].trimRev  = DEFAULT_TRIM;
    m_motors[MOTOR_RIGHT].trimRev = DEFAULT_TRIM;
  }

#if ENABLE_DEBUG
  printf("[Calibration] Fwd pulses: L=%d R=%d\n", fwdPulses[MOTOR_LEFT], fwdPulses[MOTOR_RIGHT]);
  printf("[Calibration] Rev pulses: L=%d R=%d\n", revPulses[MOTOR_LEFT], revPulses[MOTOR_RIGHT]);
  printf("[Calibration] Trim L: fwd=%.3f rev=%.3f\n", 
         m_motors[MOTOR_LEFT].trimFwd, m_motors[MOTOR_LEFT].trimRev);
  printf("[Calibration] Trim R: fwd=%.3f rev=%.3f\n", 
         m_motors[MOTOR_RIGHT].trimFwd, m_motors[MOTOR_RIGHT].trimRev);
#endif

#else
  /* Encoders not enabled - use default trim */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    m_motors[i].trimFwd = DEFAULT_TRIM;
    m_motors[i].trimRev = DEFAULT_TRIM;
  }
#endif /* ENABLE_ENCODER_CALIBRATION */
}

bool DriveTrainDifferential::isInitialized(void) const
{
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    if (!m_motors[i].initialized)
    {
      return false;
    }
  }

  return true;
}

void DriveTrainDifferential::measureMotorPulses(bool forward, int pwmValue, int* pPulses)
{
#if ENABLE_ENCODER_CALIBRATION
  if (pPulses == nullptr)
  {
    ERROR_REPORT(ERROR_NULL_POINTER);
    return;
  }

  /* Disable interrupts and set motor direction */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    gpio_set_irq_enabled(m_motors[i].pinEncoder, GPIO_IRQ_EDGE_RISE, false);
    pwm_set_gpio_level(m_motors[i].pinPwm, pwmValue);
    gpio_put(m_motors[i].pinDirFwd, forward);
    gpio_put(m_motors[i].pinDirRev, !forward);
  }

  /* Let motors reach steady state */
  sleep_ms(MOTOR_SETTLE_TIME_MS);

  /* Clear counters and enable interrupts */
  g_encoderLeftCount = 0;
  g_encoderRightCount = 0;

  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    gpio_acknowledge_irq(m_motors[i].pinEncoder, GPIO_IRQ_EDGE_RISE);
    gpio_set_irq_enabled(m_motors[i].pinEncoder, GPIO_IRQ_EDGE_RISE, true);
  }

  /* Measure for calibration period */
  sleep_ms(MOTOR_CALIBRATION_TIME_MS);

  /* Disable interrupts and capture counts */
  for (int i = MOTOR_COUNT - 1; i >= 0; i--)
  {
    gpio_set_irq_enabled(m_motors[i].pinEncoder, GPIO_IRQ_EDGE_RISE, false);
  }

  pPulses[MOTOR_LEFT] = g_encoderLeftCount;
  pPulses[MOTOR_RIGHT] = g_encoderRightCount;
#else
  (void)forward;
  (void)pwmValue;
  if (pPulses != nullptr)
  {
    pPulses[MOTOR_LEFT]  = 0;
    pPulses[MOTOR_RIGHT] = 0;
  }
#endif /* ENABLE_ENCODER_CALIBRATION */
}

/* EOF -----------------------------------------------------------------------*/

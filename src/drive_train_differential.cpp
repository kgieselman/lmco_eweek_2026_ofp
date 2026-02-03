/*******************************************************************************
 * @file drive_train_differential.cpp
 * @brief Implementation of differential drive train controller
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "drive_train_differential.h"
#include "config.h"
#include "error_handler.h"

#ifndef UNIT_TEST
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#endif

#include <algorithm>
#include <cstdlib>

#if ENABLE_DEBUG
#include <stdio.h>
#endif


/* Interrupt Service Routines ------------------------------------------------*/
#if ENABLE_ENCODER_CALIBRATION && !defined(UNIT_TEST)
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
  , m_motorDriver()
{
  /* Initialize motor state */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    m_motorState[i].initialized = false;
    m_motorState[i].pinEncoder  = PIN_INVALID;
    m_motorState[i].trimFwd     = DEFAULT_TRIM;
    m_motorState[i].trimRev     = DEFAULT_TRIM;
  }
}

DriveTrainDifferential::~DriveTrainDifferential()
{
  stop();
}

MotorDriver::MotorChannel_e DriveTrainDifferential::getChannelForMotor(MotorId_e motor) const
{
  return (motor == MOTOR_LEFT) ? MotorDriver::MOTOR_A : MotorDriver::MOTOR_B;
}

bool DriveTrainDifferential::configureEncoder(MotorId_e motor, int pinEncoder)
{
  /* Store encoder pin for calibration */
  m_motorState[motor].pinEncoder = pinEncoder;

#if ENABLE_ENCODER_CALIBRATION && !defined(UNIT_TEST)
  if (pinEncoder >= GPIO_PIN_MIN && pinEncoder <= GPIO_PIN_MAX)
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
  (void)pinEncoder;
#endif

  return true;
}

#if USE_MOTOR_DRIVER_DRV8833
bool DriveTrainDifferential::addMotor(MotorId_e motor,
                                      int       pinIn1,
                                      int       pinIn2,
                                      int       pinEncoder)
{
  /* Validate motor ID */
  if (motor < MOTOR_LEFT || motor >= MOTOR_COUNT)
  {
    ERROR_REPORT(ERROR_DT_INVALID_MOTOR);
    return false;
  }

  /* Get motor channel using helper */
  MotorDriver::MotorChannel_e channel = getChannelForMotor(motor);

  /* Configure motor through driver */
  if (!m_motorDriver.configureMotor(channel, pinIn1, pinIn2, pinEncoder))
  {
    ERROR_REPORT(ERROR_DT_PWM_FAILED);
    return false;
  }

  /* Configure encoder using helper */
  configureEncoder(motor, pinEncoder);

  /* Reset trim values and mark initialized */
  m_motorState[motor].trimFwd = DEFAULT_TRIM;
  m_motorState[motor].trimRev = DEFAULT_TRIM;
  m_motorState[motor].initialized = true;

#if ENABLE_DEBUG
  printf("[DriveTrain] Motor %d configured (DRV8833): IN1=%d, IN2=%d\n",
         motor, pinIn1, pinIn2);
#endif

  return true;
}

#else /* L298N */

bool DriveTrainDifferential::addMotor(MotorId_e motor,
                                      int       pinPwm,
                                      int       pinDirFwd,
                                      int       pinDirRev,
                                      int       pinEncoder)
{
  /* Validate motor ID */
  if (motor < MOTOR_LEFT || motor >= MOTOR_COUNT)
  {
    ERROR_REPORT(ERROR_DT_INVALID_MOTOR);
    return false;
  }

  /* Get motor channel using helper */
  MotorDriver::MotorChannel_e channel = getChannelForMotor(motor);

  /* Configure motor through driver */
  if (!m_motorDriver.configureMotor(channel, pinPwm, pinDirFwd, pinDirRev, pinEncoder))
  {
    ERROR_REPORT(ERROR_DT_PWM_FAILED);
    return false;
  }

  /* Configure encoder using helper */
  configureEncoder(motor, pinEncoder);

  /* Reset trim values and mark initialized */
  m_motorState[motor].trimFwd = DEFAULT_TRIM;
  m_motorState[motor].trimRev = DEFAULT_TRIM;
  m_motorState[motor].initialized = true;

#if ENABLE_DEBUG
  printf("[DriveTrain] Motor %d configured (L298N): PWM=%d, FWD=%d, REV=%d\n",
         motor, pinPwm, pinDirFwd, pinDirRev);
#endif

  return true;
}
#endif /* USE_MOTOR_DRIVER_DRV8833 */

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

  /* Scale to prevent clipping while maintaining direction ratio */
  int maxAbs = std::max(std::abs(motorValues[MOTOR_LEFT]),
                        std::abs(motorValues[MOTOR_RIGHT]));
  
  if (maxAbs > USER_INPUT_MAX)
  {
    float scale = static_cast<float>(USER_INPUT_MAX) / static_cast<float>(maxAbs);
    motorValues[MOTOR_LEFT]  = static_cast<int>(motorValues[MOTOR_LEFT] * scale);
    motorValues[MOTOR_RIGHT] = static_cast<int>(motorValues[MOTOR_RIGHT] * scale);
  }

  /* Apply trim and output to motors */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    float trim = (motorValues[i] > 0) ? m_motorState[i].trimFwd : m_motorState[i].trimRev;
    MotorDriver::MotorChannel_e channel = 
      (i == MOTOR_LEFT) ? MotorDriver::MOTOR_A : MotorDriver::MOTOR_B;
    (void)m_motorDriver.setMotorWithTrim(channel, motorValues[i], trim);
  }
}

void DriveTrainDifferential::stop(void)
{
  m_speed = 0;
  m_turn = 0;

  m_motorDriver.stopAll();
}

void DriveTrainDifferential::calibrate(void)
{
#if ENABLE_ENCODER_CALIBRATION && !defined(UNIT_TEST)
  /* Verify encoders are configured */
  bool encodersReady = true;
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    if (m_motorState[i].pinEncoder < GPIO_PIN_MIN || 
        m_motorState[i].pinEncoder > GPIO_PIN_MAX)
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
      m_motorState[i].trimFwd = DEFAULT_TRIM;
      m_motorState[i].trimRev = DEFAULT_TRIM;
    }
    return;
  }

  /* Stop motors and wait for settle */
  stop();
  sleep_ms(MOTOR_SETTLE_TIME_MS);

  /* Measure forward direction at 50% speed */
  int fwdPulses[MOTOR_COUNT] = {0};
  measureMotorPulses(true, USER_INPUT_MAX / 2, fwdPulses);

  stop();
  sleep_ms(MOTOR_SETTLE_TIME_MS);

  /* Measure reverse direction */
  int revPulses[MOTOR_COUNT] = {0};
  measureMotorPulses(false, USER_INPUT_MAX / 2, revPulses);

  stop();

  /* Calculate trim values based on slower motor */
  int minFwd = std::min(fwdPulses[MOTOR_LEFT], fwdPulses[MOTOR_RIGHT]);
  int minRev = std::min(revPulses[MOTOR_LEFT], revPulses[MOTOR_RIGHT]);

  /* Apply forward trim */
  if (minFwd > 0)
  {
    m_motorState[MOTOR_LEFT].trimFwd = 
      static_cast<float>(minFwd) / static_cast<float>(fwdPulses[MOTOR_LEFT]);
    m_motorState[MOTOR_RIGHT].trimFwd = 
      static_cast<float>(minFwd) / static_cast<float>(fwdPulses[MOTOR_RIGHT]);
  }
  else
  {
    m_motorState[MOTOR_LEFT].trimFwd  = DEFAULT_TRIM;
    m_motorState[MOTOR_RIGHT].trimFwd = DEFAULT_TRIM;
  }

  /* Apply reverse trim */
  if (minRev > 0)
  {
    m_motorState[MOTOR_LEFT].trimRev = 
      static_cast<float>(minRev) / static_cast<float>(revPulses[MOTOR_LEFT]);
    m_motorState[MOTOR_RIGHT].trimRev = 
      static_cast<float>(minRev) / static_cast<float>(revPulses[MOTOR_RIGHT]);
  }
  else
  {
    m_motorState[MOTOR_LEFT].trimRev  = DEFAULT_TRIM;
    m_motorState[MOTOR_RIGHT].trimRev = DEFAULT_TRIM;
  }

#if ENABLE_DEBUG
  printf("[Calibration] Fwd pulses: L=%d R=%d\n", fwdPulses[MOTOR_LEFT], fwdPulses[MOTOR_RIGHT]);
  printf("[Calibration] Rev pulses: L=%d R=%d\n", revPulses[MOTOR_LEFT], revPulses[MOTOR_RIGHT]);
  printf("[Calibration] Trim L: fwd=%.3f rev=%.3f\n", 
         m_motorState[MOTOR_LEFT].trimFwd, m_motorState[MOTOR_LEFT].trimRev);
  printf("[Calibration] Trim R: fwd=%.3f rev=%.3f\n", 
         m_motorState[MOTOR_RIGHT].trimFwd, m_motorState[MOTOR_RIGHT].trimRev);
#endif

#else
  /* Encoders not enabled - use default trim */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    m_motorState[i].trimFwd = DEFAULT_TRIM;
    m_motorState[i].trimRev = DEFAULT_TRIM;
  }
#endif /* ENABLE_ENCODER_CALIBRATION */
}

bool DriveTrainDifferential::isInitialized(void) const
{
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    if (!m_motorState[i].initialized)
    {
      return false;
    }
  }

  return true;
}

void DriveTrainDifferential::measureMotorPulses(bool forward, int motorValue, int* pPulses)
{
#if ENABLE_ENCODER_CALIBRATION && !defined(UNIT_TEST)
  if (pPulses == nullptr)
  {
    ERROR_REPORT(ERROR_NULL_POINTER);
    return;
  }

  /* Set motor direction and speed */
  int value = forward ? motorValue : -motorValue;
  
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    gpio_set_irq_enabled(m_motorState[i].pinEncoder, GPIO_IRQ_EDGE_RISE, false);
    MotorDriver::MotorChannel channel = 
      (i == MOTOR_LEFT) ? MotorDriver::MOTOR_A : MotorDriver::MOTOR_B;
    (void)m_motorDriver.setMotor(channel, value);
  }

  /* Let motors reach steady state */
  sleep_ms(MOTOR_SETTLE_TIME_MS);

  /* Clear counters and enable interrupts */
  g_encoderLeftCount = 0;
  g_encoderRightCount = 0;

  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    gpio_acknowledge_irq(m_motorState[i].pinEncoder, GPIO_IRQ_EDGE_RISE);
    gpio_set_irq_enabled(m_motorState[i].pinEncoder, GPIO_IRQ_EDGE_RISE, true);
  }

  /* Measure for calibration period */
  sleep_ms(MOTOR_CALIBRATION_TIME_MS);

  /* Disable interrupts and capture counts */
  for (int i = MOTOR_COUNT - 1; i >= 0; i--)
  {
    gpio_set_irq_enabled(m_motorState[i].pinEncoder, GPIO_IRQ_EDGE_RISE, false);
  }

  pPulses[MOTOR_LEFT] = g_encoderLeftCount;
  pPulses[MOTOR_RIGHT] = g_encoderRightCount;
#else
  (void)forward;
  (void)motorValue;
  if (pPulses != nullptr)
  {
    pPulses[MOTOR_LEFT]  = 0;
    pPulses[MOTOR_RIGHT] = 0;
  }
#endif /* ENABLE_ENCODER_CALIBRATION */
}

/* EOF -----------------------------------------------------------------------*/

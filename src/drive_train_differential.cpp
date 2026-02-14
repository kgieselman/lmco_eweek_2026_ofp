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
#include <cmath>
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
  : m_motorDriver(MOTOR_DRIVER_MODE)
  , m_speed(0)
  , m_turn(0)
  , m_steerRate(STEER_RATE_MAX)
  , m_useManualTrim(true)  /* Default to manual trim mode */
{
  /* Initialize motor state */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    m_motorState[i].initialized  = false;
    m_motorState[i].pinEncoder   = PIN_INVALID;
    m_motorState[i].trimFwd      = DEFAULT_TRIM;
    m_motorState[i].trimRev      = DEFAULT_TRIM;
    m_motorState[i].calibTrimFwd = DEFAULT_TRIM;
    m_motorState[i].calibTrimRev = DEFAULT_TRIM;
    m_lastMotorOutput[i]         = 0;
  }
}

DriveTrainDifferential::~DriveTrainDifferential()
{
  stop();
}

bool DriveTrainDifferential::validateUserInput(int value) const
{
  return (value >= USER_INPUT_MIN) && (value <= USER_INPUT_MAX);
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

#if MOTOR_DRIVER_MODE_2PWM
bool DriveTrainDifferential::addMotor(MotorId_e motor,
                                      int       pinFwd,
                                      int       pinRev,
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
  if (!m_motorDriver.configureMotor(channel, pinFwd, pinRev, pinEncoder))
  {
    ERROR_REPORT(ERROR_DT_NOT_INIT);
    return false;
  }

  /* Configure encoder using helper */
  configureEncoder(motor, pinEncoder);

  /* Reset trim values and mark initialized */
  m_motorState[motor].trimFwd      = DEFAULT_TRIM;
  m_motorState[motor].trimRev      = DEFAULT_TRIM;
  m_motorState[motor].calibTrimFwd = DEFAULT_TRIM;
  m_motorState[motor].calibTrimRev = DEFAULT_TRIM;
  m_motorState[motor].initialized  = true;

#if ENABLE_DEBUG
  printf("[DriveTrain] Motor %d configured (2-PWM): FWD=%d, REV=%d\n",
         motor, pinFwd, pinRev);
#endif

  return true;
}

#elif MOTOR_DRIVER_MODE_1PWM_2DIR

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
    ERROR_REPORT(ERROR_DT_NOT_INIT);
    return false;
  }

  /* Configure encoder using helper */
  configureEncoder(motor, pinEncoder);

  /* Reset trim values and mark initialized */
  m_motorState[motor].trimFwd      = DEFAULT_TRIM;
  m_motorState[motor].trimRev      = DEFAULT_TRIM;
  m_motorState[motor].calibTrimFwd = DEFAULT_TRIM;
  m_motorState[motor].calibTrimRev = DEFAULT_TRIM;
  m_motorState[motor].initialized  = true;

#if ENABLE_DEBUG
  printf("[DriveTrain] Motor %d configured (1-PWM+2-DIR): PWM=%d, FWD=%d, REV=%d\n",
         motor, pinPwm, pinDirFwd, pinDirRev);
#endif

  return true;
}
#endif /* MOTOR_DRIVER_MODE selection */

bool DriveTrainDifferential::setSpeed(int speed)
{
  if (!validateUserInput(speed))
  {
    ERROR_REPORT(ERROR_OUT_OF_RANGE);
    return false;
  }

  m_speed = speed;
  return true;
}

bool DriveTrainDifferential::setTurn(int turn)
{
  if (!validateUserInput(turn))
  {
    ERROR_REPORT(ERROR_OUT_OF_RANGE);
    return false;
  }

  m_turn = turn;
  return true;
}

void DriveTrainDifferential::setSteerRate(int rate)
{
  if (rate < 0)
  {
    rate = 0;
  }
  else if (rate > STEER_RATE_MAX)
  {
    rate = STEER_RATE_MAX;
  }

  m_steerRate = rate;
}

void DriveTrainDifferential::update(void)
{
  /* Verify all motors are initialized */
  if (!isInitialized())
  {
    ERROR_REPORT(ERROR_DT_NOT_INIT);
    return;
  }

  /* Apply steer rate to turn value */
  float steerMultiplier = static_cast<float>(m_steerRate) / static_cast<float>(STEER_RATE_MAX);
  int adjustedTurn = static_cast<int>(static_cast<float>(m_turn) * steerMultiplier);

  /* Calculate raw motor values */
  int motorValues[MOTOR_COUNT];
  motorValues[MOTOR_LEFT]  = m_speed + adjustedTurn;
  motorValues[MOTOR_RIGHT] = m_speed - adjustedTurn;

  /* Scale to prevent clipping while maintaining direction ratio */
  int maxAbs = std::max(std::abs(motorValues[MOTOR_LEFT]),
                        std::abs(motorValues[MOTOR_RIGHT]));

  if (maxAbs > USER_INPUT_MAX)
  {
    float scale = static_cast<float>(USER_INPUT_MAX) / static_cast<float>(maxAbs);
    motorValues[MOTOR_LEFT]  = static_cast<int>(motorValues[MOTOR_LEFT] * scale);
    motorValues[MOTOR_RIGHT] = static_cast<int>(motorValues[MOTOR_RIGHT] * scale);
  }

  /* Store computed values for external readback */
  m_lastMotorOutput[MOTOR_LEFT]  = motorValues[MOTOR_LEFT];
  m_lastMotorOutput[MOTOR_RIGHT] = motorValues[MOTOR_RIGHT];

  /* Apply trim and output to motors */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    float trim = (motorValues[i] > 0) ? m_motorState[i].trimFwd : m_motorState[i].trimRev;

    MotorDriver::MotorChannel_e channel = (i == MOTOR_LEFT)
      ? MotorDriver::MOTOR_A
      : MotorDriver::MOTOR_B;

    (void)m_motorDriver.setMotorWithTrim(channel, motorValues[i], trim);
  }
}

void DriveTrainDifferential::stop(void)
{
  m_speed = 0;
  m_turn = 0;

  m_lastMotorOutput[MOTOR_LEFT]  = 0;
  m_lastMotorOutput[MOTOR_RIGHT] = 0;

  m_motorDriver.stopAll();
}

int DriveTrainDifferential::getMotorOutput(MotorId_e motor) const
{
  if (motor < MOTOR_LEFT || motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return m_lastMotorOutput[motor];
}

int DriveTrainDifferential::getMotorOutputPct(MotorId_e motor) const
{
  if (motor < MOTOR_LEFT || motor >= MOTOR_COUNT)
  {
    return 0;
  }

  /* Convert [-500..+500] to [-100..+100] */
  return (m_lastMotorOutput[motor] * 100) / USER_INPUT_MAX;
}

int DriveTrainDifferential::getForwardTrimOffset(void) const
{
  /*
   * Trim floats are in range [0.5, 1.0] where 1.0 = no reduction.
   * If left trim < right trim, left motor is reduced → robot veers left → positive offset.
   * If right trim < left trim, right motor is reduced → robot veers right → negative offset.
   *
   * Offset = (rightReduction - leftReduction) * 100
   * Where reduction = (1.0 - trim) in range [0.0, 0.5]
   * This gives a range of [-50, +50].
   */
  float leftReduction  = DEFAULT_TRIM - m_motorState[MOTOR_LEFT].trimFwd;
  float rightReduction = DEFAULT_TRIM - m_motorState[MOTOR_RIGHT].trimFwd;

  return static_cast<int>((leftReduction - rightReduction) * 100.0f);
}

int DriveTrainDifferential::getReverseTrimOffset(void) const
{
  float leftReduction  = DEFAULT_TRIM - m_motorState[MOTOR_LEFT].trimRev;
  float rightReduction = DEFAULT_TRIM - m_motorState[MOTOR_RIGHT].trimRev;

  return static_cast<int>((leftReduction - rightReduction) * 100.0f);
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
      m_motorState[i].calibTrimFwd = DEFAULT_TRIM;
      m_motorState[i].calibTrimRev = DEFAULT_TRIM;
    }

    /* Apply calibrated values if in calibrated mode */
    if (!m_useManualTrim)
    {
      for (int i = 0; i < MOTOR_COUNT; i++)
      {
        m_motorState[i].trimFwd = m_motorState[i].calibTrimFwd;
        m_motorState[i].trimRev = m_motorState[i].calibTrimRev;
      }
    }
    return;
  }

  /* Stop motors and wait for settle */
  stop();
  sleep_ms(TIMING_MOTOR_SETTLE_MS);

  /* Measure forward direction at 50% speed */
  int fwdPulses[MOTOR_COUNT] = {0};
  measureMotorPulses(true, USER_INPUT_MAX / 2, fwdPulses);

  stop();
  sleep_ms(TIMING_MOTOR_SETTLE_MS);

  /* Measure reverse direction */
  int revPulses[MOTOR_COUNT] = {0};
  measureMotorPulses(false, USER_INPUT_MAX / 2, revPulses);

  stop();

  /* Calculate trim values based on slower motor */
  int minFwd = std::min(fwdPulses[MOTOR_LEFT], fwdPulses[MOTOR_RIGHT]);
  int minRev = std::min(revPulses[MOTOR_LEFT], revPulses[MOTOR_RIGHT]);

  /* Calculate and store forward calibrated trim */
  if (minFwd > 0)
  {
    m_motorState[MOTOR_LEFT].calibTrimFwd =
      static_cast<float>(minFwd) / static_cast<float>(fwdPulses[MOTOR_LEFT]);
    m_motorState[MOTOR_RIGHT].calibTrimFwd =
      static_cast<float>(minFwd) / static_cast<float>(fwdPulses[MOTOR_RIGHT]);
  }
  else
  {
    m_motorState[MOTOR_LEFT].calibTrimFwd  = DEFAULT_TRIM;
    m_motorState[MOTOR_RIGHT].calibTrimFwd = DEFAULT_TRIM;
  }

  /* Calculate and store reverse calibrated trim */
  if (minRev > 0)
  {
    m_motorState[MOTOR_LEFT].calibTrimRev =
      static_cast<float>(minRev) / static_cast<float>(revPulses[MOTOR_LEFT]);
    m_motorState[MOTOR_RIGHT].calibTrimRev =
      static_cast<float>(minRev) / static_cast<float>(revPulses[MOTOR_RIGHT]);
  }
  else
  {
    m_motorState[MOTOR_LEFT].calibTrimRev  = DEFAULT_TRIM;
    m_motorState[MOTOR_RIGHT].calibTrimRev = DEFAULT_TRIM;
  }

  /* Apply calibrated values if in calibrated mode */
  if (!m_useManualTrim)
  {
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
      m_motorState[i].trimFwd = m_motorState[i].calibTrimFwd;
      m_motorState[i].trimRev = m_motorState[i].calibTrimRev;
    }
  }

#if ENABLE_DEBUG
  printf("[Calibration] Fwd pulses: L=%d R=%d\n", fwdPulses[MOTOR_LEFT], fwdPulses[MOTOR_RIGHT]);
  printf("[Calibration] Rev pulses: L=%d R=%d\n", revPulses[MOTOR_LEFT], revPulses[MOTOR_RIGHT]);
  printf("[Calibration] Calib Trim L: fwd=%.3f rev=%.3f\n",
         m_motorState[MOTOR_LEFT].calibTrimFwd, m_motorState[MOTOR_LEFT].calibTrimRev);
  printf("[Calibration] Calib Trim R: fwd=%.3f rev=%.3f\n",
         m_motorState[MOTOR_RIGHT].calibTrimFwd, m_motorState[MOTOR_RIGHT].calibTrimRev);
#endif

#else
  /* Encoders not enabled - use default trim */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    m_motorState[i].calibTrimFwd = DEFAULT_TRIM;
    m_motorState[i].calibTrimRev = DEFAULT_TRIM;
  }

  /* Apply calibrated values if in calibrated mode */
  if (!m_useManualTrim)
  {
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
      m_motorState[i].trimFwd = m_motorState[i].calibTrimFwd;
      m_motorState[i].trimRev = m_motorState[i].calibTrimRev;
    }
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
    MotorDriver::MotorChannel_e channel =
      (i == MOTOR_LEFT) ? MotorDriver::MOTOR_A : MotorDriver::MOTOR_B;
    (void)m_motorDriver.setMotor(channel, value);
  }

  /* Let motors reach steady state */
  sleep_ms(TIMING_MOTOR_SETTLE_MS);

  /* Clear counters and enable interrupts */
  g_encoderLeftCount = 0;
  g_encoderRightCount = 0;

  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    gpio_acknowledge_irq(m_motorState[i].pinEncoder, GPIO_IRQ_EDGE_RISE);
    gpio_set_irq_enabled(m_motorState[i].pinEncoder, GPIO_IRQ_EDGE_RISE, true);
  }

  /* Measure for calibration period */
  sleep_ms(TIMING_MOTOR_CALIBRATION_MS);

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

void DriveTrainDifferential::channelToTrim(int channelValue,
                                           float* pLeftTrim,
                                           float* pRightTrim) const
{
  if ((pLeftTrim == nullptr) || (pRightTrim == nullptr))
  {
    return;
  }

  /* Clamp channel value to valid range */
  if (channelValue < IBUS_CHANNEL_MIN)
  {
    channelValue = IBUS_CHANNEL_MIN;
  }
  else if (channelValue > IBUS_CHANNEL_MAX)
  {
    channelValue = IBUS_CHANNEL_MAX;
  }

  /* Calculate offset from center (-500 to +500) */
  int offset = channelValue - IBUS_CHANNEL_CENTER;

  /* Convert offset to trim value
   * - Positive offset (>1500): reduce left motor power (right is weaker)
   * - Negative offset (<1500): reduce right motor power (left is weaker)
   * - Zero offset (1500): no trim applied
   *
   * Trim range is MIN_TRIM to DEFAULT_TRIM (0.5 to 1.0)
   * At max offset (500), the reduced motor gets MIN_TRIM
   */
  float trimRange = DEFAULT_TRIM - MIN_TRIM;  /* 0.5 */
  float offsetNormalized = static_cast<float>(std::abs(offset)) / 500.0f;

  if (offset > 0)
  {
    /* Reduce left motor (right is weaker, so left needs to slow down) */
    *pLeftTrim = DEFAULT_TRIM - (trimRange * offsetNormalized);
    *pRightTrim = DEFAULT_TRIM;
  }
  else if (offset < 0)
  {
    /* Reduce right motor (left is weaker, so right needs to slow down) */
    *pLeftTrim = DEFAULT_TRIM;
    *pRightTrim = DEFAULT_TRIM - (trimRange * offsetNormalized);
  }
  else
  {
    /* No trim */
    *pLeftTrim = DEFAULT_TRIM;
    *pRightTrim = DEFAULT_TRIM;
  }
}

void DriveTrainDifferential::setForwardTrimFromChannel(int channelValue)
{
  /* Only apply manual trim when in manual mode */
  if (!m_useManualTrim)
  {
    return;
  }

  float leftTrim  = DEFAULT_TRIM;
  float rightTrim = DEFAULT_TRIM;
  channelToTrim(channelValue, &leftTrim, &rightTrim);

  m_motorState[MOTOR_LEFT].trimFwd = leftTrim;
  m_motorState[MOTOR_RIGHT].trimFwd = rightTrim;
}

void DriveTrainDifferential::setReverseTrimFromChannel(int channelValue)
{
  /* Only apply manual trim when in manual mode */
  if (!m_useManualTrim)
  {
    return;
  }

  float leftTrim, rightTrim;
  channelToTrim(channelValue, &leftTrim, &rightTrim);

  m_motorState[MOTOR_LEFT].trimRev = leftTrim;
  m_motorState[MOTOR_RIGHT].trimRev = rightTrim;
}

void DriveTrainDifferential::getForwardTrim(float* pLeftTrim, float* pRightTrim) const
{
  if (pLeftTrim != nullptr)
  {
    *pLeftTrim = m_motorState[MOTOR_LEFT].trimFwd;
  }
  if (pRightTrim != nullptr)
  {
    *pRightTrim = m_motorState[MOTOR_RIGHT].trimFwd;
  }
}

void DriveTrainDifferential::getReverseTrim(float* pLeftTrim, float* pRightTrim) const
{
  if (pLeftTrim != nullptr)
  {
    *pLeftTrim = m_motorState[MOTOR_LEFT].trimRev;
  }
  if (pRightTrim != nullptr)
  {
    *pRightTrim = m_motorState[MOTOR_RIGHT].trimRev;
  }
}

void DriveTrainDifferential::setManualTrimMode(bool useManual)
{
  /* Only take action if mode is actually changing */
  if (useManual == m_useManualTrim)
  {
    return;
  }

  m_useManualTrim = useManual;

  if (!useManual)
  {
    /* Switching to calibrated mode - restore calibrated trim values */
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
      m_motorState[i].trimFwd = m_motorState[i].calibTrimFwd;
      m_motorState[i].trimRev = m_motorState[i].calibTrimRev;
    }

#if ENABLE_DEBUG
    printf("[DriveTrain] Switched to CALIBRATED trim mode\n");
    printf("[DriveTrain] Trim L: fwd=%.3f rev=%.3f\n",
           m_motorState[MOTOR_LEFT].trimFwd, m_motorState[MOTOR_LEFT].trimRev);
    printf("[DriveTrain] Trim R: fwd=%.3f rev=%.3f\n",
           m_motorState[MOTOR_RIGHT].trimFwd, m_motorState[MOTOR_RIGHT].trimRev);
#endif
  }
  else
  {
#if ENABLE_DEBUG
    printf("[DriveTrain] Switched to MANUAL trim mode\n");
#endif
    /* When switching to manual mode, keep current trim values until
     * explicitly changed by setForwardTrimFromChannel/setReverseTrimFromChannel */
  }
}

bool DriveTrainDifferential::isManualTrimMode(void) const
{
  return m_useManualTrim;
}


/* EOF -----------------------------------------------------------------------*/

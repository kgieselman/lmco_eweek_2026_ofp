/*******************************************************************************
 * @file test_motor_driver.cpp
 * @brief Unit tests for MotorDriver (1-PWM + 2-DIR H-bridge driver)
 *
 * Tests pin validation, configuration, motor value clamping, trim,
 * stop modes, and state tracking for the unified MotorDriver class.
 *
 * Hardware calls (PWM, GPIO) are stubbed so the tests run on the host.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#define UNIT_TEST
#include "unit_test.h"

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <algorithm>


/*============================================================================*/
/* Pico SDK Stubs                                                             */
/*============================================================================*/

/* Minimal stubs so motor_driver.cpp compiles on the host. */

struct pwm_config { uint32_t div; uint32_t top; uint32_t csr; };

static uint16_t g_stub_pwm_level[30] = {0};
static bool     g_stub_gpio_state[30] = {false};
static bool     g_stub_gpio_initialized[30] = {false};
static bool     g_stub_pwm_initialized[30] = {false};

static void stub_reset(void)
{
  memset(g_stub_pwm_level, 0, sizeof(g_stub_pwm_level));
  memset(g_stub_gpio_state, 0, sizeof(g_stub_gpio_state));
  memset(g_stub_gpio_initialized, 0, sizeof(g_stub_gpio_initialized));
  memset(g_stub_pwm_initialized, 0, sizeof(g_stub_pwm_initialized));
}

/* GPIO stubs */
extern "C" {
void gpio_init(unsigned int pin)            { if (pin < 30) g_stub_gpio_initialized[pin] = true; }
void gpio_set_dir(unsigned int, bool)       {}
void gpio_put(unsigned int pin, bool val)   { if (pin < 30) g_stub_gpio_state[pin] = val; }
void gpio_set_function(unsigned int, int)   {}

/* PWM stubs */
unsigned int pwm_gpio_to_slice_num(unsigned int pin)  { return pin / 2; }
unsigned int pwm_gpio_to_channel(unsigned int pin)    { return pin & 1; }
pwm_config   pwm_get_default_config(void)             { pwm_config c = {0,0,0}; return c; }
void pwm_config_set_clkdiv(pwm_config*, float)        {}
void pwm_config_set_wrap(pwm_config*, uint16_t)       {}
void pwm_init(unsigned int, pwm_config*, bool)         {}
void pwm_set_chan_level(unsigned int slice, unsigned int chan, uint16_t level)
{
  unsigned int pin = slice * 2 + chan;
  if (pin < 30) { g_stub_pwm_level[pin] = level; g_stub_pwm_initialized[pin] = true; }
}
void pwm_set_enabled(unsigned int, bool) {}
} /* extern "C" */


/*============================================================================*/
/* Include the implementation under test                                      */
/*============================================================================*/

/* Provide the error_report stub expected by motor_driver.cpp */
#include "../inc/error_handler.h"
extern "C" void error_report(ErrorCode_t code, const char* file, int line)
{
  (void)code; (void)file; (void)line;
}

#include "../src/motor_driver.cpp"


/*============================================================================*/
/* Configuration Tests                                                        */
/*============================================================================*/

TEST_FUNC(configure_single_motor)
{
  stub_reset();
  MotorDriver drv;
  ASSERT_FALSE(drv.isConfigured(MotorDriver::MOTOR_A));
  ASSERT_TRUE(drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10));
  ASSERT_TRUE(drv.isConfigured(MotorDriver::MOTOR_A));
  ASSERT_FALSE(drv.isConfigured(MotorDriver::MOTOR_B));
  ASSERT_FALSE(drv.isFullyConfigured());
}

TEST_FUNC(configure_both_motors)
{
  stub_reset();
  MotorDriver drv;
  ASSERT_TRUE(drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10));
  ASSERT_TRUE(drv.configureMotor(MotorDriver::MOTOR_B, 11, 12, 13));
  ASSERT_TRUE(drv.isFullyConfigured());
}

TEST_FUNC(configure_with_encoder)
{
  stub_reset();
  MotorDriver drv;
  ASSERT_TRUE(drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10, 14));
  ASSERT_EQUAL(14, drv.getEncoderPin(MotorDriver::MOTOR_A));
}

TEST_FUNC(configure_without_encoder)
{
  stub_reset();
  MotorDriver drv;
  ASSERT_TRUE(drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10));
  ASSERT_EQUAL(PIN_INVALID, drv.getEncoderPin(MotorDriver::MOTOR_A));
}

TEST_FUNC(configure_invalid_pwm_pin)
{
  stub_reset();
  MotorDriver drv;
  ASSERT_FALSE(drv.configureMotor(MotorDriver::MOTOR_A, -1, 9, 10));
  ASSERT_FALSE(drv.configureMotor(MotorDriver::MOTOR_A, 30, 9, 10));
  ASSERT_FALSE(drv.isConfigured(MotorDriver::MOTOR_A));
}

TEST_FUNC(configure_invalid_dir_fwd_pin)
{
  stub_reset();
  MotorDriver drv;
  ASSERT_FALSE(drv.configureMotor(MotorDriver::MOTOR_A, 8, -1, 10));
  ASSERT_FALSE(drv.configureMotor(MotorDriver::MOTOR_A, 8, 30, 10));
}

TEST_FUNC(configure_invalid_dir_rev_pin)
{
  stub_reset();
  MotorDriver drv;
  ASSERT_FALSE(drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, -1));
  ASSERT_FALSE(drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 30));
}

TEST_FUNC(configure_invalid_encoder_pin)
{
  stub_reset();
  MotorDriver drv;
  ASSERT_FALSE(drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10, 30));
  ASSERT_FALSE(drv.isConfigured(MotorDriver::MOTOR_A));
}

TEST_FUNC(configure_invalid_channel)
{
  stub_reset();
  MotorDriver drv;
  auto bad = static_cast<MotorDriver::MotorChannel_e>(5);
  ASSERT_FALSE(drv.isConfigured(bad));
  ASSERT_EQUAL(PIN_INVALID, drv.getEncoderPin(bad));
  ASSERT_FALSE(drv.setMotor(bad, 250));
}

TEST_FUNC(reconfigure_motor)
{
  stub_reset();
  MotorDriver drv;
  ASSERT_TRUE(drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10));
  ASSERT_TRUE(drv.configureMotor(MotorDriver::MOTOR_A, 11, 12, 13));
  ASSERT_TRUE(drv.isConfigured(MotorDriver::MOTOR_A));
}


/*============================================================================*/
/* Set Motor Tests                                                            */
/*============================================================================*/

TEST_FUNC(set_motor_forward)
{
  stub_reset();
  MotorDriver drv;
  drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10);
  ASSERT_TRUE(drv.setMotor(MotorDriver::MOTOR_A, 500));
  ASSERT_EQUAL(500, drv.getCurrentValue(MotorDriver::MOTOR_A));
}

TEST_FUNC(set_motor_reverse)
{
  stub_reset();
  MotorDriver drv;
  drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10);
  ASSERT_TRUE(drv.setMotor(MotorDriver::MOTOR_A, -500));
  ASSERT_EQUAL(-500, drv.getCurrentValue(MotorDriver::MOTOR_A));
}

TEST_FUNC(set_motor_zero)
{
  stub_reset();
  MotorDriver drv;
  drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10);
  drv.setMotor(MotorDriver::MOTOR_A, 500);
  ASSERT_TRUE(drv.setMotor(MotorDriver::MOTOR_A, 0));
  ASSERT_EQUAL(0, drv.getCurrentValue(MotorDriver::MOTOR_A));
}

TEST_FUNC(set_motor_unconfigured_fails)
{
  stub_reset();
  MotorDriver drv;
  ASSERT_FALSE(drv.setMotor(MotorDriver::MOTOR_A, 250));
}

TEST_FUNC(set_motor_full_range)
{
  stub_reset();
  MotorDriver drv;
  drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10);
  ASSERT_TRUE(drv.setMotor(MotorDriver::MOTOR_A, 1000));
  ASSERT_TRUE(drv.setMotor(MotorDriver::MOTOR_A, -1000));
  ASSERT_TRUE(drv.setMotor(MotorDriver::MOTOR_A, 1));
  ASSERT_TRUE(drv.setMotor(MotorDriver::MOTOR_A, -1));
}


/*============================================================================*/
/* Clamping Tests                                                             */
/*============================================================================*/

TEST_FUNC(clamp_above_max)
{
  stub_reset();
  MotorDriver drv;
  drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10);
  /* Values beyond +-1000 should be clamped, not rejected */
  ASSERT_TRUE(drv.setMotor(MotorDriver::MOTOR_A, 2000));
  ASSERT_TRUE(drv.setMotor(MotorDriver::MOTOR_A, -2000));
}

TEST_FUNC(clamp_static_constexpr)
{
  /* Verify the constexpr clamp helper directly */
  /* We can't call the private static directly, so test via boundaries */
  stub_reset();
  MotorDriver drv;
  drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10);
  ASSERT_TRUE(drv.setMotor(MotorDriver::MOTOR_A, MotorDriver::MOTOR_VALUE_MAX));
  ASSERT_TRUE(drv.setMotor(MotorDriver::MOTOR_A, MotorDriver::MOTOR_VALUE_MIN));
}


/*============================================================================*/
/* Trim Tests                                                                 */
/*============================================================================*/

TEST_FUNC(trim_full)
{
  stub_reset();
  MotorDriver drv;
  drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10);
  ASSERT_TRUE(drv.setMotorWithTrim(MotorDriver::MOTOR_A, 500, 1.0f));
}

TEST_FUNC(trim_half)
{
  stub_reset();
  MotorDriver drv;
  drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10);
  ASSERT_TRUE(drv.setMotorWithTrim(MotorDriver::MOTOR_A, 500, 0.5f));
}

TEST_FUNC(trim_zero)
{
  stub_reset();
  MotorDriver drv;
  drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10);
  ASSERT_TRUE(drv.setMotorWithTrim(MotorDriver::MOTOR_A, 500, 0.0f));
}

TEST_FUNC(trim_clamped_over)
{
  stub_reset();
  MotorDriver drv;
  drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10);
  /* Trim > 1.0 should be accepted (clamped internally) */
  ASSERT_TRUE(drv.setMotorWithTrim(MotorDriver::MOTOR_A, 500, 1.5f));
}

TEST_FUNC(trim_clamped_under)
{
  stub_reset();
  MotorDriver drv;
  drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10);
  /* Trim < 0.0 should be accepted (clamped internally) */
  ASSERT_TRUE(drv.setMotorWithTrim(MotorDriver::MOTOR_A, 500, -0.5f));
}

TEST_FUNC(trim_unconfigured_fails)
{
  stub_reset();
  MotorDriver drv;
  ASSERT_FALSE(drv.setMotorWithTrim(MotorDriver::MOTOR_A, 500, 1.0f));
}


/*============================================================================*/
/* Stop Mode Tests                                                            */
/*============================================================================*/

TEST_FUNC(coast_configured)
{
  stub_reset();
  MotorDriver drv;
  drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10);
  drv.setMotor(MotorDriver::MOTOR_A, 500);
  drv.coast(MotorDriver::MOTOR_A);
  /* After coast, current value should track to 0 */
  ASSERT_EQUAL(0, drv.getCurrentValue(MotorDriver::MOTOR_A));
}

TEST_FUNC(brake_configured)
{
  stub_reset();
  MotorDriver drv;
  drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10);
  drv.setMotor(MotorDriver::MOTOR_A, 500);
  drv.brake(MotorDriver::MOTOR_A);
  ASSERT_EQUAL(0, drv.getCurrentValue(MotorDriver::MOTOR_A));
}

TEST_FUNC(stop_coast_mode)
{
  stub_reset();
  MotorDriver drv;
  drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10);
  drv.setMotor(MotorDriver::MOTOR_A, 300);
  drv.stop(MotorDriver::MOTOR_A, MotorDriver::STOP_COAST);
  ASSERT_EQUAL(0, drv.getCurrentValue(MotorDriver::MOTOR_A));
}

TEST_FUNC(stop_brake_mode)
{
  stub_reset();
  MotorDriver drv;
  drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10);
  drv.setMotor(MotorDriver::MOTOR_A, 300);
  drv.stop(MotorDriver::MOTOR_A, MotorDriver::STOP_BRAKE);
  ASSERT_EQUAL(0, drv.getCurrentValue(MotorDriver::MOTOR_A));
}

TEST_FUNC(stop_default_mode)
{
  stub_reset();
  MotorDriver drv;
  drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10);
  drv.setMotor(MotorDriver::MOTOR_A, 300);
  drv.stop(MotorDriver::MOTOR_A);  /* defaults to STOP_COAST */
  ASSERT_EQUAL(0, drv.getCurrentValue(MotorDriver::MOTOR_A));
}

TEST_FUNC(stop_all_motors)
{
  stub_reset();
  MotorDriver drv;
  drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10);
  drv.configureMotor(MotorDriver::MOTOR_B, 11, 12, 13);
  drv.setMotor(MotorDriver::MOTOR_A, 500);
  drv.setMotor(MotorDriver::MOTOR_B, -500);
  drv.stopAll(MotorDriver::STOP_COAST);
  ASSERT_EQUAL(0, drv.getCurrentValue(MotorDriver::MOTOR_A));
  ASSERT_EQUAL(0, drv.getCurrentValue(MotorDriver::MOTOR_B));
}

TEST_FUNC(stop_all_brake)
{
  stub_reset();
  MotorDriver drv;
  drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10);
  drv.configureMotor(MotorDriver::MOTOR_B, 11, 12, 13);
  drv.setMotor(MotorDriver::MOTOR_A, 500);
  drv.setMotor(MotorDriver::MOTOR_B, -500);
  drv.stopAll(MotorDriver::STOP_BRAKE);
  ASSERT_EQUAL(0, drv.getCurrentValue(MotorDriver::MOTOR_A));
  ASSERT_EQUAL(0, drv.getCurrentValue(MotorDriver::MOTOR_B));
}


/*============================================================================*/
/* Default Stop Mode Tests                                                    */
/*============================================================================*/

TEST_FUNC(default_stop_mode_is_coast)
{
  stub_reset();
  MotorDriver drv;
  ASSERT_EQUAL((int)MotorDriver::STOP_COAST, (int)drv.getDefaultStopMode());
}

TEST_FUNC(set_default_stop_mode_brake)
{
  stub_reset();
  MotorDriver drv;
  drv.setDefaultStopMode(MotorDriver::STOP_BRAKE);
  ASSERT_EQUAL((int)MotorDriver::STOP_BRAKE, (int)drv.getDefaultStopMode());
}

TEST_FUNC(set_default_stop_mode_back_to_coast)
{
  stub_reset();
  MotorDriver drv;
  drv.setDefaultStopMode(MotorDriver::STOP_BRAKE);
  drv.setDefaultStopMode(MotorDriver::STOP_COAST);
  ASSERT_EQUAL((int)MotorDriver::STOP_COAST, (int)drv.getDefaultStopMode());
}


/*============================================================================*/
/* Custom PWM Frequency Tests                                                 */
/*============================================================================*/

TEST_FUNC(custom_pwm_frequency)
{
  stub_reset();
  MotorDriver drv10k(10000);
  ASSERT_TRUE(drv10k.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10));

  MotorDriver drv50k(50000);
  ASSERT_TRUE(drv50k.configureMotor(MotorDriver::MOTOR_A, 11, 12, 13));
}


/*============================================================================*/
/* getCurrentValue Tests                                                      */
/*============================================================================*/

TEST_FUNC(current_value_invalid_channel)
{
  stub_reset();
  MotorDriver drv;
  auto bad = static_cast<MotorDriver::MotorChannel_e>(5);
  ASSERT_EQUAL(0, drv.getCurrentValue(bad));
}

TEST_FUNC(current_value_tracks_setMotor)
{
  stub_reset();
  MotorDriver drv;
  drv.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10);
  drv.setMotor(MotorDriver::MOTOR_A, 750);
  ASSERT_EQUAL(750, drv.getCurrentValue(MotorDriver::MOTOR_A));
  drv.setMotor(MotorDriver::MOTOR_A, -300);
  ASSERT_EQUAL(-300, drv.getCurrentValue(MotorDriver::MOTOR_A));
}


/*============================================================================*/
/* Main                                                                       */
/*============================================================================*/

int main(void)
{
  printf("\n=== MotorDriver Unit Tests ===\n");

  SUITE_START("Configuration");
  RUN_TEST(configure_single_motor);
  RUN_TEST(configure_both_motors);
  RUN_TEST(configure_with_encoder);
  RUN_TEST(configure_without_encoder);
  RUN_TEST(configure_invalid_pwm_pin);
  RUN_TEST(configure_invalid_dir_fwd_pin);
  RUN_TEST(configure_invalid_dir_rev_pin);
  RUN_TEST(configure_invalid_encoder_pin);
  RUN_TEST(configure_invalid_channel);
  RUN_TEST(reconfigure_motor);

  SUITE_START("Set Motor");
  RUN_TEST(set_motor_forward);
  RUN_TEST(set_motor_reverse);
  RUN_TEST(set_motor_zero);
  RUN_TEST(set_motor_unconfigured_fails);
  RUN_TEST(set_motor_full_range);

  SUITE_START("Value Clamping");
  RUN_TEST(clamp_above_max);
  RUN_TEST(clamp_static_constexpr);

  SUITE_START("Trim");
  RUN_TEST(trim_full);
  RUN_TEST(trim_half);
  RUN_TEST(trim_zero);
  RUN_TEST(trim_clamped_over);
  RUN_TEST(trim_clamped_under);
  RUN_TEST(trim_unconfigured_fails);

  SUITE_START("Stop Modes");
  RUN_TEST(coast_configured);
  RUN_TEST(brake_configured);
  RUN_TEST(stop_coast_mode);
  RUN_TEST(stop_brake_mode);
  RUN_TEST(stop_default_mode);
  RUN_TEST(stop_all_motors);
  RUN_TEST(stop_all_brake);

  SUITE_START("Default Stop Mode");
  RUN_TEST(default_stop_mode_is_coast);
  RUN_TEST(set_default_stop_mode_brake);
  RUN_TEST(set_default_stop_mode_back_to_coast);

  SUITE_START("PWM Frequency");
  RUN_TEST(custom_pwm_frequency);

  SUITE_START("getCurrentValue");
  RUN_TEST(current_value_invalid_channel);
  RUN_TEST(current_value_tracks_setMotor);

  PRINT_SUMMARY();

  return (g_tests_failed > 0) ? 1 : 0;
}


/* EOF -----------------------------------------------------------------------*/

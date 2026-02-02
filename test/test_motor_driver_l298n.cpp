/*******************************************************************************
 * @file test_motor_driver_l298n.cpp
 * @brief Unit tests for L298N motor driver
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#define UNIT_TEST
#include "unit_test.h"
#include "../inc/error_handler.h"

/* Mock error_report for unit testing */
extern "C" void error_report(ErrorCode_t code, const char* file, int line) {
  (void)code; (void)file; (void)line;
}

#include "../src/motor_driver_l298n.cpp"

#include <cstdio>


/* Test Cases ----------------------------------------------------------------*/

/*****************************************************************************
 * @brief Test motor configuration
 ****************************************************************************/
void test_configure_motor(void)
{
  printf("test_configure_motor: ");
  
  MotorDriverL298N driver;
  
  /* Test valid configuration (3 pins: PWM, DIR_FWD, DIR_REV) */
  ASSERT_TRUE(driver.configureMotor(MotorDriverL298N::MOTOR_A, 8, 9, 10));
  ASSERT_TRUE(driver.isConfigured(MotorDriverL298N::MOTOR_A));
  ASSERT_FALSE(driver.isConfigured(MotorDriverL298N::MOTOR_B));
  ASSERT_FALSE(driver.isFullyConfigured());
  
  /* Configure second motor */
  ASSERT_TRUE(driver.configureMotor(MotorDriverL298N::MOTOR_B, 11, 12, 13));
  ASSERT_TRUE(driver.isConfigured(MotorDriverL298N::MOTOR_B));
  ASSERT_TRUE(driver.isFullyConfigured());
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test invalid pin configuration
 ****************************************************************************/
void test_configure_invalid_pins(void)
{
  printf("test_configure_invalid_pins: ");
  
  MotorDriverL298N driver;
  
  /* Test invalid PWM pin */
  ASSERT_FALSE(driver.configureMotor(MotorDriverL298N::MOTOR_A, -1, 9, 10));
  ASSERT_FALSE(driver.configureMotor(MotorDriverL298N::MOTOR_A, 30, 9, 10));
  
  /* Test invalid direction forward pin */
  ASSERT_FALSE(driver.configureMotor(MotorDriverL298N::MOTOR_A, 8, -1, 10));
  ASSERT_FALSE(driver.configureMotor(MotorDriverL298N::MOTOR_A, 8, 30, 10));
  
  /* Test invalid direction reverse pin */
  ASSERT_FALSE(driver.configureMotor(MotorDriverL298N::MOTOR_A, 8, 9, -1));
  ASSERT_FALSE(driver.configureMotor(MotorDriverL298N::MOTOR_A, 8, 9, 30));
  
  ASSERT_FALSE(driver.isConfigured(MotorDriverL298N::MOTOR_A));
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test configuration with encoder
 ****************************************************************************/
void test_configure_with_encoder(void)
{
  printf("test_configure_with_encoder: ");
  
  MotorDriverL298N driver;
  
  /* Configure with encoder */
  ASSERT_TRUE(driver.configureMotor(MotorDriverL298N::MOTOR_A, 8, 9, 10, 14));
  ASSERT_EQUAL(driver.getEncoderPin(MotorDriverL298N::MOTOR_A), 14);
  
  /* Configure without encoder */
  ASSERT_TRUE(driver.configureMotor(MotorDriverL298N::MOTOR_B, 11, 12, 13));
  ASSERT_EQUAL(driver.getEncoderPin(MotorDriverL298N::MOTOR_B), -1);
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test invalid encoder pin
 ****************************************************************************/
void test_configure_invalid_encoder(void)
{
  printf("test_configure_invalid_encoder: ");
  
  MotorDriverL298N driver;
  
  /* Test invalid encoder pin */
  ASSERT_FALSE(driver.configureMotor(MotorDriverL298N::MOTOR_A, 8, 9, 10, 30));
  ASSERT_FALSE(driver.isConfigured(MotorDriverL298N::MOTOR_A));
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test set motor forward
 ****************************************************************************/
void test_set_motor_forward(void)
{
  printf("test_set_motor_forward: ");
  
  MotorDriverL298N driver;
  driver.configureMotor(MotorDriverL298N::MOTOR_A, 8, 9, 10);
  
  /* Test forward values */
  ASSERT_TRUE(driver.setMotor(MotorDriverL298N::MOTOR_A, 250));
  ASSERT_TRUE(driver.setMotor(MotorDriverL298N::MOTOR_A, 500));
  ASSERT_TRUE(driver.setMotor(MotorDriverL298N::MOTOR_A, 1));
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test set motor reverse
 ****************************************************************************/
void test_set_motor_reverse(void)
{
  printf("test_set_motor_reverse: ");
  
  MotorDriverL298N driver;
  driver.configureMotor(MotorDriverL298N::MOTOR_A, 8, 9, 10);
  
  /* Test reverse values */
  ASSERT_TRUE(driver.setMotor(MotorDriverL298N::MOTOR_A, -250));
  ASSERT_TRUE(driver.setMotor(MotorDriverL298N::MOTOR_A, -500));
  ASSERT_TRUE(driver.setMotor(MotorDriverL298N::MOTOR_A, -1));
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test set motor zero (stop)
 ****************************************************************************/
void test_set_motor_zero(void)
{
  printf("test_set_motor_zero: ");
  
  MotorDriverL298N driver;
  driver.configureMotor(MotorDriverL298N::MOTOR_A, 8, 9, 10);
  
  /* Set motor running then stop */
  driver.setMotor(MotorDriverL298N::MOTOR_A, 250);
  ASSERT_TRUE(driver.setMotor(MotorDriverL298N::MOTOR_A, 0));
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test value clamping
 ****************************************************************************/
void test_value_clamping(void)
{
  printf("test_value_clamping: ");
  
  MotorDriverL298N driver;
  driver.configureMotor(MotorDriverL298N::MOTOR_A, 8, 9, 10);
  
  /* Values outside range should be clamped, not rejected */
  ASSERT_TRUE(driver.setMotor(MotorDriverL298N::MOTOR_A, 1000));
  ASSERT_TRUE(driver.setMotor(MotorDriverL298N::MOTOR_A, -1000));
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test unconfigured motor
 ****************************************************************************/
void test_unconfigured_motor(void)
{
  printf("test_unconfigured_motor: ");
  
  MotorDriverL298N driver;
  
  /* Should fail on unconfigured motor */
  ASSERT_FALSE(driver.setMotor(MotorDriverL298N::MOTOR_A, 250));
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test motor with trim
 ****************************************************************************/
void test_motor_with_trim(void)
{
  printf("test_motor_with_trim: ");
  
  MotorDriverL298N driver;
  driver.configureMotor(MotorDriverL298N::MOTOR_A, 8, 9, 10);
  
  /* Test various trim values */
  ASSERT_TRUE(driver.setMotorWithTrim(MotorDriverL298N::MOTOR_A, 500, 1.0f));
  ASSERT_TRUE(driver.setMotorWithTrim(MotorDriverL298N::MOTOR_A, 500, 0.5f));
  ASSERT_TRUE(driver.setMotorWithTrim(MotorDriverL298N::MOTOR_A, 500, 0.0f));
  
  /* Trim should be clamped */
  ASSERT_TRUE(driver.setMotorWithTrim(MotorDriverL298N::MOTOR_A, 500, 1.5f));
  ASSERT_TRUE(driver.setMotorWithTrim(MotorDriverL298N::MOTOR_A, 500, -0.5f));
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test coast stop mode
 ****************************************************************************/
void test_coast(void)
{
  printf("test_coast: ");
  
  MotorDriverL298N driver;
  driver.configureMotor(MotorDriverL298N::MOTOR_A, 8, 9, 10);
  
  /* Set motor running then coast */
  driver.setMotor(MotorDriverL298N::MOTOR_A, 250);
  driver.coast(MotorDriverL298N::MOTOR_A);
  
  /* Should not throw on unconfigured */
  driver.coast(MotorDriverL298N::MOTOR_B);
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test brake stop mode
 ****************************************************************************/
void test_brake(void)
{
  printf("test_brake: ");
  
  MotorDriverL298N driver;
  driver.configureMotor(MotorDriverL298N::MOTOR_A, 8, 9, 10);
  
  /* Set motor running then brake */
  driver.setMotor(MotorDriverL298N::MOTOR_A, 250);
  driver.brake(MotorDriverL298N::MOTOR_A);
  
  /* Should not throw on unconfigured */
  driver.brake(MotorDriverL298N::MOTOR_B);
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test stop with mode selection
 ****************************************************************************/
void test_stop_modes(void)
{
  printf("test_stop_modes: ");
  
  MotorDriverL298N driver;
  driver.configureMotor(MotorDriverL298N::MOTOR_A, 8, 9, 10);
  
  /* Test stop with coast mode */
  driver.setMotor(MotorDriverL298N::MOTOR_A, 250);
  driver.stop(MotorDriverL298N::MOTOR_A, MotorDriverL298N::STOP_COAST);
  
  /* Test stop with brake mode */
  driver.setMotor(MotorDriverL298N::MOTOR_A, 250);
  driver.stop(MotorDriverL298N::MOTOR_A, MotorDriverL298N::STOP_BRAKE);
  
  /* Test default stop mode */
  driver.setMotor(MotorDriverL298N::MOTOR_A, 250);
  driver.stop(MotorDriverL298N::MOTOR_A);
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test stop all motors
 ****************************************************************************/
void test_stop_all(void)
{
  printf("test_stop_all: ");
  
  MotorDriverL298N driver;
  driver.configureMotor(MotorDriverL298N::MOTOR_A, 8, 9, 10);
  driver.configureMotor(MotorDriverL298N::MOTOR_B, 11, 12, 13);
  
  /* Set both motors running */
  driver.setMotor(MotorDriverL298N::MOTOR_A, 250);
  driver.setMotor(MotorDriverL298N::MOTOR_B, -250);
  
  /* Stop all with coast */
  driver.stopAll(MotorDriverL298N::STOP_COAST);
  
  /* Stop all with brake */
  driver.setMotor(MotorDriverL298N::MOTOR_A, 250);
  driver.setMotor(MotorDriverL298N::MOTOR_B, -250);
  driver.stopAll(MotorDriverL298N::STOP_BRAKE);
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test default stop mode setting
 ****************************************************************************/
void test_default_stop_mode(void)
{
  printf("test_default_stop_mode: ");
  
  MotorDriverL298N driver;
  
  /* Default should be coast */
  ASSERT_EQUAL(static_cast<int>(driver.getDefaultStopMode()), static_cast<int>(MotorDriverL298N::STOP_COAST));
  
  /* Change to brake */
  driver.setDefaultStopMode(MotorDriverL298N::STOP_BRAKE);
  ASSERT_EQUAL(static_cast<int>(driver.getDefaultStopMode()), static_cast<int>(MotorDriverL298N::STOP_BRAKE));
  
  /* Change back to coast */
  driver.setDefaultStopMode(MotorDriverL298N::STOP_COAST);
  ASSERT_EQUAL(static_cast<int>(driver.getDefaultStopMode()), static_cast<int>(MotorDriverL298N::STOP_COAST));
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test custom PWM frequency
 ****************************************************************************/
void test_custom_pwm_frequency(void)
{
  printf("test_custom_pwm_frequency: ");
  
  /* Test with different frequencies */
  MotorDriverL298N driver1(10000);  /* 10 kHz */
  ASSERT_TRUE(driver1.configureMotor(MotorDriverL298N::MOTOR_A, 8, 9, 10));
  
  MotorDriverL298N driver2(50000);  /* 50 kHz */
  ASSERT_TRUE(driver2.configureMotor(MotorDriverL298N::MOTOR_A, 11, 12, 13));
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test invalid channel
 ****************************************************************************/
void test_invalid_channel(void)
{
  printf("test_invalid_channel: ");
  
  MotorDriverL298N driver;
  
  /* Test invalid channel on various methods */
  ASSERT_FALSE(driver.isConfigured(static_cast<MotorDriverL298N::MotorChannel>(5)));
  ASSERT_EQUAL(driver.getEncoderPin(static_cast<MotorDriverL298N::MotorChannel>(5)), -1);
  ASSERT_FALSE(driver.setMotor(static_cast<MotorDriverL298N::MotorChannel>(5), 250));
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test reconfiguring a motor
 ****************************************************************************/
void test_reconfigure_motor(void)
{
  printf("test_reconfigure_motor: ");
  
  MotorDriverL298N driver;
  
  /* Configure motor */
  ASSERT_TRUE(driver.configureMotor(MotorDriverL298N::MOTOR_A, 8, 9, 10));
  ASSERT_TRUE(driver.isConfigured(MotorDriverL298N::MOTOR_A));
  
  /* Reconfigure with different pins */
  ASSERT_TRUE(driver.configureMotor(MotorDriverL298N::MOTOR_A, 11, 12, 13));
  ASSERT_TRUE(driver.isConfigured(MotorDriverL298N::MOTOR_A));
  
  printf("PASSED\n");
}


/* Main Function -------------------------------------------------------------*/
int main(void)
{
  printf("\n=== L298N Motor Driver Unit Tests ===\n\n");
  
  test_configure_motor();
  test_configure_invalid_pins();
  test_configure_with_encoder();
  test_configure_invalid_encoder();
  test_set_motor_forward();
  test_set_motor_reverse();
  test_set_motor_zero();
  test_value_clamping();
  test_unconfigured_motor();
  test_motor_with_trim();
  test_coast();
  test_brake();
  test_stop_modes();
  test_stop_all();
  test_default_stop_mode();
  test_custom_pwm_frequency();
  test_invalid_channel();
  test_reconfigure_motor();
  
  printf("\nAll tests PASSED!\n\n");
  return 0;
}


/* EOF -----------------------------------------------------------------------*/

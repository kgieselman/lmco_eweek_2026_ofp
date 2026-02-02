/*******************************************************************************
 * @file test_motor_driver_drv8833.cpp
 * @brief Unit tests for DRV8833 motor driver
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#define UNIT_TEST
#include "unit_test.h"
#include "../inc/error_handler.h"

/* Mock error_report for unit testing */
extern "C" void error_report(ErrorCode_t code, const char* file, int line) {
  (void)code; (void)file; (void)line;
}

#include "../src/motor_driver_drv8833.cpp"

#include <cstdio>


/* Test Cases ----------------------------------------------------------------*/

/*****************************************************************************
 * @brief Test motor configuration
 ****************************************************************************/
void test_configure_motor(void)
{
  printf("test_configure_motor: ");
  
  MotorDriverDRV8833 driver;
  
  /* Test valid configuration */
  ASSERT_TRUE(driver.configureMotor(MotorDriverDRV8833::MOTOR_A, 8, 9));
  ASSERT_TRUE(driver.isConfigured(MotorDriverDRV8833::MOTOR_A));
  ASSERT_FALSE(driver.isConfigured(MotorDriverDRV8833::MOTOR_B));
  ASSERT_FALSE(driver.isFullyConfigured());
  
  /* Configure second motor */
  ASSERT_TRUE(driver.configureMotor(MotorDriverDRV8833::MOTOR_B, 10, 11));
  ASSERT_TRUE(driver.isConfigured(MotorDriverDRV8833::MOTOR_B));
  ASSERT_TRUE(driver.isFullyConfigured());
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test invalid pin configuration
 ****************************************************************************/
void test_configure_invalid_pins(void)
{
  printf("test_configure_invalid_pins: ");
  
  MotorDriverDRV8833 driver;
  
  /* Test invalid pins */
  ASSERT_FALSE(driver.configureMotor(MotorDriverDRV8833::MOTOR_A, -1, 9));
  ASSERT_FALSE(driver.configureMotor(MotorDriverDRV8833::MOTOR_A, 8, -1));
  ASSERT_FALSE(driver.configureMotor(MotorDriverDRV8833::MOTOR_A, 30, 9));
  ASSERT_FALSE(driver.configureMotor(MotorDriverDRV8833::MOTOR_A, 8, 30));
  
  ASSERT_FALSE(driver.isConfigured(MotorDriverDRV8833::MOTOR_A));
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test configuration with encoder
 ****************************************************************************/
void test_configure_with_encoder(void)
{
  printf("test_configure_with_encoder: ");
  
  MotorDriverDRV8833 driver;
  
  /* Configure with encoder */
  ASSERT_TRUE(driver.configureMotor(MotorDriverDRV8833::MOTOR_A, 8, 9, 12));
  ASSERT_EQUAL(driver.getEncoderPin(MotorDriverDRV8833::MOTOR_A), 12);
  
  /* Configure without encoder */
  ASSERT_TRUE(driver.configureMotor(MotorDriverDRV8833::MOTOR_B, 10, 11));
  ASSERT_EQUAL(driver.getEncoderPin(MotorDriverDRV8833::MOTOR_B), -1);
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test set motor forward
 ****************************************************************************/
void test_set_motor_forward(void)
{
  printf("test_set_motor_forward: ");
  
  MotorDriverDRV8833 driver;
  driver.configureMotor(MotorDriverDRV8833::MOTOR_A, 8, 9);
  
  /* Test forward values */
  ASSERT_TRUE(driver.setMotor(MotorDriverDRV8833::MOTOR_A, 250));
  ASSERT_TRUE(driver.setMotor(MotorDriverDRV8833::MOTOR_A, 500));
  ASSERT_TRUE(driver.setMotor(MotorDriverDRV8833::MOTOR_A, 1));
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test set motor reverse
 ****************************************************************************/
void test_set_motor_reverse(void)
{
  printf("test_set_motor_reverse: ");
  
  MotorDriverDRV8833 driver;
  driver.configureMotor(MotorDriverDRV8833::MOTOR_A, 8, 9);
  
  /* Test reverse values */
  ASSERT_TRUE(driver.setMotor(MotorDriverDRV8833::MOTOR_A, -250));
  ASSERT_TRUE(driver.setMotor(MotorDriverDRV8833::MOTOR_A, -500));
  ASSERT_TRUE(driver.setMotor(MotorDriverDRV8833::MOTOR_A, -1));
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test set motor zero (stop)
 ****************************************************************************/
void test_set_motor_zero(void)
{
  printf("test_set_motor_zero: ");
  
  MotorDriverDRV8833 driver;
  driver.configureMotor(MotorDriverDRV8833::MOTOR_A, 8, 9);
  
  /* Set motor running then stop */
  driver.setMotor(MotorDriverDRV8833::MOTOR_A, 250);
  ASSERT_TRUE(driver.setMotor(MotorDriverDRV8833::MOTOR_A, 0));
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test value clamping
 ****************************************************************************/
void test_value_clamping(void)
{
  printf("test_value_clamping: ");
  
  MotorDriverDRV8833 driver;
  driver.configureMotor(MotorDriverDRV8833::MOTOR_A, 8, 9);
  
  /* Values outside range should be clamped, not rejected */
  ASSERT_TRUE(driver.setMotor(MotorDriverDRV8833::MOTOR_A, 1000));
  ASSERT_TRUE(driver.setMotor(MotorDriverDRV8833::MOTOR_A, -1000));
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test unconfigured motor
 ****************************************************************************/
void test_unconfigured_motor(void)
{
  printf("test_unconfigured_motor: ");
  
  MotorDriverDRV8833 driver;
  
  /* Should fail on unconfigured motor */
  ASSERT_FALSE(driver.setMotor(MotorDriverDRV8833::MOTOR_A, 250));
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test motor with trim
 ****************************************************************************/
void test_motor_with_trim(void)
{
  printf("test_motor_with_trim: ");
  
  MotorDriverDRV8833 driver;
  driver.configureMotor(MotorDriverDRV8833::MOTOR_A, 8, 9);
  
  /* Test various trim values */
  ASSERT_TRUE(driver.setMotorWithTrim(MotorDriverDRV8833::MOTOR_A, 500, 1.0f));
  ASSERT_TRUE(driver.setMotorWithTrim(MotorDriverDRV8833::MOTOR_A, 500, 0.5f));
  ASSERT_TRUE(driver.setMotorWithTrim(MotorDriverDRV8833::MOTOR_A, 500, 0.0f));
  
  /* Trim should be clamped */
  ASSERT_TRUE(driver.setMotorWithTrim(MotorDriverDRV8833::MOTOR_A, 500, 1.5f));
  ASSERT_TRUE(driver.setMotorWithTrim(MotorDriverDRV8833::MOTOR_A, 500, -0.5f));
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test coast stop mode
 ****************************************************************************/
void test_coast(void)
{
  printf("test_coast: ");
  
  MotorDriverDRV8833 driver;
  driver.configureMotor(MotorDriverDRV8833::MOTOR_A, 8, 9);
  
  /* Set motor running then coast */
  driver.setMotor(MotorDriverDRV8833::MOTOR_A, 250);
  driver.coast(MotorDriverDRV8833::MOTOR_A);
  
  /* Should not throw on unconfigured */
  driver.coast(MotorDriverDRV8833::MOTOR_B);
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test brake stop mode
 ****************************************************************************/
void test_brake(void)
{
  printf("test_brake: ");
  
  MotorDriverDRV8833 driver;
  driver.configureMotor(MotorDriverDRV8833::MOTOR_A, 8, 9);
  
  /* Set motor running then brake */
  driver.setMotor(MotorDriverDRV8833::MOTOR_A, 250);
  driver.brake(MotorDriverDRV8833::MOTOR_A);
  
  /* Should not throw on unconfigured */
  driver.brake(MotorDriverDRV8833::MOTOR_B);
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test stop with mode selection
 ****************************************************************************/
void test_stop_modes(void)
{
  printf("test_stop_modes: ");
  
  MotorDriverDRV8833 driver;
  driver.configureMotor(MotorDriverDRV8833::MOTOR_A, 8, 9);
  
  /* Test stop with coast mode */
  driver.setMotor(MotorDriverDRV8833::MOTOR_A, 250);
  driver.stop(MotorDriverDRV8833::MOTOR_A, MotorDriverDRV8833::STOP_COAST);
  
  /* Test stop with brake mode */
  driver.setMotor(MotorDriverDRV8833::MOTOR_A, 250);
  driver.stop(MotorDriverDRV8833::MOTOR_A, MotorDriverDRV8833::STOP_BRAKE);
  
  /* Test default stop mode */
  driver.setMotor(MotorDriverDRV8833::MOTOR_A, 250);
  driver.stop(MotorDriverDRV8833::MOTOR_A);
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test stop all motors
 ****************************************************************************/
void test_stop_all(void)
{
  printf("test_stop_all: ");
  
  MotorDriverDRV8833 driver;
  driver.configureMotor(MotorDriverDRV8833::MOTOR_A, 8, 9);
  driver.configureMotor(MotorDriverDRV8833::MOTOR_B, 10, 11);
  
  /* Set both motors running */
  driver.setMotor(MotorDriverDRV8833::MOTOR_A, 250);
  driver.setMotor(MotorDriverDRV8833::MOTOR_B, -250);
  
  /* Stop all with coast */
  driver.stopAll(MotorDriverDRV8833::STOP_COAST);
  
  /* Stop all with brake */
  driver.setMotor(MotorDriverDRV8833::MOTOR_A, 250);
  driver.setMotor(MotorDriverDRV8833::MOTOR_B, -250);
  driver.stopAll(MotorDriverDRV8833::STOP_BRAKE);
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test default stop mode setting
 ****************************************************************************/
void test_default_stop_mode(void)
{
  printf("test_default_stop_mode: ");
  
  MotorDriverDRV8833 driver;
  
  /* Default should be coast */
  ASSERT_EQUAL(static_cast<int>(driver.getDefaultStopMode()), static_cast<int>(MotorDriverDRV8833::STOP_COAST));
  
  /* Change to brake */
  driver.setDefaultStopMode(MotorDriverDRV8833::STOP_BRAKE);
  ASSERT_EQUAL(static_cast<int>(driver.getDefaultStopMode()), static_cast<int>(MotorDriverDRV8833::STOP_BRAKE));
  
  /* Change back to coast */
  driver.setDefaultStopMode(MotorDriverDRV8833::STOP_COAST);
  ASSERT_EQUAL(static_cast<int>(driver.getDefaultStopMode()), static_cast<int>(MotorDriverDRV8833::STOP_COAST));
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test custom PWM frequency
 ****************************************************************************/
void test_custom_pwm_frequency(void)
{
  printf("test_custom_pwm_frequency: ");
  
  /* Test with different frequencies */
  MotorDriverDRV8833 driver1(10000);  /* 10 kHz */
  ASSERT_TRUE(driver1.configureMotor(MotorDriverDRV8833::MOTOR_A, 8, 9));
  
  MotorDriverDRV8833 driver2(50000);  /* 50 kHz */
  ASSERT_TRUE(driver2.configureMotor(MotorDriverDRV8833::MOTOR_A, 10, 11));
  
  printf("PASSED\n");
}

/*****************************************************************************
 * @brief Test invalid channel
 ****************************************************************************/
void test_invalid_channel(void)
{
  printf("test_invalid_channel: ");
  
  MotorDriverDRV8833 driver;
  
  /* Test invalid channel on various methods */
  ASSERT_FALSE(driver.isConfigured(static_cast<MotorDriverDRV8833::MotorChannel>(5)));
  ASSERT_EQUAL(driver.getEncoderPin(static_cast<MotorDriverDRV8833::MotorChannel>(5)), -1);
  ASSERT_FALSE(driver.setMotor(static_cast<MotorDriverDRV8833::MotorChannel>(5), 250));
  
  printf("PASSED\n");
}


/* Main Function -------------------------------------------------------------*/
int main(void)
{
  printf("\n=== DRV8833 Motor Driver Unit Tests ===\n\n");
  
  test_configure_motor();
  test_configure_invalid_pins();
  test_configure_with_encoder();
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
  
  printf("\nAll tests PASSED!\n\n");
  return 0;
}


/* EOF -----------------------------------------------------------------------*/

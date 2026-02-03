/*******************************************************************************
 * @file test_differential.cpp
 * @brief Unit tests for differential drive train motor mixing
 *
 * Tests the motor value calculations for differential drive.
 ******************************************************************************/

#include "unit_test.h"
#include <cmath>
#include <algorithm>


/* Constants -----------------------------------------------------------------*/
constexpr int USER_INPUT_MIN = -500;
constexpr int USER_INPUT_MAX = 500;
constexpr int MOTOR_COUNT = 2;
constexpr int MOTOR_LEFT = 0;
constexpr int MOTOR_RIGHT = 1;


/* Function Definitions ------------------------------------------------------*/

/*******************************************************************************
 * @brief Calculate differential drive motor values
 ******************************************************************************/
void calculate_motor_values(int speed, int turn, int* leftOut, int* rightOut)
{
  *leftOut = speed + turn;
  *rightOut = speed - turn;
}

/*******************************************************************************
 * @brief Calculate PWM scaling multiplier
 ******************************************************************************/
float calculate_multiplier(int speed, int turn, int leftVal, int rightVal, int pwmTop)
{
  int inputMax = std::max(std::abs(speed), std::abs(turn));
  int calcMax = std::max(std::abs(leftVal), std::abs(rightVal));

  if (calcMax <= 0)
  {
    return 0.0f;
  }

  float inputPercent = static_cast<float>(inputMax) / USER_INPUT_MAX;
  return (inputPercent * pwmTop) / static_cast<float>(calcMax);
}

/*******************************************************************************
 * @brief Validate user input is in range
 ******************************************************************************/
bool validate_input(int value)
{
  return (value >= USER_INPUT_MIN) && (value <= USER_INPUT_MAX);
}

/*============================================================================*/
/* Input Validation Tests                                                     */
/*============================================================================*/

TEST_FUNC(validate_center_input)
{
  ASSERT_TRUE(validate_input(0));
}

TEST_FUNC(validate_min_input)
{
  ASSERT_TRUE(validate_input(-500));
  ASSERT_FALSE(validate_input(-501));
}

TEST_FUNC(validate_max_input)
{
  ASSERT_TRUE(validate_input(500));
  ASSERT_FALSE(validate_input(501));
}

/*============================================================================*/
/* Motor Mixing Tests                                                         */
/*============================================================================*/

TEST_FUNC(mix_stopped)
{
  int left, right;
  calculate_motor_values(0, 0, &left, &right);
  ASSERT_EQUAL(0, left);
  ASSERT_EQUAL(0, right);
}

TEST_FUNC(mix_forward_only)
{
  int left, right;
  calculate_motor_values(500, 0, &left, &right);
  ASSERT_EQUAL(500, left);
  ASSERT_EQUAL(500, right);
}

TEST_FUNC(mix_reverse_only)
{
  int left, right;
  calculate_motor_values(-500, 0, &left, &right);
  ASSERT_EQUAL(-500, left);
  ASSERT_EQUAL(-500, right);
}

TEST_FUNC(mix_turn_right_only)
{
  int left, right;
  calculate_motor_values(0, 500, &left, &right);
  ASSERT_EQUAL(500, left);   // Left motor forward
  ASSERT_EQUAL(-500, right); // Right motor reverse
}

TEST_FUNC(mix_turn_left_only)
{
  int left, right;
  calculate_motor_values(0, -500, &left, &right);
  ASSERT_EQUAL(-500, left);  // Left motor reverse
  ASSERT_EQUAL(500, right);  // Right motor forward
}

TEST_FUNC(mix_forward_turn_right)
{
  int left, right;
  calculate_motor_values(300, 200, &left, &right);
  ASSERT_EQUAL(500, left);   // speed + turn
  ASSERT_EQUAL(100, right);  // speed - turn
}

TEST_FUNC(mix_forward_turn_left)
{
  int left, right;
  calculate_motor_values(300, -200, &left, &right);
  ASSERT_EQUAL(100, left);   // speed + turn
  ASSERT_EQUAL(500, right);  // speed - turn
}

TEST_FUNC(mix_reverse_turn_right)
{
  int left, right;
  calculate_motor_values(-300, 200, &left, &right);
  ASSERT_EQUAL(-100, left);  // -300 + 200
  ASSERT_EQUAL(-500, right); // -300 - 200
}

TEST_FUNC(mix_half_speed_no_turn)
{
  int left, right;
  calculate_motor_values(250, 0, &left, &right);
  ASSERT_EQUAL(250, left);
  ASSERT_EQUAL(250, right);
}

/*============================================================================*/
/* Scaling Tests                                                              */
/*============================================================================*/

TEST_FUNC(scale_zero_input)
{
  float mult = calculate_multiplier(0, 0, 0, 0, 1000);
  ASSERT_FLOAT_EQUAL(0.0f, mult, 0.001f);
}

TEST_FUNC(scale_full_forward)
{
  int left, right;
  calculate_motor_values(500, 0, &left, &right);
  float mult = calculate_multiplier(500, 0, left, right, 1000);
  // Full input should scale to full PWM
  float expectedPwm = std::abs(left) * mult;
  ASSERT_FLOAT_EQUAL(1000.0f, expectedPwm, 1.0f);
}

TEST_FUNC(scale_preserves_ratio)
{
  int left, right;
  calculate_motor_values(300, 200, &left, &right);
  float mult = calculate_multiplier(300, 200, left, right, 1000);

  float pwmLeft = std::abs(left) * mult;
  float pwmRight = std::abs(right) * mult;

  // The stronger motor (left=500) should scale to max input (300/500)*1000=600
  // The ratio between motors should be preserved
  float ratio = static_cast<float>(right) / static_cast<float>(left);
  float pwmRatio = pwmRight / pwmLeft;
  ASSERT_FLOAT_EQUAL(ratio, pwmRatio, 0.01f);
}

/*============================================================================*/
/* Symmetry Tests                                                             */
/*============================================================================*/

TEST_FUNC(symmetric_turning)
{
  int left1, right1, left2, right2;
  calculate_motor_values(0, 300, &left1, &right1);
  calculate_motor_values(0, -300, &left2, &right2);
  // Should be mirror images
  ASSERT_EQUAL(left1, -left2);
  ASSERT_EQUAL(right1, -right2);
}

TEST_FUNC(symmetric_speed)
{
  int left1, right1, left2, right2;
  calculate_motor_values(300, 0, &left1, &right1);
  calculate_motor_values(-300, 0, &left2, &right2);
  // Should be mirror images
  ASSERT_EQUAL(left1, -left2);
  ASSERT_EQUAL(right1, -right2);
}

/*============================================================================*/
/* Boundary Tests                                                             */
/*============================================================================*/

TEST_FUNC(boundary_combined_max)
{
  int left, right;
  calculate_motor_values(500, 500, &left, &right);
  ASSERT_EQUAL(1000, left);  // Exceeds normal range before scaling
  ASSERT_EQUAL(0, right);
}

TEST_FUNC(boundary_combined_min)
{
  int left, right;
  calculate_motor_values(-500, -500, &left, &right);
  ASSERT_EQUAL(-1000, left);
  ASSERT_EQUAL(0, right);
}

/*============================================================================*/
/* Main                                                                       */
/*============================================================================*/

int main(void)
{
  printf("\n=== Differential Drive Train Unit Tests ===\n");

  SUITE_START("Input Validation");
  RUN_TEST(validate_center_input);
  RUN_TEST(validate_min_input);
  RUN_TEST(validate_max_input);

  SUITE_START("Motor Mixing - Basic");
  RUN_TEST(mix_stopped);
  RUN_TEST(mix_forward_only);
  RUN_TEST(mix_reverse_only);
  RUN_TEST(mix_half_speed_no_turn);

  SUITE_START("Motor Mixing - Turning");
  RUN_TEST(mix_turn_right_only);
  RUN_TEST(mix_turn_left_only);
  RUN_TEST(mix_forward_turn_right);
  RUN_TEST(mix_forward_turn_left);
  RUN_TEST(mix_reverse_turn_right);

  SUITE_START("PWM Scaling");
  RUN_TEST(scale_zero_input);
  RUN_TEST(scale_full_forward);
  RUN_TEST(scale_preserves_ratio);

  SUITE_START("Symmetry");
  RUN_TEST(symmetric_turning);
  RUN_TEST(symmetric_speed);

  SUITE_START("Boundary Conditions");
  RUN_TEST(boundary_combined_max);
  RUN_TEST(boundary_combined_min);

  PRINT_SUMMARY();

  return (g_tests_failed > 0) ? 1 : 0;
}


/* EOF -----------------------------------------------------------------------*/

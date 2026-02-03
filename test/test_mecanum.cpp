/*******************************************************************************
 * @file test_mecanum.cpp
 * @brief Unit tests for mecanum drive train motor mixing
 *
 * Tests the motor value calculations for mecanum (omnidirectional) drive.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "unit_test.h"
#include <cmath>
#include <algorithm>


/* Constants -----------------------------------------------------------------*/
constexpr int USER_INPUT_MIN = -500;
constexpr int USER_INPUT_MAX = 500;
constexpr int MOTOR_COUNT = 4;
constexpr int MOTOR_FL = 0;  // Front Left
constexpr int MOTOR_FR = 1;  // Front Right
constexpr int MOTOR_RR = 2;  // Rear Right
constexpr int MOTOR_RL = 3;  // Rear Left

/* Helper Function Definitions -----------------------------------------------*/

/*******************************************************************************
 * @brief Calculate mecanum drive motor values
 *
 * Motor layout (top view):
 *   FL  FR
 *   RL  RR
 ******************************************************************************/
void calculate_motor_values(int speed, int strafe, int turn, int* motors)
{
  motors[MOTOR_FL] = speed + strafe + turn;
  motors[MOTOR_FR] = speed - strafe - turn;
  motors[MOTOR_RR] = speed + strafe - turn;
  motors[MOTOR_RL] = speed - strafe + turn;
}

/*******************************************************************************
 * @brief Calculate PWM scaling multiplier
 ******************************************************************************/
float calculate_multiplier(int speed, int strafe, int turn, 
                           const int* motors, int pwmTop)
{
  int inputMax = std::max({std::abs(speed), std::abs(strafe), std::abs(turn)});
  int calcMax = std::max({std::abs(motors[MOTOR_FL]), std::abs(motors[MOTOR_FR]),
                          std::abs(motors[MOTOR_RR]), std::abs(motors[MOTOR_RL])});

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

TEST_FUNC(validate_boundary_inputs)
{
  ASSERT_TRUE(validate_input(-500));
  ASSERT_TRUE(validate_input(500));
  ASSERT_FALSE(validate_input(-501));
  ASSERT_FALSE(validate_input(501));
}

/*============================================================================*/
/* Motor Mixing - Basic Movement Tests                                        */
/*============================================================================*/

TEST_FUNC(mix_stopped)
{
  int motors[4];
  calculate_motor_values(0, 0, 0, motors);
  ASSERT_EQUAL(0, motors[MOTOR_FL]);
  ASSERT_EQUAL(0, motors[MOTOR_FR]);
  ASSERT_EQUAL(0, motors[MOTOR_RR]);
  ASSERT_EQUAL(0, motors[MOTOR_RL]);
}

TEST_FUNC(mix_forward_only)
{
  int motors[4];
  calculate_motor_values(500, 0, 0, motors);
  // All motors forward with same speed
  ASSERT_EQUAL(500, motors[MOTOR_FL]);
  ASSERT_EQUAL(500, motors[MOTOR_FR]);
  ASSERT_EQUAL(500, motors[MOTOR_RR]);
  ASSERT_EQUAL(500, motors[MOTOR_RL]);
}

TEST_FUNC(mix_reverse_only)
{
  int motors[4];
  calculate_motor_values(-500, 0, 0, motors);
  // All motors reverse with same speed
  ASSERT_EQUAL(-500, motors[MOTOR_FL]);
  ASSERT_EQUAL(-500, motors[MOTOR_FR]);
  ASSERT_EQUAL(-500, motors[MOTOR_RR]);
  ASSERT_EQUAL(-500, motors[MOTOR_RL]);
}

/*============================================================================*/
/* Motor Mixing - Strafe Tests                                                */
/*============================================================================*/

TEST_FUNC(mix_strafe_right)
{
  int motors[4];
  calculate_motor_values(0, 500, 0, motors);
  // Strafe right pattern
  ASSERT_EQUAL(500, motors[MOTOR_FL]);   // +strafe
  ASSERT_EQUAL(-500, motors[MOTOR_FR]);  // -strafe
  ASSERT_EQUAL(500, motors[MOTOR_RR]);   // +strafe
  ASSERT_EQUAL(-500, motors[MOTOR_RL]);  // -strafe
}

TEST_FUNC(mix_strafe_left)
{
  int motors[4];
  calculate_motor_values(0, -500, 0, motors);
  // Strafe left pattern (opposite of right)
  ASSERT_EQUAL(-500, motors[MOTOR_FL]);
  ASSERT_EQUAL(500, motors[MOTOR_FR]);
  ASSERT_EQUAL(-500, motors[MOTOR_RR]);
  ASSERT_EQUAL(500, motors[MOTOR_RL]);
}

/*============================================================================*/
/* Motor Mixing - Turn Tests                                                  */
/*============================================================================*/

TEST_FUNC(mix_turn_clockwise)
{
  int motors[4];
  calculate_motor_values(0, 0, 500, motors);
  // Turn right (clockwise): left side forward, right side reverse
  ASSERT_EQUAL(500, motors[MOTOR_FL]);   // +turn
  ASSERT_EQUAL(-500, motors[MOTOR_FR]);  // -turn
  ASSERT_EQUAL(-500, motors[MOTOR_RR]);  // -turn
  ASSERT_EQUAL(500, motors[MOTOR_RL]);   // +turn
}

TEST_FUNC(mix_turn_counter_clockwise)
{
  int motors[4];
  calculate_motor_values(0, 0, -500, motors);
  // Turn left (counter-clockwise): right side forward, left side reverse
  ASSERT_EQUAL(-500, motors[MOTOR_FL]);
  ASSERT_EQUAL(500, motors[MOTOR_FR]);
  ASSERT_EQUAL(500, motors[MOTOR_RR]);
  ASSERT_EQUAL(-500, motors[MOTOR_RL]);
}

/*============================================================================*/
/* Motor Mixing - Diagonal Tests                                              */
/*============================================================================*/

TEST_FUNC(mix_diagonal_forward_right)
{
  int motors[4];
  // Forward + strafe right = diagonal motion
  calculate_motor_values(500, 500, 0, motors);
  ASSERT_EQUAL(1000, motors[MOTOR_FL]);  // speed + strafe
  ASSERT_EQUAL(0, motors[MOTOR_FR]);     // speed - strafe
  ASSERT_EQUAL(1000, motors[MOTOR_RR]);  // speed + strafe
  ASSERT_EQUAL(0, motors[MOTOR_RL]);     // speed - strafe
}

TEST_FUNC(mix_diagonal_forward_left)
{
  int motors[4];
  calculate_motor_values(500, -500, 0, motors);
  ASSERT_EQUAL(0, motors[MOTOR_FL]);     // speed + strafe
  ASSERT_EQUAL(1000, motors[MOTOR_FR]);  // speed - strafe
  ASSERT_EQUAL(0, motors[MOTOR_RR]);     // speed + strafe
  ASSERT_EQUAL(1000, motors[MOTOR_RL]);  // speed - strafe
}

/*============================================================================*/
/* Motor Mixing - Combined Movement Tests                                     */
/*============================================================================*/

TEST_FUNC(mix_forward_strafe_turn)
{
  int motors[4];
  calculate_motor_values(200, 100, 50, motors);
  // FL = 200 + 100 + 50 = 350
  // FR = 200 - 100 - 50 = 50
  // RR = 200 + 100 - 50 = 250
  // RL = 200 - 100 + 50 = 150
  ASSERT_EQUAL(350, motors[MOTOR_FL]);
  ASSERT_EQUAL(50, motors[MOTOR_FR]);
  ASSERT_EQUAL(250, motors[MOTOR_RR]);
  ASSERT_EQUAL(150, motors[MOTOR_RL]);
}

/*============================================================================*/
/* Scaling Tests                                                              */
/*============================================================================*/

TEST_FUNC(scale_zero_input)
{
  int motors[4] = {0, 0, 0, 0};
  float mult = calculate_multiplier(0, 0, 0, motors, 1500);
  ASSERT_FLOAT_EQUAL(0.0f, mult, 0.001f);
}

TEST_FUNC(scale_full_forward)
{
  int motors[4];
  calculate_motor_values(500, 0, 0, motors);
  float mult = calculate_multiplier(500, 0, 0, motors, 1500);
  // Full input should scale to full PWM
  float expectedPwm = std::abs(motors[MOTOR_FL]) * mult;
  ASSERT_FLOAT_EQUAL(1500.0f, expectedPwm, 1.0f);
}

TEST_FUNC(scale_combined_clipping_prevention)
{
  int motors[4];
  // Max combined input could theoretically exceed PWM range
  calculate_motor_values(500, 500, 500, motors);
  float mult = calculate_multiplier(500, 500, 500, motors, 1500);

  // All scaled values should not exceed PWM top
  for (int i = 0; i < 4; i++)
  {
    float scaledPwm = std::abs(motors[i]) * mult;
    ASSERT_TRUE(scaledPwm <= 1500.0f);
  }
}

/*============================================================================*/
/* Symmetry Tests                                                             */
/*============================================================================*/

TEST_FUNC(symmetric_strafe)
{
  int motors1[4], motors2[4];
  calculate_motor_values(0, 300, 0, motors1);
  calculate_motor_values(0, -300, 0, motors2);
  // Should be mirror images
  ASSERT_EQUAL(motors1[MOTOR_FL], -motors2[MOTOR_FL]);
  ASSERT_EQUAL(motors1[MOTOR_FR], -motors2[MOTOR_FR]);
  ASSERT_EQUAL(motors1[MOTOR_RR], -motors2[MOTOR_RR]);
  ASSERT_EQUAL(motors1[MOTOR_RL], -motors2[MOTOR_RL]);
}

TEST_FUNC(symmetric_turn)
{
  int motors1[4], motors2[4];
  calculate_motor_values(0, 0, 300, motors1);
  calculate_motor_values(0, 0, -300, motors2);
  // Should be mirror images
  ASSERT_EQUAL(motors1[MOTOR_FL], -motors2[MOTOR_FL]);
  ASSERT_EQUAL(motors1[MOTOR_FR], -motors2[MOTOR_FR]);
  ASSERT_EQUAL(motors1[MOTOR_RR], -motors2[MOTOR_RR]);
  ASSERT_EQUAL(motors1[MOTOR_RL], -motors2[MOTOR_RL]);
}

/*============================================================================*/
/* Main                                                                       */
/*============================================================================*/

int main(void)
{
  printf("\n=== Mecanum Drive Train Unit Tests ===\n");

  SUITE_START("Input Validation");
  RUN_TEST(validate_center_input);
  RUN_TEST(validate_boundary_inputs);

  SUITE_START("Motor Mixing - Basic");
  RUN_TEST(mix_stopped);
  RUN_TEST(mix_forward_only);
  RUN_TEST(mix_reverse_only);

  SUITE_START("Motor Mixing - Strafe");
  RUN_TEST(mix_strafe_right);
  RUN_TEST(mix_strafe_left);

  SUITE_START("Motor Mixing - Turn");
  RUN_TEST(mix_turn_clockwise);
  RUN_TEST(mix_turn_counter_clockwise);

  SUITE_START("Motor Mixing - Diagonal");
  RUN_TEST(mix_diagonal_forward_right);
  RUN_TEST(mix_diagonal_forward_left);

  SUITE_START("Motor Mixing - Combined");
  RUN_TEST(mix_forward_strafe_turn);

  SUITE_START("PWM Scaling");
  RUN_TEST(scale_zero_input);
  RUN_TEST(scale_full_forward);
  RUN_TEST(scale_combined_clipping_prevention);

  SUITE_START("Symmetry");
  RUN_TEST(symmetric_strafe);
  RUN_TEST(symmetric_turn);

  PRINT_SUMMARY();

  return (g_tests_failed > 0) ? 1 : 0;
}


/* EOF -----------------------------------------------------------------------*/

/******************************************************************************
 * @file test_differential.cpp
 * @brief Unit tests for differential drive train calculations
 *****************************************************************************/

#include "unit_test.h"
#include <algorithm>

/* Constants */
const int USER_INPUT_MAX = 500;
const int PWM_TOP_COUNT = 1000;

/* Result structure */
struct DriveResult {
    int leftPWM;
    int rightPWM;
    bool leftFwd;
    bool leftRev;
    bool rightFwd;
    bool rightRev;
};

/* Simulate differential drive calculation */
DriveResult calc_diff(int speed, int turn, float trimLF=1.0, float trimLR=1.0, 
                      float trimRF=1.0, float trimRR=1.0)
{
    DriveResult r = {0, 0, false, false, false, false};
    
    // Calculate raw values
    int left = speed + turn;
    int right = speed - turn;
    
    // Apply trim
    if (left > 0) left = (int)(left * trimLF);
    else if (left < 0) left = (int)(left * trimLR);
    
    if (right > 0) right = (int)(right * trimRF);
    else if (right < 0) right = (int)(right * trimRR);
    
    // Calculate scaling
    int inputMax = std::max(std::abs(speed), std::abs(turn));
    int calcMax = std::max(std::abs(left), std::abs(right));
    
    float inputMaxPercent = (float)inputMax / USER_INPUT_MAX;
    float motorMultiplier = 0.0f;
    if (calcMax > 0) {
        motorMultiplier = (inputMaxPercent * PWM_TOP_COUNT) / (float)calcMax;
    }
    
    // Set outputs
    r.leftPWM = std::abs(left) * motorMultiplier;
    r.rightPWM = std::abs(right) * motorMultiplier;
    r.leftFwd = (left > 0);
    r.leftRev = (left < 0);
    r.rightFwd = (right > 0);
    r.rightRev = (right < 0);
    
    return r;
}

/* ===== TESTS ===== */

TEST_FUNC(forward_full_speed)
{
    DriveResult r = calc_diff(500, 0);
    ASSERT_EQUAL(1000, r.leftPWM);
    ASSERT_EQUAL(1000, r.rightPWM);
    ASSERT_TRUE(r.leftFwd && r.rightFwd);
    ASSERT_TRUE(!r.leftRev && !r.rightRev);
}

TEST_FUNC(backward_full_speed)
{
    DriveResult r = calc_diff(-500, 0);
    ASSERT_EQUAL(1000, r.leftPWM);
    ASSERT_EQUAL(1000, r.rightPWM);
    ASSERT_TRUE(!r.leftFwd && !r.rightFwd);
    ASSERT_TRUE(r.leftRev && r.rightRev);
}

TEST_FUNC(forward_half_speed)
{
    DriveResult r = calc_diff(250, 0);
    ASSERT_EQUAL(500, r.leftPWM);
    ASSERT_EQUAL(500, r.rightPWM);
}

TEST_FUNC(stopped)
{
    DriveResult r = calc_diff(0, 0);
    ASSERT_EQUAL(0, r.leftPWM);
    ASSERT_EQUAL(0, r.rightPWM);
}

TEST_FUNC(turn_right_in_place)
{
    DriveResult r = calc_diff(0, 500);
    ASSERT_TRUE(r.leftFwd);
    ASSERT_TRUE(r.rightRev);
    ASSERT_EQUAL(r.leftPWM, r.rightPWM);
}

TEST_FUNC(turn_left_in_place)
{
    DriveResult r = calc_diff(0, -500);
    ASSERT_TRUE(r.leftRev);
    ASSERT_TRUE(r.rightFwd);
    ASSERT_EQUAL(r.leftPWM, r.rightPWM);
}

TEST_FUNC(gentle_right_turn)
{
    DriveResult r = calc_diff(400, 100);
    ASSERT_TRUE(r.leftFwd && r.rightFwd);
    ASSERT_TRUE(r.leftPWM > r.rightPWM);
}

TEST_FUNC(gentle_left_turn)
{
    DriveResult r = calc_diff(400, -100);
    ASSERT_TRUE(r.leftFwd && r.rightFwd);
    ASSERT_TRUE(r.rightPWM > r.leftPWM);
}

TEST_FUNC(sharp_turn_forward)
{
    DriveResult r = calc_diff(300, 400);
    ASSERT_TRUE(r.leftFwd);
    ASSERT_TRUE(r.rightRev);
}

TEST_FUNC(no_overflow_max_inputs)
{
    DriveResult r = calc_diff(500, 500);
    ASSERT_IN_RANGE(r.leftPWM, 0, PWM_TOP_COUNT);
    ASSERT_IN_RANGE(r.rightPWM, 0, PWM_TOP_COUNT);
}

TEST_FUNC(scaling_preserves_ratio)
{
    DriveResult r = calc_diff(400, 200);
    int leftRaw = 600;
    int rightRaw = 200;
    float expectedRatio = (float)leftRaw / rightRaw;
    float actualRatio = (float)r.leftPWM / r.rightPWM;
    ASSERT_FLOAT_EQUAL(expectedRatio, actualRatio, 0.01f);
}

TEST_FUNC(trim_reduces_left_motor)
{
    DriveResult r = calc_diff(500, 0, 0.8f);
    ASSERT_TRUE(r.leftPWM < r.rightPWM);
    ASSERT_EQUAL(1000, r.rightPWM);
}

TEST_FUNC(trim_affects_correct_direction)
{
    DriveResult fwd = calc_diff(500, 0, 0.5f);
    DriveResult rev = calc_diff(-500, 0, 0.5f);
    
    ASSERT_TRUE(fwd.leftPWM < fwd.rightPWM);
    ASSERT_EQUAL(rev.leftPWM, rev.rightPWM);
}

TEST_FUNC(zero_trim_stops_motor)
{
    DriveResult r = calc_diff(500, 0, 0.0f);
    ASSERT_EQUAL(0, r.leftPWM);
    ASSERT_EQUAL(1000, r.rightPWM);
}

/* ===== MAIN ===== */

int main(void)
{
    printf("\n╔══════════════════════════════════════════════════╗\n");
    printf("║   Differential Drive Train Unit Tests           ║\n");
    printf("╚══════════════════════════════════════════════════╝\n");
    
    SUITE_START("Basic Movement");
    RUN_TEST(forward_full_speed);
    RUN_TEST(backward_full_speed);
    RUN_TEST(forward_half_speed);
    RUN_TEST(stopped);
    
    SUITE_START("Turning");
    RUN_TEST(turn_right_in_place);
    RUN_TEST(turn_left_in_place);
    RUN_TEST(gentle_right_turn);
    RUN_TEST(gentle_left_turn);
    RUN_TEST(sharp_turn_forward);
    
    SUITE_START("PWM Scaling");
    RUN_TEST(no_overflow_max_inputs);
    RUN_TEST(scaling_preserves_ratio);
    
    SUITE_START("Trim Application");
    RUN_TEST(trim_reduces_left_motor);
    RUN_TEST(trim_affects_correct_direction);
    RUN_TEST(zero_trim_stops_motor);
    
    PRINT_SUMMARY();
    
    return (g_tests_failed > 0) ? 1 : 0;
}

/* EOF */

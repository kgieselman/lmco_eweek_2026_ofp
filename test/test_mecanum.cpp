/******************************************************************************
 * @file test_mecanum.cpp
 * @brief Unit tests for mecanum drive train calculations
 *****************************************************************************/

#include "simple_test.h"
#include <algorithm>

/* Constants */
const float INPUT_MAX = 500.0f;
const float MOTOR_PWM_MAX = 1500.0f;

/* Motor indices */
enum Motor { FL=0, FR, RR, RL };

/* Result structure */
struct MecanumResult {
    int pwm[4];
    bool fwd[4];
    bool rev[4];
};

/* Simulate mecanum drive calculation */
MecanumResult calc_mecanum(int speed, int strafe, int turn)
{
    MecanumResult r;
    for (int i = 0; i < 4; i++) {
        r.pwm[i] = 0;
        r.fwd[i] = false;
        r.rev[i] = false;
    }
    
    // Calculate raw mecanum values
    int val[4];
    val[FL] = speed + strafe + turn;
    val[FR] = speed - strafe - turn;
    val[RR] = speed + strafe - turn;
    val[RL] = speed - strafe + turn;
    
    // Determine scaling
    int inputMax = std::max({std::abs(speed), std::abs(strafe), std::abs(turn)});
    int calcMax = std::max({std::abs(val[FL]), std::abs(val[FR]), 
                            std::abs(val[RR]), std::abs(val[RL])});
    
    float inputMaxPercent = (float)inputMax / INPUT_MAX;
    float motorMultiplier = 0.0f;
    if (calcMax > 0) {
        motorMultiplier = (inputMaxPercent * MOTOR_PWM_MAX) / (float)calcMax;
    }
    
    // Calculate PWM and direction
    for (int i = 0; i < 4; i++) {
        r.pwm[i] = std::abs(val[i]) * motorMultiplier;
        r.fwd[i] = (val[i] > 0);
        r.rev[i] = (val[i] < 0);
    }
    
    return r;
}

/* ===== TESTS ===== */

TEST_FUNC(forward_full_speed)
{
    MecanumResult r = calc_mecanum(500, 0, 0);
    for (int i = 0; i < 4; i++) {
        ASSERT_EQUAL(1500, r.pwm[i]);
        ASSERT_TRUE(r.fwd[i]);
        ASSERT_TRUE(!r.rev[i]);
    }
}

TEST_FUNC(backward_full_speed)
{
    MecanumResult r = calc_mecanum(-500, 0, 0);
    for (int i = 0; i < 4; i++) {
        ASSERT_EQUAL(1500, r.pwm[i]);
        ASSERT_TRUE(!r.fwd[i]);
        ASSERT_TRUE(r.rev[i]);
    }
}

TEST_FUNC(stopped)
{
    MecanumResult r = calc_mecanum(0, 0, 0);
    for (int i = 0; i < 4; i++) {
        ASSERT_EQUAL(0, r.pwm[i]);
    }
}

TEST_FUNC(forward_half_speed)
{
    MecanumResult r = calc_mecanum(250, 0, 0);
    for (int i = 0; i < 4; i++) {
        ASSERT_EQUAL(750, r.pwm[i]);
    }
}

TEST_FUNC(strafe_right_full)
{
    MecanumResult r = calc_mecanum(0, 500, 0);
    ASSERT_TRUE(r.fwd[FL] && r.fwd[RR]);
    ASSERT_TRUE(r.rev[FR] && r.rev[RL]);
    for (int i = 0; i < 4; i++) {
        ASSERT_EQUAL(1500, r.pwm[i]);
    }
}

TEST_FUNC(strafe_left_full)
{
    MecanumResult r = calc_mecanum(0, -500, 0);
    ASSERT_TRUE(r.fwd[FR] && r.fwd[RL]);
    ASSERT_TRUE(r.rev[FL] && r.rev[RR]);
}

TEST_FUNC(rotate_clockwise)
{
    MecanumResult r = calc_mecanum(0, 0, 500);
    ASSERT_TRUE(r.fwd[FL] && r.fwd[RL]);
    ASSERT_TRUE(r.rev[FR] && r.rev[RR]);
}

TEST_FUNC(rotate_counterclockwise)
{
    MecanumResult r = calc_mecanum(0, 0, -500);
    ASSERT_TRUE(r.fwd[FR] && r.fwd[RR]);
    ASSERT_TRUE(r.rev[FL] && r.rev[RL]);
}

TEST_FUNC(diagonal_forward_right)
{
    MecanumResult r = calc_mecanum(500, 500, 0);
    ASSERT_TRUE(r.pwm[FL] > 0 && r.pwm[RR] > 0);
    ASSERT_EQUAL(0, r.pwm[FR]);
    ASSERT_EQUAL(0, r.pwm[RL]);
}

TEST_FUNC(forward_and_rotate)
{
    MecanumResult r = calc_mecanum(400, 0, 100);
    for (int i = 0; i < 4; i++) {
        ASSERT_TRUE(r.fwd[i]);
    }
    ASSERT_TRUE(r.pwm[FL] > r.pwm[FR]);
    ASSERT_TRUE(r.pwm[RL] > r.pwm[RR]);
}

TEST_FUNC(all_inputs_combined)
{
    MecanumResult r = calc_mecanum(300, 200, 100);
    for (int i = 0; i < 4; i++) {
        ASSERT_IN_RANGE(r.pwm[i], 0, (int)MOTOR_PWM_MAX);
    }
}

TEST_FUNC(no_overflow_max_inputs)
{
    MecanumResult r = calc_mecanum(500, 500, 500);
    for (int i = 0; i < 4; i++) {
        ASSERT_IN_RANGE(r.pwm[i], 0, (int)MOTOR_PWM_MAX);
    }
    int maxPWM = std::max({r.pwm[0], r.pwm[1], r.pwm[2], r.pwm[3]});
    ASSERT_IN_RANGE(maxPWM, 1490, 1500);
}

TEST_FUNC(small_inputs_scaled)
{
    MecanumResult r = calc_mecanum(100, 50, 25);
    int maxPWM = std::max({r.pwm[0], r.pwm[1], r.pwm[2], r.pwm[3]});
    ASSERT_IN_RANGE(maxPWM, 280, 320);
}

TEST_FUNC(kinematics_fl_correct)
{
    // FL = speed + strafe + turn
    MecanumResult r = calc_mecanum(100, 200, 50);
    // Raw value should be 350, check direction
    ASSERT_TRUE(r.fwd[FL]);
}

TEST_FUNC(kinematics_fr_correct)
{
    // FR = speed - strafe - turn
    MecanumResult r = calc_mecanum(100, 200, 50);
    // Raw value should be -150, check direction
    ASSERT_TRUE(r.rev[FR]);
}

TEST_FUNC(symmetry_forward_backward)
{
    MecanumResult fwd = calc_mecanum(300, 0, 0);
    MecanumResult bwd = calc_mecanum(-300, 0, 0);
    for (int i = 0; i < 4; i++) {
        ASSERT_EQUAL(fwd.pwm[i], bwd.pwm[i]);
    }
}

TEST_FUNC(symmetry_left_right_strafe)
{
    MecanumResult right = calc_mecanum(0, 400, 0);
    MecanumResult left = calc_mecanum(0, -400, 0);
    for (int i = 0; i < 4; i++) {
        ASSERT_EQUAL(right.pwm[i], left.pwm[i]);
    }
}

/* ===== MAIN ===== */

int main(void)
{
    printf("\n╔══════════════════════════════════════════════════╗\n");
    printf("║   Mecanum Drive Train Unit Tests                ║\n");
    printf("╚══════════════════════════════════════════════════╝\n");
    
    SUITE_START("Basic Movement");
    RUN_TEST(forward_full_speed);
    RUN_TEST(backward_full_speed);
    RUN_TEST(stopped);
    RUN_TEST(forward_half_speed);
    
    SUITE_START("Strafing");
    RUN_TEST(strafe_right_full);
    RUN_TEST(strafe_left_full);
    
    SUITE_START("Rotation");
    RUN_TEST(rotate_clockwise);
    RUN_TEST(rotate_counterclockwise);
    
    SUITE_START("Combined Movement");
    RUN_TEST(diagonal_forward_right);
    RUN_TEST(forward_and_rotate);
    RUN_TEST(all_inputs_combined);
    
    SUITE_START("PWM Scaling");
    RUN_TEST(no_overflow_max_inputs);
    RUN_TEST(small_inputs_scaled);
    
    SUITE_START("Kinematics");
    RUN_TEST(kinematics_fl_correct);
    RUN_TEST(kinematics_fr_correct);
    
    SUITE_START("Symmetry");
    RUN_TEST(symmetry_forward_backward);
    RUN_TEST(symmetry_left_right_strafe);
    
    PRINT_SUMMARY();
    
    return (g_tests_failed > 0) ? 1 : 0;
}

/* EOF */

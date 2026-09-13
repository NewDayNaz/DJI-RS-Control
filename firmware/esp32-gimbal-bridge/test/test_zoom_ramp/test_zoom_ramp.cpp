#include <cmath>
#include <unity.h>

#include "zoom_ramp.h"

void setUp(void) {}
void tearDown(void) {}

static constexpr float kDt = 0.05f;  // 20 Hz, same as firmware ZOOM_HZ
static constexpr float kVmax = 600.0f;
static constexpr float kAccel = 1800.0f;
static constexpr float kVcut = 80.0f;
static constexpr float kDecel = 1800.0f;

static bool runUntil(float &pos, float &vel, float target, int maxTicks) {
    bool arrived = false;
    for (int i = 0; i < maxTicks; i++) {
        arrived = zoom::rampStep(pos, vel, target, kVmax, kAccel, kDt, kVcut, kDecel);
        pos = zoom::clampf(pos, zoom::kMin, zoom::kMax);
        if (arrived) return true;
    }
    return arrived;
}

void test_clamp_motor_rejects_empty_and_non_12bit_endpoints() {
    TEST_ASSERT_EQUAL_UINT16(1, zoom::clampMotor(0));
    TEST_ASSERT_EQUAL_UINT16(1, zoom::clampMotor(-40));
    TEST_ASSERT_EQUAL_UINT16(4095, zoom::clampMotor(4096));
    TEST_ASSERT_EQUAL_UINT16(4095, zoom::clampMotor(99999));
    TEST_ASSERT_EQUAL_UINT16(1, zoom::clampMotor(1));
    TEST_ASSERT_EQUAL_UINT16(4095, zoom::clampMotor(4095));
    TEST_ASSERT_EQUAL_UINT16(2000, zoom::clampMotor(2000));
}

void test_arrive_from_rest_on_large_jump() {
    float pos = 500.0f;
    float vel = 0.0f;
    TEST_ASSERT_TRUE(runUntil(pos, vel, 3500.0f, 400));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 3500.0f, pos);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, vel);
}

void test_stays_at_target() {
    float pos = 1800.0f;
    float vel = 0.0f;
    TEST_ASSERT_TRUE(zoom::rampStep(pos, vel, 1800.0f, kVmax, kAccel, kDt, kVcut, kDecel));
    TEST_ASSERT_EQUAL_FLOAT(1800.0f, pos);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, vel);
}

void test_high_speed_pass_does_not_snap_arrival() {
    float pos = 100.0f;
    float vel = 2000.0f;
    float target = 100.4f;
    bool arrived = zoom::rampStep(pos, vel, target, kVmax, kAccel, 0.0f, kVcut, kDecel);
    TEST_ASSERT_FALSE(arrived);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, pos);
    TEST_ASSERT_EQUAL_FLOAT(2000.0f, vel);
}

void test_vcut_kicks_through_dead_zone_on_long_move() {
    float pos = 200.0f;
    float vel = 10.0f;
    bool arrived = zoom::rampStep(pos, vel, 3000.0f, kVmax, 1.0f, kDt, kVcut, kDecel);
    TEST_ASSERT_FALSE(arrived);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, kVcut, std::fabs(vel));
}

void test_vcut_finishing_does_not_teleport_long_remaining() {
    float pos = 200.0f;
    float vel = 80.0f;
    float target = 3000.0f;
    bool arrived = zoom::rampStep(pos, vel, target, kVmax, kAccel, kDt, kVcut, kDecel);
    TEST_ASSERT_FALSE(arrived);
    TEST_ASSERT_TRUE(std::fabs(target - pos) > 2000.0f);
}

void test_step_capped_at_vmax_dt() {
    float pos = 1000.0f;
    float vel = 5000.0f;
    float target = 3000.0f;
    (void) zoom::rampStep(pos, vel, target, kVmax, kAccel, kDt, kVcut, kDecel);
    TEST_ASSERT_TRUE(pos - 1000.0f <= kVmax * kDt + 0.01f);
}

void test_reverse_retarget_does_not_drive_into_endpoint() {
    float pos = 800.0f;
    float vel = 0.0f;
    const float towardMin = zoom::kMin;
    for (int i = 0; i < 8; i++) {
        (void) zoom::rampStep(pos, vel, towardMin, kVmax, kAccel, kDt, kVcut, kDecel);
        pos = zoom::clampf(pos, zoom::kMin, zoom::kMax);
    }
    TEST_ASSERT_TRUE(vel < 0.0f);

    const float newTarget = 3500.0f;
    zoom::applyRetarget(pos, vel, newTarget);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, vel);

    float minPos = pos;
    for (int i = 0; i < 30; i++) {
        (void) zoom::rampStep(pos, vel, newTarget, kVmax, kAccel, kDt, kVcut, kDecel);
        pos = zoom::clampf(pos, zoom::kMin, zoom::kMax);
        if (pos < minPos) minPos = pos;
    }
    TEST_ASSERT_TRUE(pos > minPos);
    TEST_ASSERT_TRUE(vel > 0.0f);
    TEST_ASSERT_TRUE(pos > 1.5f);
}

void test_keeping_old_velocity_would_keep_driving_min() {
    // Documents why applyRetarget zeros vel: without it, cruise into 1 continues.
    float pos = 80.0f;
    float vel = -kVmax;
    float minPos = pos;
    for (int i = 0; i < 8; i++) {
        (void) zoom::rampStep(pos, vel, 3500.0f, kVmax, kAccel, kDt, kVcut, kDecel);
        pos = zoom::clampf(pos, zoom::kMin, zoom::kMax);
        if (pos < minPos) minPos = pos;
    }
    TEST_ASSERT_TRUE(minPos < 80.0f);
}

void test_interrupt_heads_back_toward_new_target() {
    float pos = 1500.0f;
    float vel = 0.0f;
    for (int i = 0; i < 10; i++) {
        (void) zoom::rampStep(pos, vel, 3200.0f, kVmax, kAccel, kDt, kVcut, kDecel);
    }
    float mid = pos;
    TEST_ASSERT_TRUE(mid > 1500.0f);
    TEST_ASSERT_TRUE(vel > 0.0f);

    zoom::applyRetarget(pos, vel, 1500.0f);
    for (int i = 0; i < 16; i++) {
        (void) zoom::rampStep(pos, vel, 1500.0f, kVmax, kAccel, kDt, kVcut, kDecel);
        pos = zoom::clampf(pos, zoom::kMin, zoom::kMax);
    }
    TEST_ASSERT_TRUE(pos < mid);
    TEST_ASSERT_TRUE(vel <= 0.0f);
}

void test_never_leaves_motor_span() {
    float pos = 10.0f;
    float vel = -400.0f;
    for (int i = 0; i < 80; i++) {
        (void) zoom::rampStep(pos, vel, zoom::kMin, kVmax, kAccel, kDt, kVcut, kDecel);
        pos = zoom::clampf(pos, zoom::kMin, zoom::kMax);
        TEST_ASSERT_TRUE(pos >= zoom::kMin);
        TEST_ASSERT_TRUE(pos <= zoom::kMax);
    }
}

int main(int argc, char **argv) {
    (void) argc;
    (void) argv;
    UNITY_BEGIN();
    RUN_TEST(test_clamp_motor_rejects_empty_and_non_12bit_endpoints);
    RUN_TEST(test_arrive_from_rest_on_large_jump);
    RUN_TEST(test_stays_at_target);
    RUN_TEST(test_high_speed_pass_does_not_snap_arrival);
    RUN_TEST(test_vcut_kicks_through_dead_zone_on_long_move);
    RUN_TEST(test_vcut_finishing_does_not_teleport_long_remaining);
    RUN_TEST(test_step_capped_at_vmax_dt);
    RUN_TEST(test_reverse_retarget_does_not_drive_into_endpoint);
    RUN_TEST(test_keeping_old_velocity_would_keep_driving_min);
    RUN_TEST(test_interrupt_heads_back_toward_new_target);
    RUN_TEST(test_never_leaves_motor_span);
    return UNITY_END();
}

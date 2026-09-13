#pragma once

// Trapezoidal zoom ramp shared by firmware (src/main.cpp) and host tests.
// Extended from dji_can_session.py's zoom_ramp_step() with independent
// decel, T_running (vcut), and reverse-on-retarget. Keep this header free
// of Arduino so native / pytest models can include or twin it.

#include <cmath>
#include <cstdint>

namespace zoom {

constexpr float kMin = 1.0f;
constexpr float kMax = 4095.0f;

inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Gimbal ignores exact motor-calib endpoints: 0 is empty, 4096 is not 12-bit.
inline uint16_t clampMotor(int position) {
    if (position < 1) return 1;
    if (position > 4095) return 4095;
    return static_cast<uint16_t>(position);
}

// New target the other way: drop cruise velocity so we do not keep driving
// into 1 / 4095 before reversing.
inline void applyRetarget(float commandedPos, float &vel, float newTarget) {
    if (vel * (newTarget - commandedPos) < 0.0f) vel = 0.0f;
}

// Advance one tick. vmax is cruise (T_max). vcut is T_running: kick through
// it on the way up, ease down to it, then stop — never cruise in the dead
// zone between T_static and T_running. Accel and decel are independent so a
// short move (accel+decel only) and a long move (with cruise) still share
// the same ramps.
inline bool rampStep(float &pos, float &vel, float target, float vmax,
                     float accel, float dt, float vcut, float decel) {
    if (vmax < 1.0f) vmax = 1.0f;
    if (accel < 1.0f) accel = 1.0f;
    if (decel < 1.0f) decel = 1.0f;
    vcut = clampf(vcut, 0.0f, vmax * 0.9f);
    dt = clampf(dt, 0.0f, 0.1f);
    float remaining = target - pos;
    // Same as dji_can_session.py: only arrived when close *and* slow.
    // A high-speed pass with remaining < 1.25 used to snap and drop the move.
    if (std::fabs(remaining) < 0.5f && std::fabs(vel) < 8.0f) {
        pos = target;
        vel = 0.0f;
        return true;
    }
    if (dt <= 0.0f) return false;

    float want = remaining > 0.0f ? 1.0f : -1.0f;
    float v = std::fabs(vel);
    float decelDist;
    if (vcut < 8.0f) {
        decelDist = (v * v) / (2.0f * decel);
    } else if (v > vcut) {
        decelDist = (v * v - vcut * vcut) / (2.0f * decel);
    } else {
        decelDist = 0.0f;
    }

    bool toward = vel * remaining > 0.0f;
    bool needDecel = toward && (decelDist >= std::fabs(remaining));
    // Last-inch crawl at T_running. Never treat a long remaining as "finishing"
    // — that used to snap pos=target after a retarget while vel was in-band.
    bool finishing = vcut >= 8.0f && toward && v > 12.0f && v <= vcut + 8.0f &&
                     std::fabs(remaining) <= std::fmax(16.0f, vcut * dt * 3.0f);
    if (finishing) {
        float step = std::copysign(std::fmin(vcut, vmax), want) * dt;
        if (std::fabs(remaining) <= std::fabs(step) + 0.5f) {
            pos = target;
            vel = 0.0f;
            return true;
        }
        vel = std::copysign(vcut, want);
        pos = pos + step;
        return false;
    }

    float acc;
    if (vel * remaining < 0.0f) {
        acc = -std::copysign(decel, vel);
    } else if (needDecel) {
        acc = std::fabs(vel) > 1e-6f ? -std::copysign(decel, vel) : 0.0f;
    } else if (v < vmax) {
        acc = want * accel;
    } else {
        acc = 0.0f;
        vel = std::copysign(vmax, vel);
    }

    vel = clampf(vel + acc * dt, -vmax, vmax);
    if (vcut >= 8.0f && !needDecel && vel * remaining > 0.0f &&
        std::fabs(vel) > 0.5f && std::fabs(vel) < vcut &&
        std::fabs(remaining) > std::fmax(64.0f, vcut * 0.5f)) {
        vel = std::copysign(vcut, want);
    }
    float step = vel * dt;
    float maxStep = vmax * dt;
    if (std::fabs(step) > maxStep) step = std::copysign(maxStep, step);
    pos = pos + step;
    if ((remaining > 0.0f && pos >= target) || (remaining < 0.0f && pos <= target)) {
        pos = target;
        vel = 0.0f;
        return true;
    }
    return false;
}

}  // namespace zoom

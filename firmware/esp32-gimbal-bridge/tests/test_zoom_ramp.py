from __future__ import annotations

from tests.zoom_ramp_model import (
    ZOOM_MAX,
    ZOOM_MIN,
    apply_retarget,
    clamp_motor,
    ramp_step,
    simulate,
)

DT = 0.05
VMAX = 600.0
ACCEL = 1800.0
VCUT = 80.0
DECEL = 1800.0


def test_clamp_motor_rejects_empty_and_non_12bit_endpoints():
    assert clamp_motor(0) == 1
    assert clamp_motor(-40) == 1
    assert clamp_motor(4096) == 4095
    assert clamp_motor(1) == 1
    assert clamp_motor(4095) == 4095
    assert clamp_motor(2000) == 2000


def test_arrive_from_rest_on_large_jump():
    hist = simulate(500.0, 0.0, 3500.0)
    pos, vel, arrived, _ = hist[-1]
    assert arrived
    assert pos == 3500.0
    assert vel == 0.0


def test_stays_at_target():
    pos, vel, arrived = ramp_step(1800.0, 0.0, 1800.0, VMAX, ACCEL, DT, VCUT, DECEL)
    assert arrived
    assert pos == 1800.0
    assert vel == 0.0


def test_high_speed_pass_does_not_snap_arrival():
    pos, vel, arrived = ramp_step(100.0, 2000.0, 100.4, VMAX, ACCEL, 0.0, VCUT, DECEL)
    assert not arrived
    assert pos == 100.0
    assert vel == 2000.0


def test_vcut_finishing_does_not_teleport_long_remaining():
    pos, vel, arrived = ramp_step(200.0, 80.0, 3000.0, VMAX, ACCEL, DT, VCUT, DECEL)
    assert not arrived
    assert abs(3000.0 - pos) > 2000.0


def test_step_capped_at_vmax_dt():
    pos, _, _ = ramp_step(1000.0, 5000.0, 3000.0, VMAX, ACCEL, DT, VCUT, DECEL)
    assert pos - 1000.0 <= VMAX * DT + 0.01


def test_vcut_kicks_through_dead_zone_on_long_move():
    # Tiny accel so one tick stays below T_running; firmware must kick to vcut.
    pos, vel, arrived = ramp_step(200.0, 10.0, 3000.0, VMAX, 1.0, DT, VCUT, DECEL)
    assert not arrived
    assert abs(abs(vel) - VCUT) < 0.01


def test_reverse_retarget_does_not_drive_into_endpoint():
    hist = simulate(800.0, 0.0, ZOOM_MIN, ticks=8)
    pos, vel, _, _ = hist[-1]
    assert vel < 0.0
    vel = apply_retarget(pos, vel, 3500.0)
    assert vel == 0.0
    hist2 = simulate(pos, vel, 3500.0, ticks=30)
    positions = [h[0] for h in hist2]
    assert hist2[-1][1] > 0.0
    assert min(positions) > 1.5
    assert hist2[-1][0] > pos


def test_keeping_old_velocity_keeps_driving_toward_min():
    hist = simulate(80.0, -VMAX, 3500.0, ticks=8)
    assert min(h[0] for h in hist) < 80.0


def test_interrupt_heads_back_toward_new_target():
    hist = simulate(1500.0, 0.0, 3200.0, ticks=80, retarget_at=10, new_target=1500.0)
    mid = hist[9][0]
    later = hist[-1][0]
    assert mid > 1500.0
    assert later < mid


def test_never_leaves_motor_span():
    hist = simulate(10.0, -400.0, ZOOM_MIN, ticks=80)
    for pos, _, _, _ in hist:
        assert ZOOM_MIN <= pos <= ZOOM_MAX

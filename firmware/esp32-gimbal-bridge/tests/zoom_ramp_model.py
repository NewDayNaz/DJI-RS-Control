"""Line-by-line twin of src/zoom_ramp.h for host pytest (no compiler required)."""

from __future__ import annotations

import math

ZOOM_MIN = 1.0
ZOOM_MAX = 4095.0


def clampf(v: float, lo: float, hi: float) -> float:
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def clamp_motor(position: int) -> int:
    if position < 1:
        return 1
    if position > 4095:
        return 4095
    return int(position)


def apply_retarget(commanded_pos: float, vel: float, new_target: float) -> float:
    if vel * (new_target - commanded_pos) < 0.0:
        return 0.0
    return vel


def ramp_step(
    pos: float,
    vel: float,
    target: float,
    vmax: float,
    accel: float,
    dt: float,
    vcut: float,
    decel: float,
) -> tuple[float, float, bool]:
    vmax = max(1.0, vmax)
    accel = max(1.0, accel)
    decel = max(1.0, decel)
    vcut = clampf(vcut, 0.0, vmax * 0.9)
    dt = clampf(dt, 0.0, 0.1)
    remaining = target - pos
    if abs(remaining) < 0.5 and abs(vel) < 8.0:
        return target, 0.0, True
    if dt <= 0.0:
        return pos, vel, False

    want = 1.0 if remaining > 0.0 else -1.0
    v = abs(vel)
    if vcut < 8.0:
        decel_dist = (v * v) / (2.0 * decel)
    elif v > vcut:
        decel_dist = (v * v - vcut * vcut) / (2.0 * decel)
    else:
        decel_dist = 0.0

    toward = vel * remaining > 0.0
    need_decel = toward and (decel_dist >= abs(remaining))
    finishing = (
        vcut >= 8.0
        and toward
        and v > 12.0
        and v <= vcut + 8.0
        and abs(remaining) <= max(16.0, vcut * dt * 3.0)
    )
    if finishing:
        step = math.copysign(min(vcut, vmax), want) * dt
        if abs(remaining) <= abs(step) + 0.5:
            return target, 0.0, True
        vel = math.copysign(vcut, want)
        return pos + step, vel, False

    if vel * remaining < 0.0:
        acc = -math.copysign(decel, vel)
    elif need_decel:
        acc = -math.copysign(decel, vel) if abs(vel) > 1e-6 else 0.0
    elif v < vmax:
        acc = want * accel
    else:
        acc = 0.0
        vel = math.copysign(vmax, vel)

    vel = clampf(vel + acc * dt, -vmax, vmax)
    if (
        vcut >= 8.0
        and not need_decel
        and vel * remaining > 0.0
        and abs(vel) > 0.5
        and abs(vel) < vcut
        and abs(remaining) > max(64.0, vcut * 0.5)
    ):
        vel = math.copysign(vcut, want)
    step = vel * dt
    max_step = vmax * dt
    if abs(step) > max_step:
        step = math.copysign(max_step, step)
    pos = pos + step
    if (remaining > 0.0 and pos >= target) or (remaining < 0.0 and pos <= target):
        return target, 0.0, True
    return pos, vel, False


def simulate(
    pos: float,
    vel: float,
    target: float,
    vmax: float = 600.0,
    accel: float = 1800.0,
    vcut: float = 80.0,
    decel: float = 1800.0,
    ticks: int = 400,
    dt: float = 0.05,
    retarget_at: int | None = None,
    new_target: float | None = None,
) -> list[tuple[float, float, bool, float]]:
    history: list[tuple[float, float, bool, float]] = []
    for i in range(ticks):
        if retarget_at is not None and i == retarget_at and new_target is not None:
            vel = apply_retarget(pos, vel, new_target)
            target = new_target
        pos, vel, arrived = ramp_step(pos, vel, target, vmax, accel, dt, vcut, decel)
        pos = clampf(pos, ZOOM_MIN, ZOOM_MAX)
        history.append((pos, vel, arrived, target))
        if arrived and (retarget_at is None or i >= retarget_at):
            break
    return history

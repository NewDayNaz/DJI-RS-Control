#!/usr/bin/env python3
"""Thread-owned CANable session for the DJI R SDK gimbal protocol."""

from __future__ import annotations

import math
import queue
import struct
import threading
import time
from dataclasses import dataclass
from typing import Any

import dji_gimbal_cli as proto

BITRATE = 1_000_000
SKIP_SLCAN_VIDS = {
    (0x1A86, 0x55D3),
    (0x303A, 0x1001),
}

SPEED_HZ = 20.0
ZOOM_HZ = 20.0
ZOOM_MIN = 0.0
ZOOM_MAX = 4096.0
ZOOM_VMAX_DEFAULT = 900.0
ZOOM_ACCEL_DEFAULT = 1800.0
DEADMAN_S = 0.25
ANGLE_POLL_S = 0.05
PUSH_STALE_S = 0.35


@dataclass
class AdapterInfo:
    device: str
    description: str
    interface: str
    vid: int | None = None
    pid: int | None = None


@dataclass
class GimbalSnapshot:
    connected: bool
    adapter: str | None
    interface: str | None
    yaw: float | None = None
    roll: float | None = None
    pitch: float | None = None
    zoom: int | None = None
    zoom_target: int | None = None
    zoom_vmax: float = ZOOM_VMAX_DEFAULT
    zoom_accel: float = ZOOM_ACCEL_DEFAULT
    last_rx_age_s: float | None = None
    last_error: str | None = None
    tx_ok: int = 0
    rx_ok: int = 0


@dataclass
class _HeldSpeed:
    yaw: float
    roll: float
    pitch: float
    at: float


def zoom_ramp_step(
    pos: float,
    vel: float,
    target: float,
    vmax: float,
    accel: float,
    dt: float,
) -> tuple[float, float, bool]:
    """Advance a 1-D trapezoidal move. Returns (pos, vel, arrived)."""
    vmax = max(1.0, vmax)
    accel = max(1.0, accel)
    dt = min(max(dt, 0.0), 0.1)
    remaining = target - pos
    if abs(remaining) < 0.5 and abs(vel) < 8.0:
        return target, 0.0, True
    if dt <= 0.0:
        return pos, vel, False

    want = 1.0 if remaining > 0.0 else -1.0
    stop_dist = (vel * vel) / (2.0 * accel)
    if vel * remaining < 0.0:
        acc = -math.copysign(accel, vel)
    elif stop_dist >= abs(remaining):
        acc = -math.copysign(accel, vel) if abs(vel) > 1e-6 else 0.0
    elif abs(vel) < vmax:
        acc = want * accel
    else:
        acc = 0.0
        vel = math.copysign(vmax, vel)

    vel = max(-vmax, min(vmax, vel + acc * dt))
    pos = pos + vel * dt
    if (remaining > 0.0 and pos >= target) or (remaining < 0.0 and pos <= target):
        return target, 0.0, True
    return pos, vel, False


class PacketReassembler:
    def __init__(self) -> None:
        self._buf = bytearray()
        self._pack_len = 0
        self._step = 0

    def reset(self) -> None:
        self._buf = bytearray()
        self._pack_len = 0
        self._step = 0

    def feed(self, payload: bytes) -> bytes | None:
        for b in payload:
            if self._step == 0:
                if b == proto.SOF:
                    self._buf = bytearray([b])
                    self._step = 1
                continue
            if self._step == 1:
                self._pack_len = b
                self._buf.append(b)
                self._step = 2
                continue
            if self._step == 2:
                self._pack_len |= (b & 0x03) << 8
                self._buf.append(b)
                self._step = 3
                continue
            if self._step == 3:
                self._buf.append(b)
                if len(self._buf) == 12:
                    hdr = bytes(self._buf[:10])
                    got = self._buf[10] | (self._buf[11] << 8)
                    if proto.crc16(hdr) == got:
                        self._step = 4
                    else:
                        self.reset()
                continue
            if self._step == 4:
                self._buf.append(b)
                if len(self._buf) == self._pack_len:
                    out = bytes(self._buf)
                    self.reset()
                    body_crc = struct.unpack_from("<I", out, len(out) - 4)[0]
                    if proto.crc32(out[:-4]) == body_crc:
                        return out
                    return None
        return None


def list_adapters() -> list[AdapterInfo]:
    adapters: list[AdapterInfo] = []
    try:
        import serial.tools.list_ports

        for p in serial.tools.list_ports.comports():
            vidpid = (p.vid, p.pid) if p.vid is not None and p.pid is not None else None
            if vidpid in SKIP_SLCAN_VIDS:
                continue
            adapters.append(
                AdapterInfo(
                    device=p.device,
                    description=p.description or "",
                    interface="slcan",
                    vid=p.vid,
                    pid=p.pid,
                )
            )
    except Exception:
        pass
    adapters.insert(
        0,
        AdapterInfo(device="auto", description="gs_usb then first slcan port", interface="auto"),
    )
    adapters.insert(
        1,
        AdapterInfo(device="gs_usb:0", description="CANable candleLight (gs_usb)", interface="gs_usb"),
    )
    return adapters


def open_can_bus(channel: str | None, interface: str | None):
    import can

    errors: list[str] = []
    if interface == "gs_usb" or (channel and channel.startswith("gs_usb")):
        ch = 0
        if channel and ":" in channel:
            ch = int(channel.split(":", 1)[1])
        return can.Bus(interface="gs_usb", channel=ch, bitrate=BITRATE), "gs_usb", f"gs_usb:{ch}"

    if channel and channel not in ("auto", ""):
        bus = can.Bus(interface="slcan", channel=channel, bitrate=BITRATE)
        return bus, "slcan", channel

    try:
        bus = can.Bus(interface="gs_usb", channel=0, bitrate=BITRATE)
        return bus, "gs_usb", "gs_usb:0"
    except Exception as exc:
        errors.append(f"gs_usb: {exc}")

    try:
        import serial.tools.list_ports

        for p in serial.tools.list_ports.comports():
            vidpid = (p.vid, p.pid) if p.vid is not None and p.pid is not None else None
            if vidpid in SKIP_SLCAN_VIDS:
                continue
            try:
                bus = can.Bus(interface="slcan", channel=p.device, bitrate=BITRATE)
                return bus, "slcan", p.device
            except Exception as exc:
                errors.append(f"slcan {p.device}: {exc}")
    except Exception as exc:
        errors.append(f"list ports: {exc}")

    raise RuntimeError("Could not open a CAN adapter. " + " | ".join(errors) if errors else "No adapter found.")


class GimbalCanSession:
    """One thread owns the python-can bus. Web handlers enqueue work."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._cmds: queue.Queue[dict[str, Any]] = queue.Queue()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._connected = False
        self._adapter: str | None = None
        self._interface: str | None = None
        self._yaw: float | None = None
        self._roll: float | None = None
        self._pitch: float | None = None
        self._zoom: int | None = None
        self._zoom_cmd: float | None = None
        self._zoom_vel = 0.0
        self._zoom_target: float | None = None
        self._zoom_sent: int | None = None
        self._zoom_vmax = ZOOM_VMAX_DEFAULT
        self._zoom_accel = ZOOM_ACCEL_DEFAULT
        self._last_rx = 0.0
        self._last_error: str | None = None
        self._tx_ok = 0
        self._rx_ok = 0
        self._held: _HeldSpeed | None = None

    def snapshot(self) -> GimbalSnapshot:
        now = time.monotonic()
        with self._lock:
            age = (now - self._last_rx) if self._last_rx else None
            return GimbalSnapshot(
                connected=self._connected,
                adapter=self._adapter,
                interface=self._interface,
                yaw=self._yaw,
                roll=self._roll,
                pitch=self._pitch,
                zoom=self._zoom,
                zoom_target=None if self._zoom_target is None else int(round(self._zoom_target)),
                zoom_vmax=self._zoom_vmax,
                zoom_accel=self._zoom_accel,
                last_rx_age_s=age,
                last_error=self._last_error,
                tx_ok=self._tx_ok,
                rx_ok=self._rx_ok,
            )

    def connect(self, channel: str | None = None, interface: str | None = None) -> GimbalSnapshot:
        self.disconnect()
        self._stop.clear()
        self._set_error(None)
        self._thread = threading.Thread(
            target=self._run,
            args=(channel, interface),
            name="gimbal-can",
            daemon=True,
        )
        self._thread.start()
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            snap = self.snapshot()
            if snap.connected or snap.last_error:
                return snap
            time.sleep(0.05)
        return self.snapshot()

    def disconnect(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._thread = None
        with self._lock:
            self._connected = False
            self._adapter = None
            self._interface = None
            self._held = None
            self._zoom_cmd = None
            self._zoom_vel = 0.0
            self._zoom_target = None
            self._zoom_sent = None
            self._zoom = None

    def hold_speed(self, yaw: float, roll: float, pitch: float) -> None:
        with self._lock:
            if not self._connected:
                return
            self._held = _HeldSpeed(yaw, roll, pitch, time.monotonic())

    def release_speed(self) -> None:
        with self._lock:
            self._held = None
            if not self._connected:
                return
        self._cmds.put({"op": "speed", "yaw": 0.0, "roll": 0.0, "pitch": 0.0, "once": True})

    def set_zoom_target(self, position: int) -> None:
        pos = float(max(int(ZOOM_MIN), min(int(ZOOM_MAX), int(position))))
        with self._lock:
            if not self._connected:
                return
            self._zoom_target = pos

    def set_zoom_profile(self, vmax: float, accel: float) -> None:
        with self._lock:
            self._zoom_vmax = max(50.0, min(8000.0, float(vmax)))
            self._zoom_accel = max(50.0, min(40000.0, float(accel)))

    def enqueue(self, cmd: dict[str, Any]) -> None:
        with self._lock:
            if not self._connected:
                return
        self._cmds.put(cmd)

    def _set_error(self, msg: str | None) -> None:
        with self._lock:
            self._last_error = msg

    def _mark_connected(self, adapter: str, interface: str) -> None:
        with self._lock:
            self._connected = True
            self._adapter = adapter
            self._interface = interface
            self._last_error = None

    def _apply_angles(self, yaw: float, roll: float, pitch: float) -> None:
        with self._lock:
            self._yaw = yaw
            self._roll = roll
            self._pitch = pitch
            self._last_rx = time.monotonic()
            self._rx_ok += 1

    def _apply_zoom(self, zoom: int) -> None:
        z = max(int(ZOOM_MIN), min(int(ZOOM_MAX), int(zoom)))
        with self._lock:
            if self._zoom_cmd is None:
                self._zoom_cmd = float(z)
                if self._zoom_target is None:
                    self._zoom_target = float(z)
                self._zoom_vel = 0.0
                self._zoom = z
            self._last_rx = time.monotonic()

    def _run(self, channel: str | None, interface: str | None) -> None:
        bus = None
        try:
            bus, iface, adapter = open_can_bus(channel, interface)
            while True:
                try:
                    self._cmds.get_nowait()
                except queue.Empty:
                    break
            self._mark_connected(adapter, iface)
            proto._send_packet(bus, proto.build_set_parameter_push(True))
            proto._send_packet(bus, proto.build_focus_get())
            with self._lock:
                self._tx_ok += 2
            self._loop(bus)
        except Exception as exc:
            self._set_error(str(exc))
        finally:
            if bus is not None:
                try:
                    bus.shutdown()
                except Exception:
                    pass
            with self._lock:
                self._connected = False
                self._held = None
                self._zoom_cmd = None
                self._zoom_vel = 0.0
                self._zoom_target = None
                self._zoom_sent = None
                self._zoom = None

    def _loop(self, bus: Any) -> None:
        reassembler = PacketReassembler()
        last_speed = 0.0
        last_poll = 0.0
        last_push = 0.0
        last_zoom = time.monotonic()
        sent_zero = True
        while not self._stop.is_set():
            now = time.monotonic()
            self._drain_cmds(bus)
            held = self._current_held(now)
            if held is not None:
                moving = abs(held.yaw) > 0.05 or abs(held.roll) > 0.05 or abs(held.pitch) > 0.05
                if moving and now - last_speed >= 1.0 / SPEED_HZ:
                    self._send(bus, proto.build_control_speed(held.yaw, held.roll, held.pitch))
                    last_speed = now
                    sent_zero = False
                elif not moving and not sent_zero:
                    self._send(bus, proto.build_control_speed(0.0, 0.0, 0.0))
                    sent_zero = True
            if now - last_zoom >= 1.0 / ZOOM_HZ:
                self._tick_zoom(bus, now - last_zoom)
                last_zoom = now
            if now - last_poll >= ANGLE_POLL_S and (now - last_push) > PUSH_STALE_S:
                self._send(bus, proto.build_obtain_gimbal_angle(0x01))
                last_poll = now
            msg = bus.recv(timeout=0.01)
            if msg is None:
                continue
            if msg.arbitration_id != proto.CAN_ID_RX:
                continue
            packet = reassembler.feed(bytes(msg.data or []))
            if packet is None:
                continue
            parsed = proto.validate_sdk_reply(packet)
            if parsed is None:
                continue
            cmd_set, cmd_id, ret_code, data = parsed
            if cmd_set == 0x0E and cmd_id == 0x08:
                angles = _push_angles(ret_code, data)
                if angles is not None:
                    self._apply_angles(*angles)
                    last_push = time.monotonic()
                continue
            if cmd_set == 0x0E and cmd_id == 0x02:
                gimbal = proto.parse_gimbal_reply(packet)
                if gimbal is not None:
                    self._apply_angles(*gimbal)
                continue
            if cmd_set == 0x0E and cmd_id == 0x12:
                zoom = proto.parse_focus_reply(packet)
                if zoom is not None:
                    self._apply_zoom(zoom)

    def _tick_zoom(self, bus: Any, dt: float) -> None:
        with self._lock:
            target = self._zoom_target
            pos = self._zoom_cmd
            vel = self._zoom_vel
            vmax = self._zoom_vmax
            accel = self._zoom_accel
            last_sent = self._zoom_sent
        if target is None or pos is None:
            return
        pos, vel, arrived = zoom_ramp_step(pos, vel, target, vmax, accel, dt)
        pos = max(ZOOM_MIN, min(ZOOM_MAX, pos))
        commanded = int(round(pos))
        with self._lock:
            self._zoom_cmd = pos
            self._zoom_vel = 0.0 if arrived else vel
            self._zoom = commanded
        if commanded != last_sent:
            self._send(bus, proto.build_focus_set(commanded))
            with self._lock:
                self._zoom_sent = commanded

    def _current_held(self, now: float) -> _HeldSpeed | None:
        with self._lock:
            held = self._held
            if held is None:
                return None
            if now - held.at > DEADMAN_S:
                self._held = None
                self._cmds.put({"op": "speed", "yaw": 0.0, "roll": 0.0, "pitch": 0.0, "once": True})
                return None
            return held

    def _drain_cmds(self, bus: Any) -> None:
        while True:
            try:
                cmd = self._cmds.get_nowait()
            except queue.Empty:
                return
            try:
                self._exec(bus, cmd)
            except Exception as exc:
                self._set_error(str(exc))

    def _exec(self, bus: Any, cmd: dict[str, Any]) -> None:
        op = cmd.get("op")
        if op == "speed":
            pkt = proto.build_control_speed(
                float(cmd.get("yaw", 0.0)),
                float(cmd.get("roll", 0.0)),
                float(cmd.get("pitch", 0.0)),
            )
            self._send(bus, pkt)
            return
        if op == "position":
            pkt = proto.build_control_position(
                float(cmd.get("yaw", 0.0)),
                float(cmd.get("roll", 0.0)),
                float(cmd.get("pitch", 0.0)),
                absolute=bool(cmd.get("absolute", True)),
                time_s=float(cmd.get("time_s", 0.4)),
            )
            self._send(bus, pkt)
            return
        if op == "zoom":
            self.set_zoom_target(int(cmd.get("position", 0)))
            return
        if op == "zoom_get":
            self._send(bus, proto.build_focus_get())
            return
        if op == "motor-calib":
            with self._lock:
                self._zoom_cmd = None
                self._zoom_vel = 0.0
                self._zoom_target = None
                self._zoom_sent = None
                self._zoom = None
            self._send(bus, proto.build_motor_calib())
            self._send(bus, proto.build_focus_get())
            return
        builders: dict[str, Any] = {
            "sleep": proto.build_sleep,
            "wake": proto.build_wake,
            "recenter": lambda: proto.build_recenter_selfie(0x01),
            "selfie": lambda: proto.build_recenter_selfie(0x02),
            "calibrate": proto.build_calibrate,
            "activetrack": proto.build_activetrack_toggle,
            "rec-start": proto.build_record_start,
            "rec-stop": proto.build_record_stop,
            "focus-center-start": proto.build_focus_center_start,
            "focus-center-stop": proto.build_focus_center_stop,
            "push-on": lambda: proto.build_set_parameter_push(True),
            "push-off": lambda: proto.build_set_parameter_push(False),
            "cam-cmd": proto.build_camera_cmd,
            "limit": proto.build_obtain_gimbal_limit_angle,
            "info-angle": lambda: proto.build_obtain_gimbal_angle(0x01),
        }
        builder = builders.get(str(op))
        if builder is None:
            self._set_error(f"unknown command {op}")
            return
        self._send(bus, builder())

    def _send(self, bus: Any, pkt: bytes) -> None:
        proto._send_packet(bus, pkt)
        with self._lock:
            self._tx_ok += 1


def _push_angles(flags: int, data: bytes) -> tuple[float, float, float] | None:
    if (flags & 0x01) and len(data) >= 6:
        angles = proto._angles_from_int16s(data, 0)
        if angles is not None:
            return angles
    if len(data) >= 7:
        return proto._angles_from_int16s(data, 1)
    return None

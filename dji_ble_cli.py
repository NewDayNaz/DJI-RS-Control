#!/usr/bin/env python3
"""BLE client for DJI RS gimbals using the V1 (DUML) protocol.

  python dji_ble_cli.py scan
  python dji_ble_cli.py listen
  python dji_ble_cli.py joystick --pitch 0.4 --hold 1.5
  python dji_ble_cli.py recenter
  python dji_ble_cli.py focus 90
  python dji_ble_cli.py selfie

Requires: bleak
  pip install bleak
"""
from __future__ import annotations

import argparse
import asyncio
import sys
import time

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic

import dji_v1 as v1

UUID_SERVICE = "0000fff0-0000-1000-8000-00805f9b34fb"
UUID_NOTIFY = "0000fff4-0000-1000-8000-00805f9b34fb"
UUID_WRITE = "0000fff5-0000-1000-8000-00805f9b34fb"

NAME_PREFIXES = ("DJI RS", "DJI RS ", "Ronin")


class RsBle:
    def __init__(self, client: BleakClient) -> None:
        self.client = client
        self.seq = 1
        self._rx_buf = bytearray()
        self.packets: list[v1.V1Packet] = []
        self._print_rx = False

    def next_seq(self) -> int:
        s = self.seq
        self.seq += 1
        if self.seq > 0xFFFD:
            self.seq = 2
        return s

    def _on_notify(self, _char: BleakGATTCharacteristic, data: bytearray) -> None:
        self._rx_buf.extend(data)
        frames, rest = v1.split_frames(bytes(self._rx_buf))
        self._rx_buf = bytearray(rest)
        for pkt in frames:
            self.packets.append(pkt)
            if self._print_rx:
                att = v1.parse_attitude(pkt)
                ver = v1.parse_version_ascii(pkt)
                extra = ""
                if att:
                    extra = f"  yaw={att[0]:+.1f} roll={att[1]:+.1f} pitch={att[2]:+.1f}"
                elif ver:
                    extra = f"  version={ver!r}"
                print(
                    f"  RX {pkt.key()} attr={pkt.attr:02X} seq={pkt.seq} "
                    f"data[{len(pkt.data)}]={pkt.data[:24].hex()}{extra}"
                )

    async def start(self, print_rx: bool = False) -> None:
        self._print_rx = print_rx
        await self.client.start_notify(UUID_NOTIFY, self._on_notify)
        mtu = getattr(self.client, "mtu_size", None)
        if mtu is not None:
            print(f"  ATT MTU {mtu}")

    async def write(self, raw: bytes) -> None:
        await self.client.write_gatt_char(UUID_WRITE, raw, response=False)

    async def handshake(self) -> None:
        for raw in v1.handshake_packets(self.seq):
            pkt = v1.unpack(raw)
            self.seq = pkt.seq + 1
            await self.write(raw)
            await asyncio.sleep(0.03)

    async def joystick(self, pitch: float, roll: float, yaw: float) -> None:
        p = v1.stick_to_axis(pitch)
        r = v1.stick_to_axis(roll)
        y = v1.stick_to_axis(yaw)
        await self.write(v1.cmd_joystick(p, r, y, self.next_seq()))

    async def joystick_hold(self, pitch: float, roll: float, yaw: float, seconds: float, hz: float = 10.0) -> None:
        period = 1.0 / hz
        deadline = time.monotonic() + seconds
        await self.joystick(0.0, 0.0, 0.0)
        await asyncio.sleep(period)
        while time.monotonic() < deadline:
            await self.joystick(pitch, roll, yaw)
            await asyncio.sleep(period)
        await self.joystick(0.0, 0.0, 0.0)

    async def recenter(self, selfie: bool = False) -> None:
        await self.write(v1.cmd_recenter(self.next_seq(), selfie=selfie))

    async def focus_percent(self, pct: float, seconds: float = 1.5, hz: float = 20.0) -> None:
        target = v1.focus_from_percent(pct)
        steps = max(1, int(seconds * hz))
        start = v1.FOCUS_MIN
        period = 1.0 / hz
        for i in range(steps + 1):
            pos = int(round(start + (target - start) * i / steps))
            await self.write(v1.cmd_focus(pos, self.next_seq()))
            await asyncio.sleep(period)

    # Experimental (unverified on RS 5). All called only under --experimental.

    async def rotate_angle(self, pitch: float, roll: float, yaw: float, duration: float, absolute: bool) -> None:
        if absolute:
            raw = v1.cmd_rotate_by_angle_abs(pitch, roll, yaw, self.next_seq(), duration_s=duration)
        else:
            raw = v1.cmd_rotate_by_angle(pitch, roll, yaw, self.next_seq(), duration_s=duration)
        await self.write(raw)

    async def rotate_speed(self, pitch: float, roll: float, yaw: float, seconds: float, hz: float = 20.0) -> None:
        period = 1.0 / hz
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            await self.write(v1.cmd_rotate_by_speed(pitch, roll, yaw, self.next_seq()))
            await asyncio.sleep(period)
        await self.write(v1.cmd_rotate_by_speed(0.0, 0.0, 0.0, self.next_seq(), enable=False))

    async def sleep(self, suspend: bool) -> None:
        await self.write(v1.cmd_sleep(self.next_seq(), suspend=suspend))

    async def gimbal_mode(self, mode: int) -> None:
        await self.write(v1.cmd_gimbal_mode(mode, self.next_seq()))

    async def autotune(self, sub: int) -> None:
        await self.write(v1.cmd_autotune(self.next_seq(), sub=sub))

    async def stiffness(self, axis: int, value: int) -> None:
        await self.write(v1.cmd_stiffness(axis, value, self.next_seq()))

    async def record(self, start: bool) -> None:
        await self.write(v1.cmd_camera_record(self.next_seq(), start=start))


async def find_device(address: str | None) -> str:
    if address:
        return address
    print("Scanning for DJI RS…")
    devices = await BleakScanner.discover(timeout=8.0)
    matches = []
    for d in devices:
        name = d.name or ""
        if name.startswith(NAME_PREFIXES) or name.startswith("DJI RS"):
            matches.append(d)
            print(f"  {d.address}  {name}")
    if not matches:
        print("No DJI RS advertisement found. Put the gimbal in pairing mode and retry.")
        sys.exit(1)
    return matches[0].address


async def with_client(args: argparse.Namespace, print_rx: bool) -> RsBle:
    address = await find_device(args.address)
    print(f"Connecting {address}…")
    client = BleakClient(address)
    await client.connect()
    rs = RsBle(client)
    await rs.start(print_rx=print_rx)
    await rs.handshake()
    print("Handshake sent.")
    return rs


async def cmd_scan(_args: argparse.Namespace) -> None:
    devices = await BleakScanner.discover(timeout=8.0)
    for d in devices:
        name = d.name or ""
        mark = " *" if name.startswith("DJI") or "RS" in name else ""
        print(f"{d.address}  rssi={d.rssi}  {name}{mark}")


async def cmd_listen(args: argparse.Namespace) -> None:
    rs = await with_client(args, print_rx=True)
    try:
        print(f"Listening {args.seconds:.0f}s…")
        await asyncio.sleep(args.seconds)
    finally:
        await rs.client.disconnect()


async def cmd_joystick(args: argparse.Namespace) -> None:
    rs = await with_client(args, print_rx=args.verbose)
    try:
        await rs.joystick_hold(args.pitch, args.roll, args.yaw, args.hold)
        await asyncio.sleep(0.3)
    finally:
        await rs.client.disconnect()


async def cmd_recenter(args: argparse.Namespace) -> None:
    rs = await with_client(args, print_rx=args.verbose)
    try:
        await rs.recenter(selfie=False)
        await asyncio.sleep(1.0)
    finally:
        await rs.client.disconnect()


async def cmd_selfie(args: argparse.Namespace) -> None:
    rs = await with_client(args, print_rx=args.verbose)
    try:
        await rs.recenter(selfie=True)
        await asyncio.sleep(1.0)
    finally:
        await rs.client.disconnect()


async def cmd_focus(args: argparse.Namespace) -> None:
    rs = await with_client(args, print_rx=args.verbose)
    try:
        await rs.focus_percent(args.percent)
        await asyncio.sleep(0.3)
    finally:
        await rs.client.disconnect()


def _require_experimental(args: argparse.Namespace) -> None:
    if not getattr(args, "experimental", False):
        name = args.cmd
        raise SystemExit(
            f"'{name}' is unverified on RS 5 (public-DUML/Osmo candidate "
            f"{v1.EXPERIMENTAL.get(name, '?')}). It may move the gimbal or change "
            f"settings. Re-run with --experimental to send it. Watch for an ACK or "
            f"error on the notification after the write."
        )


async def _run_experimental(args: argparse.Namespace, body) -> None:
    _require_experimental(args)
    rs = await with_client(args, print_rx=True)
    try:
        await body(rs)
        # give the gimbal a moment to ACK / report an error
        await asyncio.sleep(args.settle)
    finally:
        await rs.client.disconnect()


async def cmd_rotate_angle(args: argparse.Namespace) -> None:
    await _run_experimental(
        args,
        lambda rs: rs.rotate_angle(args.pitch, args.roll, args.yaw, args.duration, args.absolute),
    )


async def cmd_rotate_speed(args: argparse.Namespace) -> None:
    await _run_experimental(
        args,
        lambda rs: rs.rotate_speed(args.pitch, args.roll, args.yaw, args.hold),
    )


async def cmd_sleep(args: argparse.Namespace) -> None:
    await _run_experimental(args, lambda rs: rs.sleep(suspend=not args.wake))


async def cmd_mode(args: argparse.Namespace) -> None:
    mode = {"lock": v1.GIMBAL_MODE_LOCK, "follow": v1.GIMBAL_MODE_FOLLOW, "fpv": v1.GIMBAL_MODE_FPV}[args.name]
    await _run_experimental(args, lambda rs: rs.gimbal_mode(mode))


async def cmd_autotune(args: argparse.Namespace) -> None:
    await _run_experimental(args, lambda rs: rs.autotune(args.sub))


async def cmd_stiffness(args: argparse.Namespace) -> None:
    axis = {"pitch": 0, "roll": 1, "yaw": 2}[args.axis]
    await _run_experimental(args, lambda rs: rs.stiffness(axis, args.value))


async def cmd_record(args: argparse.Namespace) -> None:
    await _run_experimental(args, lambda rs: rs.record(start=not args.stop))


def main() -> None:
    parser = argparse.ArgumentParser(description="DJI RS BLE V1 controller")
    parser.add_argument("-a", "--address", help="BLE address (otherwise scan)")
    parser.add_argument("-v", "--verbose", action="store_true", help="print RX packets")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("scan", help="list nearby BLE devices")

    p_listen = sub.add_parser("listen", help="connect, handshake, print notifications")
    p_listen.add_argument("--seconds", type=float, default=8.0)

    p_joy = sub.add_parser("joystick", help="hold a stick deflection then release")
    p_joy.add_argument("--pitch", type=float, default=0.0, help="-1 down .. +1 up")
    p_joy.add_argument("--roll", type=float, default=0.0, help="-1 left .. +1 right")
    p_joy.add_argument("--yaw", type=float, default=0.0, help="-1 left .. +1 right")
    p_joy.add_argument("--hold", type=float, default=1.5)

    sub.add_parser("recenter", help="recenter once")
    sub.add_parser("selfie", help="selfie / flip once (inferred 0x04/0x4C mode 2)")

    p_focus = sub.add_parser("focus", help="ramp LiDAR/focus motor to a percent")
    p_focus.add_argument("percent", type=float)

    # --- Experimental (unverified on RS 5) ---------------------------------
    def exp(p: argparse.ArgumentParser) -> argparse.ArgumentParser:
        p.add_argument("--experimental", action="store_true",
                       help="required: acknowledge this command is unverified")
        p.add_argument("--settle", type=float, default=1.5,
                       help="seconds to listen for an ACK/error after writing")
        return p

    p_ra = exp(sub.add_parser("rotate-angle", help="candidate go-to-angle 0x04/0x0A (deg)"))
    p_ra.add_argument("--pitch", type=float, default=0.0)
    p_ra.add_argument("--roll", type=float, default=0.0)
    p_ra.add_argument("--yaw", type=float, default=0.0)
    p_ra.add_argument("--duration", type=float, default=0.0, help="seconds to reach target")
    p_ra.add_argument("--absolute", action="store_true", help="use 0x04/0x14 absolute form")

    p_rs = exp(sub.add_parser("rotate-speed", help="candidate speed 0x04/0x0C (deg/s)"))
    p_rs.add_argument("--pitch", type=float, default=0.0)
    p_rs.add_argument("--roll", type=float, default=0.0)
    p_rs.add_argument("--yaw", type=float, default=0.0)
    p_rs.add_argument("--hold", type=float, default=1.0)

    p_sl = exp(sub.add_parser("sleep", help="candidate suspend 0x04/0x0D (Phantom magic)"))
    p_sl.add_argument("--wake", action="store_true", help="send resume instead of suspend")

    p_md = exp(sub.add_parser("mode", help="candidate follow-mode 0x04/0x4C (Osmo values)"))
    p_md.add_argument("name", choices=("lock", "follow", "fpv"))

    p_at = exp(sub.add_parser("autotune", help="candidate AutoTune 0x04/0x08 (watch 0x04/0x30)"))
    p_at.add_argument("--sub", type=lambda x: int(x, 0), default=0x01, help="sub-command byte")

    p_st = exp(sub.add_parser("stiffness", help="candidate stiffness 0x04/0x0F (weak candidate)"))
    p_st.add_argument("axis", choices=("pitch", "roll", "yaw"))
    p_st.add_argument("value", type=int)

    p_rc = exp(sub.add_parser("record", help="candidate camera record 0x0D (very speculative)"))
    p_rc.add_argument("--stop", action="store_true", help="stop instead of start")

    args = parser.parse_args()
    dispatch = {
        "scan": cmd_scan,
        "listen": cmd_listen,
        "joystick": cmd_joystick,
        "recenter": cmd_recenter,
        "selfie": cmd_selfie,
        "focus": cmd_focus,
        "rotate-angle": cmd_rotate_angle,
        "rotate-speed": cmd_rotate_speed,
        "sleep": cmd_sleep,
        "mode": cmd_mode,
        "autotune": cmd_autotune,
        "stiffness": cmd_stiffness,
        "record": cmd_record,
    }
    asyncio.run(dispatch[args.cmd](args))


if __name__ == "__main__":
    main()

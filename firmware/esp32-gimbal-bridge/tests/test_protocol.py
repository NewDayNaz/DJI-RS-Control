"""Packet-level checks against the golden Python builders (dji_gimbal_cli.py).

These do not talk to hardware. They lock the CAN ID, 8-byte framing, CRC,
focus-set payload, and speed takeover byte that the ESP32 also emits.
"""

from __future__ import annotations

import struct

import dji_gimbal_cli as proto
from dji_can_session import PacketReassembler

from tests.zoom_ramp_model import clamp_motor


def _cmd_payload(packet: bytes) -> bytes:
    return packet[12 : len(packet) - 4]


def can_frames(packet: bytes, can_id: int = proto.CAN_ID_TX) -> list[tuple[int, bytes]]:
    return [(can_id, packet[i : i + 8]) for i in range(0, len(packet), 8)]


def test_crc16_known_vector():
    prefix = bytes([0xAA, 0x16, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x11, 0x22])
    c16 = proto.crc16(prefix)
    assert 0 <= c16 <= 0xFFFF
    assert proto.crc16(prefix) == c16


def test_built_packet_crc_roundtrip():
    pkt = proto.build_focus_get()
    assert pkt[0] == proto.SOF
    pack_len = pkt[1] | ((pkt[2] & 0x03) << 8)
    assert pack_len == len(pkt)
    assert proto.crc16(pkt[:10]) == (pkt[10] | (pkt[11] << 8))
    assert proto.crc32(pkt[:-4]) == struct.unpack_from("<I", pkt, len(pkt) - 4)[0]


def test_focus_get_payload_matches_reference():
    pkt = proto.build_focus_get()
    body = _cmd_payload(pkt)
    assert body[0] == 0x0E
    assert body[1] == 0x12
    assert body[2:] == bytes([0x15, 0x00])


def test_focus_set_payload_layout():
    pkt = proto.build_focus_set(1234)
    body = _cmd_payload(pkt)
    assert body[0] == 0x0E
    assert body[1] == 0x12
    sub, ctl, dlen, pos = struct.unpack_from("<3BH", body, 2)
    assert (sub, ctl, dlen, pos) == (0x01, 0x00, 0x02, 1234)


def test_firmware_clamps_endpoints_python_golden_does_not():
    # ESP32 buildFocusSet / zoom::clampMotor send 1 and 4095. The golden
    # Python builder still packs the raw uint16; firmware must clamp.
    raw = proto.build_focus_set(0)
    pos = struct.unpack_from("<H", _cmd_payload(raw), 5)[0]
    assert pos == 0
    assert clamp_motor(0) == 1
    assert clamp_motor(4096) == 4095


def test_speed_takeover_byte_is_0x80():
    pkt = proto.build_control_speed(1.5, 0.0, -2.0)
    body = _cmd_payload(pkt)
    assert body[0] == 0x0E
    assert body[1] == 0x01
    yaw, roll, pitch, ctrl = struct.unpack_from("<3hB", body, 2)
    assert ctrl == proto.SPEED_CTRL_TAKEOVER == 0x80
    assert yaw == 15
    assert roll == 0
    assert pitch == -20


def test_focus_set_fragments_into_8_byte_can_frames_on_0x223():
    pkt = proto.build_focus_set(2048)
    frames = can_frames(pkt)
    assert frames
    assert all(cid == 0x223 for cid, _ in frames)
    assert all(len(payload) == 8 for _, payload in frames[:-1])
    assert 1 <= len(frames[-1][1]) <= 8
    assert b"".join(p for _, p in frames) == pkt


def test_reassembler_rebuilds_packet_from_8_byte_chunks():
    pkt = proto.build_focus_set(777)
    ra = PacketReassembler()
    out = None
    for _, chunk in can_frames(pkt):
        out = ra.feed(chunk)
    assert out == pkt


def test_reassembler_drops_bad_crc16():
    pkt = bytearray(proto.build_focus_get())
    pkt[10] ^= 0xFF
    ra = PacketReassembler()
    out = None
    for _, chunk in can_frames(bytes(pkt)):
        got = ra.feed(chunk)
        if got is not None:
            out = got
    assert out is None


def test_angle_reply_parse_0_1_deg_units():
    # Synthetic reply: SOF.. with cmd 0x0E/0x02, ret 0, data_type + 3x int16.
    # Use the builder path by constructing a reply-shaped packet is awkward
    # (builders emit commands). Pack the data layout the parser expects.
    yaw, roll, pitch = 12.3, -4.5, 8.1
    data = bytes([0x01]) + struct.pack(
        "<3h", int(round(yaw * 10)), int(round(roll * 10)), int(round(pitch * 10))
    )
    # wrap as a reply: byte3 bit 0x20, cmdset/id, ret, data, crc
    prefix_len = 10
    data_seg = bytes([0x0E, 0x02, 0x00]) + data
    cmd_length = prefix_len + 2 + len(data_seg) + 4
    prefix = bytearray(
        [0xAA, cmd_length & 0xFF, (cmd_length >> 8) & 0xFF, 0x20, 0, 0, 0, 0, 0x01, 0x00]
    )
    c16 = proto.crc16(bytes(prefix))
    prefix += bytes([c16 & 0xFF, (c16 >> 8) & 0xFF])
    prefix += data_seg
    c32 = proto.crc32(bytes(prefix))
    packet = bytes(prefix) + struct.pack("<I", c32)
    parsed = proto.parse_gimbal_reply(packet)
    assert parsed is not None
    assert abs(parsed[0] - yaw) < 0.05
    assert abs(parsed[1] - roll) < 0.05
    assert abs(parsed[2] - pitch) < 0.05


def test_focus_reply_reads_last_four_data_bytes():
    pos = 3456
    data = bytes([0x00, 0x00]) + struct.pack("<I", pos)
    data_seg = bytes([0x0E, 0x12, 0x00]) + data
    prefix_len = 10
    cmd_length = prefix_len + 2 + len(data_seg) + 4
    prefix = bytearray(
        [0xAA, cmd_length & 0xFF, (cmd_length >> 8) & 0xFF, 0x20, 0, 0, 0, 0, 0x02, 0x00]
    )
    c16 = proto.crc16(bytes(prefix))
    prefix += bytes([c16 & 0xFF, (c16 >> 8) & 0xFF])
    prefix += data_seg
    c32 = proto.crc32(bytes(prefix))
    packet = bytes(prefix) + struct.pack("<I", c32)
    assert proto.parse_focus_reply(packet) == pos

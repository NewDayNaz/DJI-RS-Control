#!/usr/bin/env python3
"""
DJI gimbal/CAN console

Uses the DJI R SDK external protocol (SOF 0xAA, CAN 0x223 Tx / 0x222 Rx, 1 Mbps).
See docs/DJI_R_SDK_Protocol.md for packet layouts.
Implements gimbal CmdSet 0x0E (angle, position, speed, limits, stiffness, user params,
parameter push, sleep/wake 0x0C, recenter/selfie, ActiveTrack, focus motor) and camera
CmdSet 0x0D (record, focus-center). Handles push (0x08, 0x10) during recv.

  python dji_gimbal_cli.py COM6                        # stream attitude (default)
  python dji_gimbal_cli.py COM6 -c version             # module version
  python dji_gimbal_cli.py COM6 -c position 0 0 -90    # set position (yaw roll pitch °)
  python dji_gimbal_cli.py COM6 -c speed 10 0 -5       # set speed (yaw roll pitch °/s)
  python dji_gimbal_cli.py COM6 -c recenter            # recenter gimbal
  python dji_gimbal_cli.py COM6 -c sleep               # sleep (0x0E/0x0C 23 01 01)
  python dji_gimbal_cli.py COM6 -c rec-start            # camera record start (0x0D/0x00 03 00)
  python dji_gimbal_cli.py COM6 -c listen               # enable push and print pushes

Requires: python-can, pyserial
  pip install python-can pyserial

Use -d/--debug to print every received CAN frame for troubleshooting.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from collections.abc import Callable

# CRC-16: poly 0x8005, init 0x3aa3 (reflected), ref in/out. Matches repo custom_crc16.
CRC16_TABLE = (
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
)


def crc16_update(crc: int, data: bytes) -> int:
    for b in data:
        tbl_idx = (crc ^ b) & 0xFF
        crc = (CRC16_TABLE[tbl_idx] ^ (crc >> 8)) & 0xFFFF
    return crc


def crc16(data: bytes) -> int:
    return crc16_update(0x3AA3, data)


# CRC-32: poly 0x04C11DB7, init 0x00003aa3, ref in/out. Matches repo custom_crc32.
CRC32_TABLE = (
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
    0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988, 0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
    0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
    0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172, 0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
    0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
    0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
    0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
    0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E, 0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
    0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
    0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0, 0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
    0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
    0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A, 0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
    0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
    0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC, 0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
    0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
    0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
    0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236, 0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
    0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
    0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38, 0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
    0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
    0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2, 0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
    0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
    0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
    0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94, 0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D,
)


def crc32_update(crc: int, data: bytes) -> int:
    for b in data:
        tbl_idx = (crc ^ b) & 0xFF
        crc = (CRC32_TABLE[tbl_idx] ^ (crc >> 8)) & 0xFFFFFFFF
    return crc


def crc32(data: bytes) -> int:
    return crc32_update(0x00003AA3, data)


# Sequence number for SDK packets (LE in packet).
_seq = 0x2210
CMD_TYPE_REPLY_REQUIRED = 0x03


def next_seq() -> int:
    global _seq
    if _seq >= 0xFFFD:
        _seq = 0x0002
    _seq += 1
    return _seq


def build_sdk_packet(cmd_set: int, cmd_id: int, cmd_data: bytes = b"", cmd_type: int = CMD_TYPE_REPLY_REQUIRED) -> bytes:
    """Build a full DJI R SDK packet (SOF 0xAA, CRCs, CmdSet/CmdID/CmdData)."""
    prefix_len = 10
    crc16_len = 2
    data_seg = bytes([cmd_set, cmd_id]) + cmd_data
    data_len = len(data_seg)
    crc32_len = 4
    cmd_length = prefix_len + crc16_len + data_len + crc32_len

    seq = next_seq()
    prefix = bytearray([
        0xAA,
        cmd_length & 0xFF,
        (cmd_length >> 8) & 0xFF,
        cmd_type,
        0x00,   # ENC
        0x00, 0x00, 0x00,  # RES
        seq & 0xFF,
        (seq >> 8) & 0xFF,
    ])
    assert len(prefix) == 10
    c16 = crc16(bytes(prefix))
    prefix.append(c16 & 0xFF)
    prefix.append((c16 >> 8) & 0xFF)
    prefix.extend(data_seg)
    c32 = crc32(bytes(prefix))
    prefix.append(c32 & 0xFF)
    prefix.append((c32 >> 8) & 0xFF)
    prefix.append((c32 >> 16) & 0xFF)
    prefix.append((c32 >> 24) & 0xFF)
    return bytes(prefix)


def build_obtain_gimbal_angle(cmd_data_byte: int = 0x01) -> bytes:
    """Build Obtain gimbal angle packet (CmdSet 0x0E, CmdID 0x02). cmd_data_byte: 0x01=attitude, 0x02=joint."""
    return build_sdk_packet(0x0E, 0x02, bytes([cmd_data_byte]))


def build_obtain_module_version(device_id: int = 0x00000001) -> bytes:
    """Build Obtain module version (CmdSet 0x0E, CmdID 0x09). device_id: 0x01 = DJI R SDK."""
    return build_sdk_packet(0x0E, 0x09, struct.pack("<I", device_id & 0xFFFFFFFF))


def build_obtain_gimbal_limit_angle() -> bytes:
    """Build Obtain gimbal limit angle (CmdSet 0x0E, CmdID 0x04)."""
    return build_sdk_packet(0x0E, 0x04, b"")


def build_obtain_motor_stiffness() -> bytes:
    """Build Obtain motor stiffness (CmdSet 0x0E, CmdID 0x06)."""
    return build_sdk_packet(0x0E, 0x06, b"")


def build_obtain_gimbal_user_params() -> bytes:
    """Build Obtain gimbal user parameters (CmdSet 0x0E, CmdID 0x0B)."""
    return build_sdk_packet(0x0E, 0x0B, b"")


def build_set_parameter_push(enable: bool) -> bytes:
    """Build Set parameter push (CmdSet 0x0E, CmdID 0x07). enable: True=0x01, False=0x00."""
    return build_sdk_packet(0x0E, 0x07, bytes([0x01 if enable else 0x00]))


def build_recenter_selfie(mode: int) -> bytes:
    """Build Recenter/Selfie (CmdSet 0x0E, CmdID 0x0E). mode: 0x01=Recenter once, 0x02=Selfie once."""
    return build_sdk_packet(0x0E, 0x0E, bytes([0xFE, mode]))


def build_activetrack_toggle() -> bytes:
    """Build ActiveTrack toggle (CmdSet 0x0E, CmdID 0x11). CmdData 0x03 = toggle start/stop."""
    return build_sdk_packet(0x0E, 0x11, bytes([0x03]))


# Sleep/wake: gimbal cmd_set 0x0E cmd_id 0x0C (not recenter 0x0E/FE 01).
# Record and focus-center: camera cmd_set 0x0D cmd_id 0x00.

def build_sleep() -> bytes:
    """Sleep gimbal (CmdSet 0x0E, CmdID 0x0C, data 23 01 01)."""
    return build_sdk_packet(0x0E, 0x0C, bytes([0x23, 0x01, 0x01]))


def build_wake() -> bytes:
    """Wake gimbal (CmdSet 0x0E, CmdID 0x0C, data 23 01 00)."""
    return build_sdk_packet(0x0E, 0x0C, bytes([0x23, 0x01, 0x00]))


def build_record_start() -> bytes:
    """Camera record start (CmdSet 0x0D, CmdID 0x00, data 03 00)."""
    return build_sdk_packet(0x0D, 0x00, bytes([0x03, 0x00]))


def build_record_stop() -> bytes:
    """Camera record stop (CmdSet 0x0D, CmdID 0x00, data 04 00)."""
    return build_sdk_packet(0x0D, 0x00, bytes([0x04, 0x00]))


def build_focus_center_start() -> bytes:
    """Focus center start (CmdSet 0x0D, CmdID 0x00, data 05 00)."""
    return build_sdk_packet(0x0D, 0x00, bytes([0x05, 0x00]))


def build_focus_center_stop() -> bytes:
    """Focus center stop (CmdSet 0x0D, CmdID 0x00, data 0B 00)."""
    return build_sdk_packet(0x0D, 0x00, bytes([0x0B, 0x00]))


PROBE_COMMANDS: dict[str, tuple[str, Callable[[], bytes]]] = {
    "sleep": ("Sleep gimbal", build_sleep),
    "wake": ("Wake gimbal", build_wake),
    "rec-start": ("Camera record start", build_record_start),
    "rec-stop": ("Camera record stop", build_record_stop),
    "focus-center-start": ("Focus center start", build_focus_center_start),
    "focus-center-stop": ("Focus center stop", build_focus_center_stop),
}


def build_control_position(
    yaw_deg: float,
    roll_deg: float,
    pitch_deg: float,
    *,
    absolute: bool = True,
    time_s: float = 0.2,
    yaw_valid: bool = True,
    roll_valid: bool = True,
    pitch_valid: bool = True,
) -> bytes:
    """Build Control gimbal position (CmdSet 0x0E, CmdID 0x00). Angles in degrees; stored as 0.1° (int16). ctrl_byte: bit0=absolute(1)/incremental(0); bits 1,2,3 = yaw/roll/pitch invalid (1=invalid). time_s in seconds (stored as 0.1s units, 1 byte)."""
    yaw = int(round(yaw_deg * 10))
    roll = int(round(roll_deg * 10))
    pitch = int(round(pitch_deg * 10))
    ctrl = 0x01 if absolute else 0x00
    if not yaw_valid:
        ctrl |= 0x02
    if not roll_valid:
        ctrl |= 0x04
    if not pitch_valid:
        ctrl |= 0x08
    time_byte = max(0, min(255, int(round(time_s * 10))))
    return build_sdk_packet(0x0E, 0x00, struct.pack("<3hBB", yaw, roll, pitch, ctrl, time_byte))


def build_control_speed(
    yaw_degs: float,
    roll_degs: float,
    pitch_degs: float
) -> bytes:
    """Build Handheld Gimbal Speed Control (CmdSet 0x0E, CmdID 0x01). Rates in deg/s; stored as 0.1 deg/s (int16, range -3600 to +3600). ctrl_byte 0x88 = take over speed control."""
    yaw_x10 = max(-3600, min(3600, int(round(yaw_degs * 10))))
    roll_x10 = max(-3600, min(3600, int(round(roll_degs * 10))))
    pitch_x10 = max(-3600, min(3600, int(round(pitch_degs * 10))))
    return build_sdk_packet(0x0E, 0x01, struct.pack("<3hB", yaw_x10, roll_x10, pitch_x10, 0x88))


def build_focus_set(
    position: int,
    cmd_sub_id: int = 0x01,
    ctl_type: int = 0x00,
    data_length: int = 0x02,
) -> bytes:
    """
    Build focus motor control command (CmdSet 0x0E, CmdID 0x12).

    Payload layout (from firmware analysis / reference controller):
      - cmd_sub_id: 0x01
      - ctl_type:   0x00
      - data_len:   0x02 (bytes)
      - position:   uint16, range 0–4096 (absolute focus position)
    """
    pos_clamped = max(0, min(0xFFFF, int(position)))
    payload = struct.pack("<3BH", cmd_sub_id & 0xFF, ctl_type & 0xFF, data_length & 0xFF, pos_clamped)
    return build_sdk_packet(0x0E, 0x12, payload)


def build_focus_get() -> bytes:
    """
    Build focus motor position query (CmdSet 0x0E, CmdID 0x12).

    Payload bytes [0x15, 0x00] match the reference `getFocPosData` implementation.
    """
    payload = struct.pack("<2B", 0x15, 0x00)
    return build_sdk_packet(0x0E, 0x12, payload)


CAN_ID_TX = 0x223
CAN_ID_RX = 0x222
SOF = 0xAA

# Return codes (PDF §2.3.2)
RET_SUCCESS = 0x00
RET_PARSE_ERROR = 0x01
RET_EXEC_FAIL = 0x02
RET_UNDEFINED = 0xFF

RET_CODE_NAMES: dict[int, str] = {
    RET_SUCCESS: "success",
    RET_PARSE_ERROR: "command parse error",
    RET_EXEC_FAIL: "command execution failed",
    RET_UNDEFINED: "undefined error",
}


def return_code_str(code: int) -> str:
    """Human-readable return code for device replies."""
    return RET_CODE_NAMES.get(code, "0x{:02X}".format(code))


def print_sdk_reply(reply: bytes | None) -> None:
    """Print cmd_set, cmd_id, ret_code, and payload hex from a 0x222 SDK reply."""
    if reply is None:
        print("No reply on 0x222.")
        return
    parsed = validate_sdk_reply(reply)
    if parsed is None:
        print("Reply failed validation.")
        return
    cmd_set, cmd_id, ret_code, data = parsed
    payload = data.hex() if data else "(empty)"
    print(
        "reply  cmd_set=0x{:02X}  cmd_id=0x{:02X}  ret=0x{:02X} ({})  payload_len={}  hex={}".format(
            cmd_set, cmd_id, ret_code, return_code_str(ret_code), len(data), payload
        )
    )


def validate_sdk_reply(packet: bytes) -> tuple[int, int, int, bytes] | None:
    """
    Validate SDK reply packet and return (cmd_set, cmd_id, return_code, data) or None.
    data is the reply payload starting at byte 14 (return_code is at 14, rest follows).
    """
    if len(packet) < 16:
        return None
    if packet[0] != SOF:
        return None
    if (packet[3] & 0x20) == 0:
        return None  # not a reply
    pack_len = packet[1] | ((packet[2] & 0x03) << 8)
    if len(packet) != pack_len:
        return None
    if crc16(packet[:10]) != (packet[10] | (packet[11] << 8)):
        return None
    if crc32(packet[:-4]) != struct.unpack_from("<I", packet, len(packet) - 4)[0]:
        return None
    cmd_set = packet[12]
    cmd_id = packet[13]
    ret_code = packet[14]
    data = packet[15 : len(packet) - 4]
    return (cmd_set, cmd_id, ret_code, data)


def parse_gimbal_reply(packet: bytes) -> tuple[float, float, float] | None:
    """Parse reply to Obtain gimbal info (0x0E, 0x02). Returns (yaw_deg, roll_deg, pitch_deg) or None."""
    r = validate_sdk_reply(packet)
    if r is None:
        return None
    cmd_set, cmd_id, ret_code, data = r
    if cmd_set != 0x0E or cmd_id != 0x02 or ret_code != RET_SUCCESS:
        return None
    if len(data) < 7:  # data_type(1) + yaw(2) + roll(2) + pitch(2)
        return None
    # data = data_type(1), yaw(2), roll(2), pitch(2)
    yaw = struct.unpack_from("<h", data, 1)[0]
    roll = struct.unpack_from("<h", data, 3)[0]
    pitch = struct.unpack_from("<h", data, 5)[0]
    return (yaw * 0.1, roll * 0.1, pitch * 0.1)


def parse_focus_reply(packet: bytes) -> int | None:
    """
    Parse reply to focus motor command (CmdSet 0x0E, CmdID 0x12).

    Empirically, the focus position is encoded in the last 4 bytes of the DATA
    segment as a little-endian uint32; valid values are typically 0–4096.
    """
    r = validate_sdk_reply(packet)
    if r is None:
        return None
    cmd_set, cmd_id, ret_code, data = r
    if cmd_set != 0x0E or cmd_id != 0x12 or ret_code != RET_SUCCESS:
        return None
    if len(data) < 4:
        return None
    value = struct.unpack_from("<I", data, len(data) - 4)[0]
    return value


def parse_module_version_reply(packet: bytes) -> tuple[int, int, tuple[int, int, int, int]] | None:
    """Parse reply to Obtain module version (0x0E, 0x09). Returns (device_id, version_uint32, (a,b,c,d)) or None."""
    r = validate_sdk_reply(packet)
    if r is None:
        return None
    cmd_set, cmd_id, ret_code, data = r
    if cmd_set != 0x0E or cmd_id != 0x09 or ret_code != RET_SUCCESS:
        return None
    if len(data) < 8:
        return None
    device_id = struct.unpack_from("<I", data, 0)[0]
    ver = struct.unpack_from("<I", data, 4)[0]
    # version 0xAABBCCDD = AA.BB.CC.DD
    v = (ver >> 24, (ver >> 16) & 0xFF, (ver >> 8) & 0xFF, ver & 0xFF)
    return (device_id, ver, v)


def parse_limit_angle_reply(packet: bytes) -> dict[str, float] | None:
    """Parse reply to Obtain gimbal limit angle (0x0E, 0x04). Returns dict with yaw/roll/pitch min/max or None."""
    r = validate_sdk_reply(packet)
    if r is None:
        return None
    cmd_set, cmd_id, ret_code, data = r
    if cmd_set != 0x0E or cmd_id != 0x04:
        return None
    if ret_code != RET_SUCCESS:
        return None
    # Payload: either 6 x int16 (12 bytes) or 1 byte prefix + 6 x int16 (13 bytes). Units 0.1°.
    def parse_6_int16(d: bytes, offset: int) -> dict[str, float] | None:
        if offset + 12 > len(d):
            return None
        try:
            return {
                "yaw_min": struct.unpack_from("<h", d, offset + 0)[0] * 0.1,
                "yaw_max": struct.unpack_from("<h", d, offset + 2)[0] * 0.1,
                "roll_min": struct.unpack_from("<h", d, offset + 4)[0] * 0.1,
                "roll_max": struct.unpack_from("<h", d, offset + 6)[0] * 0.1,
                "pitch_min": struct.unpack_from("<h", d, offset + 8)[0] * 0.1,
                "pitch_max": struct.unpack_from("<h", d, offset + 10)[0] * 0.1,
            }
        except Exception:
            return None

    if len(data) >= 12:
        res = parse_6_int16(data, 0)
        if res is not None:
            return res
    if len(data) >= 13:
        res = parse_6_int16(data, 1)
        if res is not None:
            return res
    return None


def limit_angle_reply_raw(packet: bytes) -> tuple[int, bytes] | None:
    """Return (return_code, data) for limit reply so caller can print raw when parse fails."""
    r = validate_sdk_reply(packet)
    if r is None:
        return None
    cmd_set, cmd_id, ret_code, data = r
    if cmd_set != 0x0E or cmd_id != 0x04:
        return None
    return (ret_code, data)


def parse_motor_stiffness_reply(packet: bytes) -> dict[str, int | str] | None:
    """Parse reply to Obtain motor stiffness (0x0E, 0x06). Returns dict with ret_code, raw_len, hex (any return code)."""
    r = validate_sdk_reply(packet)
    if r is None:
        return None
    cmd_set, cmd_id, ret_code, data = r
    if cmd_set != 0x0E or cmd_id != 0x06:
        return None
    return {"ret_code": ret_code, "raw_len": len(data), "hex": data.hex()}


def parse_user_params_reply(packet: bytes) -> dict[str, int | str] | None:
    """Parse reply to Obtain gimbal user parameters (0x0E, 0x0B). Returns dict with ret_code, raw_len, hex (any return code)."""
    r = validate_sdk_reply(packet)
    if r is None:
        return None
    cmd_set, cmd_id, ret_code, data = r
    if cmd_set != 0x0E or cmd_id != 0x0B:
        return None
    return {"ret_code": ret_code, "raw_len": len(data), "hex": data.hex()}


def try_handle_push(packet: bytes) -> bool:
    """
    If packet is a gimbal push (CmdSet 0x0E, CmdID 0x08 or 0x10), print it and return True.
    Otherwise return False. Used in receive loop so pushes are displayed during streaming.
    """
    r = validate_sdk_reply(packet)
    if r is None:
        return False
    cmd_set, cmd_id, ret_code, data = r
    if cmd_set != 0x0E:
        return False
    if cmd_id == 0x08:
        # Push gimbal parameters
        if len(data) >= 7:
            try:
                # Same layout as angle reply: data_type(1), yaw(2), roll(2), pitch(2)
                yaw = struct.unpack_from("<h", data, 1)[0] * 0.1
                roll = struct.unpack_from("<h", data, 3)[0] * 0.1
                pitch = struct.unpack_from("<h", data, 5)[0] * 0.1
                print("[push] gimbal params: yaw={:.1f}° roll={:.1f}° pitch={:.1f}°".format(yaw, roll, pitch))
            except Exception:
                print("[push] gimbal params (0x08): len={} hex={}".format(len(data), data.hex()))
        else:
            print("[push] gimbal params (0x08): len={} hex={}".format(len(data), data.hex()))
        return True
    if cmd_id == 0x10:
        # Auto calibration status push
        print("[push] auto calibration status (0x10): ret=0x{:02X} len={} hex={}".format(ret_code, len(data), data.hex()))
        return True
    return False


def _send_packet(bus, pkt: bytes) -> None:
    import can
    for i in range(0, len(pkt), 8):
        chunk = pkt[i : i + 8]
        msg = can.Message(arbitration_id=CAN_ID_TX, data=list(chunk), is_extended_id=False)
        bus.send(msg)


def _receive_reply(
    bus,
    reassemble,
    timeout: float,
    expect_cmd_set: int | None = None,
    expect_cmd_id: int | None = None,
    debug: bool = False,
) -> bytes | None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.05)  # type: ignore[union-attr]
        if msg is None:
            continue
        if debug:
            payload = msg.data or []
            dlc = len(payload)
            hex_data = " ".join(f"{b:02X}" for b in payload[:dlc])
            print(f"  [debug] CAN id=0x{msg.arbitration_id:03X} len={dlc} data={hex_data}")
        if msg.arbitration_id != CAN_ID_RX:
            continue
        out = reassemble(bytes(msg.data or []))
        if out is None:
            continue
        r = validate_sdk_reply(out)
        if r is None:
            continue
        if try_handle_push(out):
            continue  # was a push; keep waiting for expected reply
        cs, cid, _, _ = r
        if expect_cmd_set is not None and cs != expect_cmd_set:
            continue
        if expect_cmd_id is not None and cid != expect_cmd_id:
            continue
        return out
    return None


def run(
    channel: str,
    interval: float = 0.25,
    debug: bool = False,
    command: str = "angle",
    extra: list[str] | None = None,
) -> None:
    import can

    bitrate = 1_000_000
    bus = can.Bus(interface="slcan", channel=channel, bitrate=bitrate)

    reassem = bytearray()
    pack_len = 0
    step = 0

    def reassemble(payload: bytes) -> bytes | None:
        nonlocal reassem, pack_len, step
        for b in payload:
            if step == 0:
                if b == SOF:
                    reassem = bytearray([b])
                    step = 1
                continue
            if step == 1:
                pack_len = b
                reassem.append(b)
                step = 2
                continue
            if step == 2:
                pack_len |= (b & 0x03) << 8
                reassem.append(b)
                step = 3
                continue
            if step == 3:
                reassem.append(b)
                if len(reassem) == 12:
                    if crc16(bytes(reassem[:10])) == (reassem[10] | (reassem[11] << 8)):
                        step = 4
                    else:
                        step = 0
                        reassem = bytearray()
                continue
            if step == 4:
                reassem.append(b)
                if len(reassem) == pack_len:
                    step = 0
                    out = bytes(reassem)
                    reassem = bytearray()
                    if crc32(out[:-4]) == struct.unpack_from("<I", out, len(out) - 4)[0]:
                        return out
                    return None
        return None

    def request_reply(pkt: bytes, expect_set: int, expect_id: int, timeout: float = 0.5) -> bytes | None:
        _send_packet(bus, pkt)
        return _receive_reply(bus, reassemble, timeout, expect_cmd_set=expect_set, expect_cmd_id=expect_id, debug=debug)

    extra = extra or []

    try:
        if command == "angle":
            print("Gimbal state (yaw, roll, pitch in °) every {:.2f} s. Ctrl+C to stop.".format(interval))
            print("-" * 50)
            while True:
                pkt = build_obtain_gimbal_angle(0x01)
                _send_packet(bus, pkt)
                reply = _receive_reply(bus, reassemble, 0.25, expect_cmd_set=0x0E, expect_cmd_id=0x02, debug=debug)
                if reply is not None:
                    gimbal_result = parse_gimbal_reply(reply)
                    if gimbal_result is not None:
                        yaw_deg, roll_deg, pitch_deg = gimbal_result
                        print("yaw = {:7.1f}°  roll = {:7.1f}°  pitch = {:7.1f}°".format(yaw_deg, roll_deg, pitch_deg))
                        time.sleep(max(0.0, interval - 0.25))
                        continue
                print("(no valid gimbal reply)")
                time.sleep(max(0.0, interval - 0.25))

        elif command == "joint":
            print("Gimbal joint angles (yaw, roll, pitch in °) every {:.2f} s. Ctrl+C to stop.".format(interval))
            print("-" * 50)
            while True:
                pkt = build_obtain_gimbal_angle(0x02)
                _send_packet(bus, pkt)
                reply = _receive_reply(bus, reassemble, 0.25, expect_cmd_set=0x0E, expect_cmd_id=0x02, debug=debug)
                if reply is not None:
                    gimbal_result = parse_gimbal_reply(reply)
                    if gimbal_result is not None:
                        yaw_deg, roll_deg, pitch_deg = gimbal_result
                        print("yaw = {:7.1f}°  roll = {:7.1f}°  pitch = {:7.1f}° (joint)".format(yaw_deg, roll_deg, pitch_deg))
                        time.sleep(max(0.0, interval - 0.25))
                        continue
                print("(no valid gimbal reply)")
                time.sleep(max(0.0, interval - 0.25))

        elif command == "version":
            print("Obtaining module version...")
            reply = request_reply(build_obtain_module_version(), 0x0E, 0x09)
            if reply is not None:
                res = parse_module_version_reply(reply)
                if res is not None:
                    _dev_id, _ver, (a, b, c, d) = res
                    print("device_id = 0x{:08X}  version = {}.{}.{}.{}".format(_dev_id, a, b, c, d))
                else:
                    print("(parse failed)")
            else:
                print("(no reply)")

        elif command == "limit":
            print("Obtaining gimbal limit angles...")
            reply = request_reply(build_obtain_gimbal_limit_angle(), 0x0E, 0x04)
            if reply is not None:
                res = parse_limit_angle_reply(reply)
                if res is not None:
                    print("yaw   [{:7.1f}°, {:7.1f}°]  roll [{:7.1f}°, {:7.1f}°]  pitch [{:7.1f}°, {:7.1f}°]".format(
                        res["yaw_min"], res["yaw_max"], res["roll_min"], res["roll_max"],
                        res["pitch_min"], res["pitch_max"]))
                else:
                    raw_info = limit_angle_reply_raw(reply)
                    if raw_info is not None:
                        rc, payload = raw_info
                        if rc != RET_SUCCESS:
                            print("Device returned 0x{:02X} ({}).".format(rc, return_code_str(rc)))
                        else:
                            print("(unexpected payload) len={} hex={}".format(len(payload), payload.hex()))
                    else:
                        print("(unexpected reply)")
            else:
                print("(no reply)")

        elif command == "stiffness":
            print("Obtaining motor stiffness...")
            reply = request_reply(build_obtain_motor_stiffness(), 0x0E, 0x06)
            if reply is not None:
                res = parse_motor_stiffness_reply(reply)
                if res is not None:
                    rc = res.get("ret_code", 0)
                    length = res["raw_len"]
                    hex_str = res["hex"]
                    if rc == RET_SUCCESS:
                        if length == 0:
                            print("return_code=0x00  len=0  (empty – device may not report stiffness)")
                        else:
                            print("return_code=0x00  len={}  hex={}".format(length, hex_str))
                    else:
                        print("Device returned 0x{:02X} ({}).".format(rc, return_code_str(int(rc))))
                else:
                    print("(parse failed)")
            else:
                print("(no reply)")

        elif command == "user-params":
            print("Obtaining gimbal user parameters...")
            reply = request_reply(build_obtain_gimbal_user_params(), 0x0E, 0x0B)
            if reply is not None:
                res = parse_user_params_reply(reply)
                if res is not None:
                    rc = res.get("ret_code", 0)
                    length = res["raw_len"]
                    hex_str = res["hex"]
                    if rc == RET_SUCCESS:
                        if length == 0:
                            print("return_code=0x00  len=0  (empty – device may not report user params)")
                        else:
                            print("return_code=0x00  len={}  hex={}".format(length, hex_str))
                    else:
                        print("Device returned 0x{:02X} ({}).".format(rc, return_code_str(int(rc))))
                else:
                    print("(parse failed)")
            else:
                print("(no reply)")

        elif command == "recenter":
            print("Sending Recenter...")
            reply = request_reply(build_recenter_selfie(0x01), 0x0E, 0x0E, timeout=1.0)
            if reply is not None:
                r = validate_sdk_reply(reply)
                print("OK" if (r and r[2] == RET_SUCCESS) else "Failed (return code 0x{:02X})".format(r[2] if r else 0))
            else:
                print("Command sent (no reply; gimbal may have executed).")

        elif command == "selfie":
            print("Sending Selfie...")
            reply = request_reply(build_recenter_selfie(0x02), 0x0E, 0x0E, timeout=1.0)
            if reply is not None:
                r = validate_sdk_reply(reply)
                print("OK" if (r and r[2] == RET_SUCCESS) else "Failed (return code 0x{:02X})".format(r[2] if r else 0))
            else:
                print("Command sent (no reply; gimbal may have executed).")

        elif command == "activetrack":
            print("Toggling ActiveTrack...")
            reply = request_reply(build_activetrack_toggle(), 0x0E, 0x11, timeout=1.0)
            if reply is not None:
                r = validate_sdk_reply(reply)
                print("OK" if (r and r[2] == RET_SUCCESS) else "Failed (return code 0x{:02X})".format(r[2] if r else 0))
            else:
                print("Command sent (no reply; gimbal may have executed).")

        elif command == "push-on":
            print("Enabling parameter push...")
            reply = request_reply(build_set_parameter_push(True), 0x0E, 0x07)
            if reply is not None:
                r = validate_sdk_reply(reply)
                print("OK" if (r and r[2] == RET_SUCCESS) else "Failed (return code {})".format(r[2] if r else "?"))
            else:
                print("(no reply)")

        elif command == "push-off":
            print("Disabling parameter push...")
            reply = request_reply(build_set_parameter_push(False), 0x0E, 0x07)
            if reply is not None:
                r = validate_sdk_reply(reply)
                print("OK" if (r and r[2] == RET_SUCCESS) else "Failed (return code {})".format(r[2] if r else "?"))
            else:
                print("(no reply)")

        elif command == "position":
            yaw = float(extra[0]) if len(extra) > 0 else 0.0
            roll = float(extra[1]) if len(extra) > 1 else 0.0
            pitch = float(extra[2]) if len(extra) > 2 else 0.0
            print("Sending position: yaw={:.1f}° roll={:.1f}° pitch={:.1f}° (absolute).".format(yaw, roll, pitch))
            pkt = build_control_position(yaw, roll, pitch, absolute=True)
            reply = request_reply(pkt, 0x0E, 0x00, timeout=1.0)
            if reply is not None:
                r = validate_sdk_reply(reply)
                print("OK" if (r and r[2] == RET_SUCCESS) else "Failed (return code 0x{:02X})".format(r[2] if r else 0))
            else:
                print("(no reply)")

        elif command == "speed":
            yaw_degs = float(extra[0]) if len(extra) > 0 else 0.0
            roll_degs = float(extra[1]) if len(extra) > 1 else 0.0
            pitch_degs = float(extra[2]) if len(extra) > 2 else 0.0
            print("Sending speed: yaw={:.1f}°/s roll={:.1f}°/s pitch={:.1f}°/s".format(yaw_degs, roll_degs, pitch_degs))
            pkt = build_control_speed(yaw_degs, roll_degs, pitch_degs)
            reply = request_reply(pkt, 0x0E, 0x01, timeout=0.5)
            if reply is not None:
                r = validate_sdk_reply(reply)
                print("OK" if (r and r[2] == RET_SUCCESS) else "Failed (return code 0x{:02X})".format(r[2] if r else 0))
            else:
                print("(no reply)")

        elif command == "listen":
            print("Parameter push enabled; listening for pushes (0x08, 0x10). Ctrl+C to stop.")
            _send_packet(bus, build_set_parameter_push(True))
            time.sleep(0.1)
            while True:
                msg = bus.recv(timeout=0.2)  # type: ignore[union-attr]
                if msg is None:
                    continue
                if msg.arbitration_id != CAN_ID_RX:
                    continue
                out = reassemble(bytes(msg.data or []))
                if out is not None and validate_sdk_reply(out) is not None:
                    _ = try_handle_push(out)

        elif command == "info":
            print("Gimbal info (angle, version, limit, stiffness). One-shot.")
            pkt = build_obtain_gimbal_angle(0x01)
            _send_packet(bus, pkt)
            reply = _receive_reply(bus, reassemble, 0.4, expect_cmd_set=0x0E, expect_cmd_id=0x02, debug=debug)
            if reply is not None:
                g = parse_gimbal_reply(reply)
                if g is not None:
                    print("Angle (attitude): yaw = {:.1f}°  roll = {:.1f}°  pitch = {:.1f}°".format(*g))
                else:
                    print("Angle: (parse failed)")
            else:
                print("Angle: (no reply)")
            reply = request_reply(build_obtain_module_version(), 0x0E, 0x09)
            if reply is not None:
                res = parse_module_version_reply(reply)
                if res is not None:
                    _, _, (a, b, c, d) = res
                    print("Version: {}.{}.{}.{}".format(a, b, c, d))
                else:
                    print("Version: (parse failed)")
            else:
                print("Version: (no reply)")
            reply = request_reply(build_obtain_gimbal_limit_angle(), 0x0E, 0x04)
            if reply is not None:
                res = parse_limit_angle_reply(reply)
                if res is not None:
                    print("Limit: yaw [{:.1f},{:.1f}°] roll [{:.1f},{:.1f}°] pitch [{:.1f},{:.1f}°]".format(
                        res["yaw_min"], res["yaw_max"], res["roll_min"], res["roll_max"],
                        res["pitch_min"], res["pitch_max"]))
                else:
                    raw_info = limit_angle_reply_raw(reply)
                    if raw_info is not None:
                        rc, _ = raw_info
                        if rc != RET_SUCCESS:
                            print("Limit: Device returned 0x{:02X} ({}).".format(rc, return_code_str(rc)))
                        else:
                            print("Limit: (unexpected payload)")
                    else:
                        print("Limit: (unexpected reply)")
            else:
                print("Limit: (no reply)")
            reply = request_reply(build_obtain_motor_stiffness(), 0x0E, 0x06)
            if reply is not None:
                res = parse_motor_stiffness_reply(reply)
                if res is not None:
                    rc = res.get("ret_code", 0)
                    if rc == RET_SUCCESS:
                        print("Stiffness: len={} hex={}".format(res["raw_len"], res["hex"]))
                    else:
                        print("Stiffness: Device returned 0x{:02X} ({}).".format(rc, return_code_str(int(rc))))
                else:
                    print("Stiffness: (parse failed)")
            else:
                print("Stiffness: (no reply)")

        elif command == "focus-set":
            if not extra:
                print("focus-set requires one argument: position (0–4096).")
                return
            try:
                pos = int(extra[0])
            except ValueError:
                print("focus-set position must be an integer (0–4096).")
                return
            if pos < 0 or pos > 4096:
                print("focus-set position out of range (0–4096).")
                return
            print(f"Sending focus position: {pos} (0–4096).")
            pkt = build_focus_set(pos)
            reply = request_reply(pkt, 0x0E, 0x12, timeout=0.8)
            if reply is not None:
                val = parse_focus_reply(reply)
                if val is not None:
                    print(f"Focus position reported by device: {val}")
                else:
                    r = validate_sdk_reply(reply)
                    if r is not None:
                        _, _, rc, data = r
                        print(
                            "Focus reply: ret=0x{:02X} len={} hex={}".format(
                                rc, len(data), data.hex()
                            )
                        )
                    else:
                        print("Focus reply received but failed validation.")
            else:
                print("No focus reply (command may still have been executed).")

        elif command == "focus-get":
            print("Requesting focus motor position...")
            pkt = build_focus_get()
            reply = request_reply(pkt, 0x0E, 0x12, timeout=0.8)
            if reply is not None:
                val = parse_focus_reply(reply)
                if val is not None:
                    print(f"Focus position: {val}")
                else:
                    r = validate_sdk_reply(reply)
                    if r is not None:
                        _, _, rc, data = r
                        print(
                            "Focus reply: ret=0x{:02X} len={} hex={}".format(
                                rc, len(data), data.hex()
                            )
                        )
                    else:
                        print("Focus reply received but failed validation.")
            else:
                print("No focus reply.")

        elif command in PROBE_COMMANDS:
            label, builder = PROBE_COMMANDS[command]
            pkt = builder()
            cmd_set, cmd_id = pkt[12], pkt[13]
            payload = pkt[14:-4]
            print(
                "{}  cmd_set=0x{:02X}  cmd_id=0x{:02X}  payload={}".format(
                    label, cmd_set, cmd_id, payload.hex()
                )
            )
            print("This command executes if the gimbal accepts it.")
            reply = request_reply(pkt, cmd_set, cmd_id, timeout=1.0)
            print_sdk_reply(reply)

        else:
            print("Unknown command: {}".format(command))
            print("Use: angle, joint, version, limit, stiffness, user-params, recenter, selfie, activetrack,")
            print("     push-on, push-off, position, speed, listen, info, focus-set, focus-get,")
            print("     sleep, wake, rec-start, rec-stop, focus-center-start, focus-center-stop")
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="DJI gimbal/CAN console via SH-C31G (Canable 2.0). Commands: angle, position, speed, limit, stiffness, user-params, recenter, selfie, activetrack, sleep, wake, rec-start/stop, focus-center-start/stop, push-on/off, listen, info.",
    )
    parser.add_argument(
        "channel",
        nargs="?",
        default="COM6",
        help="CAN interface channel (e.g. COM4 on Windows, /dev/ttyACM0 on Linux). Default: COM6",
    )
    parser.add_argument(
        "-c", "--command",
        default="angle",
        choices=[
            "angle", "joint", "version", "limit", "stiffness", "user-params",
            "recenter", "selfie", "activetrack", "push-on", "push-off",
            "position", "speed", "listen", "info", "focus-set", "focus-get",
            *PROBE_COMMANDS,
        ],
        help=(
            "Command. For position pass extra args (e.g. -c position 0 0 -90). "
            "For speed pass yaw roll pitch deg/s (e.g. -c speed 10 0 -5). "
            "For focus-set pass focus position 0–4096 (e.g. -c focus-set 2048). "
            "sleep/wake/rec-*/focus-center-* execute if accepted; replies print cmd_set/cmd_id/ret/hex. "
            "Default: angle"
        ),
    )
    parser.add_argument(
        "extra",
        nargs="*",
        help="Extra args for position (yaw roll pitch °) or speed (yaw roll pitch °/s).",
    )
    parser.add_argument(
        "-i", "--interval",
        type=float,
        default=0.25,
        help="Print interval in seconds for angle/joint streams (default: 0.25)",
    )
    parser.add_argument(
        "-d", "--debug",
        action="store_true",
        help="Print every received CAN frame (id and data) for troubleshooting",
    )
    args = parser.parse_args()
    run(args.channel, args.interval, debug=args.debug, command=args.command, extra=args.extra)
    return 0


if __name__ == "__main__":
    sys.exit(main())

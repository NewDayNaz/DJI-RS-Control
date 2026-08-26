# DJI R SDK Gimbal Protocol (CAN Transport)

This document describes the DJI R SDK external protocol as used by `dji_gimbal_cli.py` to control a DJI RS gimbal over CAN.

- **Physical transport**: CAN bus via SLCAN adapter (e.g. SH-C31G / Canable 2.0)
- **Bitrate**: `1_000_000` bps (1 Mbps)
- **SOF (start-of-frame)**: `0xAA`
- **Command set (gimbal)**: `0x0E`
- **Command set (camera record / focus-center)**: `0x0D`
- **CAN IDs**:
  - Host → gimbal: `0x223`
  - Gimbal → host: `0x222`

The CLI splits logical SDK packets into CAN frames with up to 8 bytes of data each.

---

## 1. High-Level Overview

The DJI R SDK protocol encapsulates gimbal commands and telemetry in a custom packet format that is carried over CAN.

At a high level:

1. The host builds an SDK packet (SOF, header, command set/ID, payload, CRCs).
2. The packet is fragmented into 8-byte CAN frames with arbitration ID `0x223`.
3. The gimbal reassembles the SDK packet, executes the command, and replies.
4. Replies are fragmented into 8-byte CAN frames with arbitration ID `0x222`.
5. The host reassembles packets, validates CRC-16 and CRC-32, and parses the reply.

The same packet format is also used for unsolicited "push" messages where the gimbal sends state updates without an explicit request.

---

## 2. SDK Packet Format

All commands share a common packet structure.

```text
Byte 0      : SOF              (0xAA)
Byte 1      : LEN_L            (low 8 bits of total packet length)
Byte 2      : LEN_H_FLAGS      (low 2 bits = high length bits; remaining bits = flags)
Byte 3      : CMD_TYPE_FLAGS   (command type + flags)
Byte 4      : ENC              (encryption flag; 0x00 = none)
Bytes 5–7   : RES              (3 bytes reserved; all 0x00)
Bytes 8–9   : SEQ              (sequence number, little-endian)
Bytes 10–11 : CRC16            (CRC-16 over bytes 0–9, little-endian)
Byte 12     : CMD_SET          (command set; 0x0E gimbal, 0x0D camera)
Byte 13     : CMD_ID           (command ID within set)
Bytes 14..N-5 : DATA           (command-specific payload)
Bytes N-4..N-1 : CRC32         (CRC-32 over bytes 0..N-5, little-endian)
```

### 2.1 Length

The "command length" (`cmd_length`) is the total length of the SDK packet in bytes, including:

- header (bytes 0–9),
- CRC-16 (bytes 10–11),
- data segment (`CMD_SET`, `CMD_ID`, `DATA`),
- CRC-32 (last 4 bytes).

In the implementation:

- When building a packet:

  ```text
  prefix_len = 10      # bytes 0–9
  crc16_len  = 2       # bytes 10–11
  data_len   = len([CMD_SET, CMD_ID] + DATA)
  crc32_len  = 4       # last 4 bytes
  cmd_length = prefix_len + crc16_len + data_len + crc32_len
  ```

- When parsing a reply:

  ```text
  pack_len = packet[1] | ((packet[2] & 0x03) << 8)
  ```

The actual packet length must match `pack_len`, otherwise the packet is rejected.

### 2.2 CMD_TYPE and Reply Bit

Byte 3 carries the command type and some flags:

- For host-originated commands, the CLI uses:

  ```text
  CMD_TYPE_REPLY_REQUIRED = 0x03
  ```

- For replies from the gimbal, bit `0x20` must be set; otherwise the packet is not treated as a reply:

  ```text
  is_reply = (packet[3] & 0x20) != 0
  ```

Unsolicited "push" packets also have this bit set; they are distinguished by their command IDs (e.g. `0x08`, `0x10`).

### 2.3 Sequence Number

Bytes 8–9 hold the sequence number in little-endian. The CLI uses a global sequence `_seq`:

- Initialized around `0x2210`.
- Incremented per packet.
- When `_seq >= 0xFFFD`, it wraps back to `0x0002`.

You may treat the sequence as an opaque counter; the gimbal does not appear to enforce strict matching for basic operations.

---

## 3. CRC Algorithms

Two CRCs guard each packet:

### 3.1 CRC-16 (Header)

- **Polynomial**: `0x8005`
- **Initial value**: `0x3AA3`
- **Reflected**: Yes (input and output)
- **Range**: 16-bit

Usage:

- Computed over bytes `0..9` (SOF through SEQ).
- Stored as little-endian at bytes 10–11.

### 3.2 CRC-32 (Full Packet)

- **Polynomial**: `0x04C11DB7`
- **Initial value**: `0x00003AA3`
- **Reflected**: Yes (input and output)
- **Range**: 32-bit

Usage:

- Computed over the entire packet except the last 4 bytes.
- Stored as 4-byte little-endian at the end of the packet.

Validation rules:

1. Check SOF (`0xAA`).
2. Compute and verify length.
3. Verify CRC-16 over header.
4. Verify CRC-32 over body.

Only packets that pass all steps are considered valid.

---

## 4. Transport over CAN

### 4.1 Physical Connector

The gimbal/accessory's CAN port (same 4-pin connector family used on DJI Focus motor units
and other CAN-based accessories) exposes:

| Pin | Signal   | Notes                                                                 |
|-----|----------|------------------------------------------------------------------------|
| 1   | `VCC_5V` | 5V supplied **by** the gimbal/device. Can potentially power an external adapter/MCU, but the port's current budget is undocumented — don't assume it can carry a WiFi-radio-class load (see `firmware/esp32-gimbal-bridge/README.md`). Never drive a different voltage into this pin. |
| 2   | `GND`    | Common ground — always connect this regardless of how you power your adapter. |
| 3   | `CANH`   | CAN bus high                                                            |
| 4   | `CANL`   | CAN bus low                                                             |

Next to the port is a slide switch labeled **`S-BUS` / `CAN`**. It must be set to **`CAN`**
— in the `S-BUS` position the port instead speaks the single-wire S-BUS/PWM protocol used by
some third-party focus controllers, not the DJI R SDK packet format documented in this file.
If you wire up an adapter and get nothing but silence on `0x222`, check this switch first.

This is the same pinout the SH-C31G/Canable adapter's flying leads were wired to: `CANH`/
`CANL` to the transceiver's bus pins, `GND` common, `VCC_5V` left unconnected (adapter
powered separately over USB).

### 4.2 CAN IDs and Framing

The SDK packet is transported over CAN as follows:

- **Host → gimbal**:
  - Standard 11-bit CAN ID: `0x223`
  - Data: 0–8 bytes per frame
- **Gimbal → host**:
  - Standard 11-bit CAN ID: `0x222`
  - Data: 0–8 bytes per frame

The host sends each SDK packet in 8-byte chunks:

```text
for i in range(0, len(pkt), 8):
    send CAN frame with arbitration_id=0x223 and data=pkt[i : i+8]
```

On reception, the CLI:

1. Filters by arbitration ID `0x222`.
2. Feeds each frame's data to a state-machine reassembler.
3. Detects `SOF` and length.
4. Validates CRC-16 and CRC-32.
5. Emits a complete SDK packet when fully reassembled and valid.

---

## 5. Reply Packets and Return Codes

Replies from the gimbal share the same packet structure, but:

- Byte 3 must have the reply bit set (`0x20`).
- Byte 12 is the command set.
- Byte 13 is the command ID.
- Byte 14 is the **return code**.
- Bytes 15..N-5 (if any) are the **reply payload**.

### 5.1 Reply Parsing

The CLI uses a helper (`validate_sdk_reply`) which:

1. Checks total length.
2. Confirms SOF and reply bit.
3. Validates CRC-16 and CRC-32.
4. Returns a tuple:

```text
(cmd_set, cmd_id, ret_code, data)
```

where `data` is the reply payload from byte 15 up to (but not including) the CRC-32.

### 5.2 Return Codes

Return codes (from the DJI R SDK documentation, §2.3.2):

- `0x00` → **success**
- `0x01` → **command parse error**
- `0x02` → **command execution failed**
- `0xFF` → **undefined error**
- Others → reported as raw hex (e.g. `0xAB`).

Most of the CLI commands treat `ret_code == 0x00` as success and display additional information only when parsing the payload.

---

## 6. Gimbal Command Set 0x0E

Gimbal motion, telemetry, and sleep/wake in the CLI use:

- `CMD_SET = 0x0E`

Camera record and Ronin focus-center use `CMD_SET = 0x0D` (section 21). Those are a different command set, not extra `0x0E` cmd_ids.

The sections below document each observed command (CMD_ID) and payload.

### 6.1 Summary of Command IDs

| CMD_SET | CMD_ID | Direction       | Purpose                             |
|--------:|-------:|-----------------|-------------------------------------|
|  0x0E   | 0x00   | host → gimbal   | Control gimbal position             |
|  0x0E   | 0x01   | host → gimbal   | Control gimbal speed (handheld)     |
|  0x0E   | 0x02   | host ↔ gimbal   | Obtain gimbal/joint angles          |
|  0x0E   | 0x04   | host ↔ gimbal   | Obtain gimbal limit angles          |
|  0x0E   | 0x06   | host ↔ gimbal   | Obtain motor stiffness              |
|  0x0E   | 0x07   | host ↔ gimbal   | Enable/disable parameter push       |
|  0x0E   | 0x08   | gimbal → host   | Gimbal parameter push (angles)      |
|  0x0E   | 0x09   | host ↔ gimbal   | Obtain module version               |
|  0x0E   | 0x0B   | host ↔ gimbal   | Obtain gimbal user parameters       |
|  0x0E   | 0x0C   | host ↔ gimbal   | Sleep / wake                        |
|  0x0E   | 0x0E   | host ↔ gimbal   | Recenter / Selfie                   |
|  0x0E   | 0x10   | gimbal → host   | Auto-calibration status push        |
|  0x0E   | 0x11   | host ↔ gimbal   | ActiveTrack toggle                  |
|  0x0E   | 0x12   | host ↔ gimbal   | Focus motor control / query         |
|  0x0D   | 0x00   | host → camera   | Record / focus-center (section 21)  |

The following subsections describe the payloads and expected replies.

---

## 7. Position Control – CMD_ID 0x00

**Purpose**: Move the gimbal to the specified yaw/roll/pitch, either absolute or incremental.

- **Command set**: `0x0E`
- **Command ID**: `0x00`

### 7.1 Request Payload

Layout (little-endian):

```text
struct "<3hBB":
    yaw   int16  (0.1° units)
    roll  int16  (0.1° units)
    pitch int16  (0.1° units)
    ctrl  uint8
    time  uint8  (0.1 s units)
```

Units and behavior:

- `yaw`, `roll`, `pitch`:
  - Given in degrees in the CLI.
  - Encoded as `round(deg * 10)` (int16), range ≈ ±3276.8°.
- `time`:
  - Time to reach the target position, in seconds.
  - Encoded as `round(time_s * 10)`, clamped to `[0, 255]`.
  - Range: 0.0–25.5 s.

`ctrl` byte bitfields:

- Bit 0:
  - `1` → absolute control (angles are absolute).
  - `0` → incremental control (angles are deltas).
- Bit 1:
  - `1` → yaw is invalid (ignore yaw).
  - `0` → yaw is valid.
- Bit 2:
  - `1` → roll is invalid.
  - `0` → roll is valid.
- Bit 3:
  - `1` → pitch is invalid.
  - `0` → pitch is valid.

The CLI currently uses:

- Absolute mode (`ctrl` bit 0 set).
- All axes valid (bits 1–3 cleared).

### 7.2 Reply

- Reply command: `CMD_SET = 0x0E`, `CMD_ID = 0x00`.
- Return code semantics as usual:
  - `0x00` → success.
  - Others → error; payload is typically empty.

---

## 8. Speed Control – CMD_ID 0x01

**Purpose**: Control angular rates of the gimbal (handheld speed control).

- **Command set**: `0x0E`
- **Command ID**: `0x01`

### 8.1 Request Payload

Layout:

```text
struct "<3hB":
    yaw_rate   int16 (0.1°/s)
    roll_rate  int16 (0.1°/s)
    pitch_rate int16 (0.1°/s)
    ctrl       uint8
```

Units and clamping:

- Input: `deg_per_second` per axis.
- Encoding:
  - `value_x10 = round(deg_per_second * 10)`
  - Clamped to `[-3600, +3600]` = ±360.0°/s.

Control byte:

- The CLI always sends `0x88`, described as "take over speed control".

### 8.2 Reply

- Reply command: `CMD_SET = 0x0E`, `CMD_ID = 0x01`.
- Command is considered successful if `ret_code == 0x00`.

---

## 9. Obtain Gimbal / Joint Angles – CMD_ID 0x02

**Purpose**: Read current gimbal "attitude" or "joint" angles.

- **Command set**: `0x0E`
- **Command ID**: `0x02`

### 9.1 Request Payload

Single byte:

```text
data_type uint8
    0x01 → attitude angles
    0x02 → joint angles
```

### 9.2 Reply Payload

Layout:

```text
data_type uint8  (echo)
yaw       int16  (0.1°)
roll      int16  (0.1°)
pitch     int16  (0.1°)
```

Conversions:

- `yaw_deg   = yaw   * 0.1`
- `roll_deg  = roll  * 0.1`
- `pitch_deg = pitch * 0.1`

The CLI uses this for:

- Continuous streams (`angle`, `joint` commands).
- One-shot "info" requests.

---

## 10. Obtain Gimbal Limit Angles – CMD_ID 0x04

**Purpose**: Obtain configured gimbal axis limits (min/max per axis).

- **Command set**: `0x0E`
- **Command ID**: `0x04`

### 10.1 Request Payload

No payload (`DATA` is empty).

### 10.2 Reply Payload

Two observed formats (all int16 in 0.1° units):

1. **6 x int16** (12 bytes total):

   ```text
   yaw_min   int16
   yaw_max   int16
   roll_min  int16
   roll_max  int16
   pitch_min int16
   pitch_max int16
   ```

2. **1 x uint8 prefix + 6 x int16** (13 bytes total):

   ```text
   prefix    uint8 (meaning not yet decoded)
   yaw_min   int16
   yaw_max   int16
   roll_min  int16
   roll_max  int16
   pitch_min int16
   pitch_max int16
   ```

All values are scaled by 0.1 when presented in degrees.

If decoding fails but `ret_code == 0x00`, the CLI prints the raw payload length and hex representation for further analysis.

---

## 11. Obtain Motor Stiffness – CMD_ID 0x06

**Purpose**: Query motor stiffness parameters (if exposed by the device).

- **Command set**: `0x0E`
- **Command ID**: `0x06`

### 11.1 Request Payload

No payload (`DATA` is empty).

### 11.2 Reply Payload

The CLI currently does not decode the structure of the payload; instead, it reports:

- `ret_code` (success or error).
- `raw_len` (payload length).
- `hex` (hexadecimal payload representation).

Devices which do not report stiffness may reply with:

- `ret_code == 0x00`, `raw_len == 0`.

---

## 12. Set Parameter Push – CMD_ID 0x07

**Purpose**: Enable or disable periodic push messages from the gimbal.

- **Command set**: `0x0E`
- **Command ID**: `0x07`

### 12.1 Request Payload

Single byte:

```text
enable uint8
    0x01 → enable parameter push
    0x00 → disable parameter push
```

### 12.2 Reply

- Reply command: `CMD_SET = 0x0E`, `CMD_ID = 0x07`.
- `ret_code == 0x00` indicates success.

After enabling push, the gimbal will start sending unsolicited packets with:

- `CMD_ID = 0x08` (gimbal parameter push).
- `CMD_ID = 0x10` (auto-calibration status).

---

## 13. Gimbal Parameter Push – CMD_ID 0x08 (Unsolicited)

**Purpose**: Continuous push of gimbal parameters, including current angles.

- **Command set**: `0x0E`
- **Command ID**: `0x08`
- **Direction**: gimbal → host (unsolicited)

### 13.1 Payload Layout

Same as angle reply:

```text
data_type uint8  (mode / angle type)
yaw       int16  (0.1°)
roll      int16  (0.1°)
pitch     int16  (0.1°)
```

If payload length is at least 7 bytes, the CLI decodes and prints:

```text
[push] gimbal params: yaw=X.X° roll=Y.Y° pitch=Z.Z°
```

Otherwise, it prints raw hex data for investigation.

---

## 14. Auto-Calibration Status Push – CMD_ID 0x10 (Unsolicited)

**Purpose**: Status updates for automatic gimbal calibration.

- **Command set**: `0x0E`
- **Command ID**: `0x10`
- **Direction**: gimbal → host (unsolicited)

Payload structure is currently treated as opaque; the CLI just prints:

```text
[push] auto calibration status (0x10): ret=0xRR len=NN hex=...
```

where:

- `RR` is `ret_code`.
- `NN` is payload length.

---

## 15. Obtain Module Version – CMD_ID 0x09

**Purpose**: Read firmware version of a specific module (e.g. DJI R SDK).

- **Command set**: `0x0E`
- **Command ID**: `0x09`

### 15.1 Request Payload

Layout:

```text
device_id uint32 (little-endian)
```

The CLI uses:

- `device_id = 0x00000001` for the DJI R SDK module.

### 15.2 Reply Payload

Layout:

```text
device_id uint32
ver       uint32  (0xAABBCCDD)
```

The version is decoded as:

```text
A = ver >> 24
B = (ver >> 16) & 0xFF
C = (ver >> 8)  & 0xFF
D = ver & 0xFF
version string = "A.B.C.D"
```

---

## 16. Obtain Gimbal User Parameters – CMD_ID 0x0B

**Purpose**: Retrieve user-configurable gimbal parameters.

- **Command set**: `0x0E`
- **Command ID**: `0x0B`

### 16.1 Request Payload

No payload (`DATA` is empty).

### 16.2 Reply Payload

The CLI currently does not interpret the structure of the user parameters; instead, it reports:

- `ret_code` (success or error).
- `raw_len` (payload length).
- `hex` (hex representation of the payload).

Future work could decode individual user parameters based on further firmware analysis.

---

## 17. Sleep / Wake – CMD_ID 0x0C

**Purpose**: Put the gimbal to sleep or wake it.

- **Command set**: `0x0E`
- **Command ID**: `0x0C`

This is not recenter. Recenter is `CMD_SET = 0x0E`, `CMD_ID = 0x0E`, payload `FE 01` (section 18). Sleep and wake share `CMD_ID = 0x0C`. The last payload byte is the switch.

### 17.1 Request Payload

Three bytes. Total SDK packet length is 21.

```text
0x23  uint8  # constant
0x01  uint8  # constant
mode  uint8
    0x01 → sleep  (CLI: sleep)
    0x00 → wake   (CLI: wake)
```

CLI builders:

- `build_sleep()` → `0x0E / 0x0C / 23 01 01`
- `build_wake()`  → `0x0E / 0x0C / 23 01 00`

### 17.2 Reply

- Reply command: `CMD_SET = 0x0E`, `CMD_ID = 0x0C`.
- The CLI prints `cmd_set`, `cmd_id`, `ret_code`, and payload hex from the `0x222` reply.
- These commands execute if the gimbal accepts them. Use `sleep` only when you mean to sleep the unit.

---

## 18. Recenter / Selfie – CMD_ID 0x0E

**Purpose**: Trigger a recenter or selfie operation.

- **Command set**: `0x0E`
- **Command ID**: `0x0E`

### 18.1 Request Payload

Two bytes:

```text
0xFE  uint8  # constant prefix
mode uint8
    0x01 → Recenter once
    0x02 → Selfie once
```

### 18.2 Reply

- Reply command: `CMD_SET = 0x0E`, `CMD_ID = 0x0E`.
- `ret_code == 0x00` → success.
- The CLI also treats a missing reply as "command may still have executed" based on empirical behavior.

Do not confuse this with sleep/wake (`CMD_ID = 0x0C`, payload `23 01 xx`).

---

## 19. ActiveTrack Toggle – CMD_ID 0x11

**Purpose**: Toggle ActiveTrack on/off.

- **Command set**: `0x0E`
- **Command ID**: `0x11`

### 19.1 Request Payload

Single byte:

```text
0x03  uint8  # toggle ActiveTrack start/stop
```

### 19.2 Reply

- Reply command: `CMD_SET = 0x0E`, `CMD_ID = 0x11`.
- `ret_code == 0x00` → ActiveTrack toggled successfully.

---

## 20. Focus Motor Control and Query – CMD_ID 0x12

**Purpose**: Control and query the external focus motor.

This is the follow-focus motor (`CMD_ID = 0x12`). Ronin camera focus-center is `CMD_SET = 0x0D` (section 21).

- **Command set**: `0x0E`
- **Command ID**: `0x12`

There are at least two sub-operations:

1. Set focus position (`focus-set`).
2. Get focus position (`focus-get`).

### 20.1 Set Focus Position (`focus-set`)

**Request payload**:

```text
struct "<3BH":
    cmd_sub_id uint8 (0x01)
    ctl_type   uint8 (0x00)
    data_len   uint8 (0x02)
    position   uint16 (0–4096)
```

Fields:

- `cmd_sub_id = 0x01`
- `ctl_type   = 0x00`
- `data_len   = 0x02` (two-byte position field)
- `position`:
  - Range typically `0–4096`.
  - CLI clamps user input to `[0, 4096]`.

**Reply payload**:

Empirical behavior suggests that the focus position is encoded as a **little-endian uint32** in the last 4 bytes of the DATA segment:

```text
... <other bytes> ... focus_pos uint32
```

The CLI extracts:

```text
focus_pos = struct.unpack_from("<I", data, len(data) - 4)[0]
```

Values are typically in the range `0–4096`.

### 20.2 Get Focus Position (`focus-get`)

**Request payload**:

```text
0x15  uint8
0x00  uint8
```

These bytes match a reference implementation (`getFocPosData`) observed in the firmware.

**Reply payload**:

Uses the same parsing as `focus-set` replies:

- The last 4 bytes of `data` are interpreted as the current focus position, encoded as little-endian uint32.

If decoding fails, the CLI falls back to printing:

- `ret_code`
- `len(data)`
- `data` in hex.

---

## 21. Camera Command Set 0x0D

**Purpose**: Trigger camera record and Ronin focus-center. These are not gimbal `0x0E` commands.

- **Command set**: `0x0D`
- **Command ID**: `0x00` for the four CLI commands below

| CLI | DATA |
|-----|------|
| `rec-start` | `03 00` |
| `rec-stop` | `04 00` |
| `focus-center-start` | `05 00` |
| `focus-center-stop` | `0B 00` |

Total SDK packet length is 20 (2-byte payload).

### 21.1 Request Payload

```text
op    uint8
0x00  uint8  # constant
```

`op` values:

- `0x03` record start
- `0x04` record stop
- `0x05` focus-center start
- `0x0B` focus-center stop

CLI builders: `build_record_start()`, `build_record_stop()`, `build_focus_center_start()`, `build_focus_center_stop()`.

### 21.2 Reply

- Reply command: `CMD_SET = 0x0D`, `CMD_ID = 0x00`.
- The CLI prints `cmd_set`, `cmd_id`, `ret_code`, and payload hex from the `0x222` reply.
- These commands execute if accepted. `rec-start` starts recording on a connected camera.

---

## 22. High-Level Interaction Patterns

The CLI (`dji_gimbal_cli.py`) demonstrates several typical flows:

### 22.1 Continuous Angle Streaming

- `angle`:
  - Repeatedly sends `CMD_SET = 0x0E`, `CMD_ID = 0x02`, `DATA = [0x01]` (attitude angles).
  - Parses yaw/roll/pitch and prints them at a given interval.
- `joint`:
  - Same, but `DATA = [0x02]` (joint angles).

### 22.2 One-Shot Info Query

The `info` command performs a sequence:

1. Obtain attitude angles (`0x02`).
2. Obtain module version (`0x09`).
3. Obtain gimbal limit angles (`0x04`).
4. Obtain motor stiffness (`0x06`).

All results are printed in a human-readable format for quick diagnostics.

### 22.3 Push-Based Telemetry

- `push-on`:
  - Sends `CMD_SET = 0x0E`, `CMD_ID = 0x07`, `DATA = [0x01]`.
  - Enables push messages.
- `listen`:
  - Ensures push is enabled.
  - Continuously reads from CAN ID `0x222`.
  - Reassembles SDK packets.
  - For each valid packet in the gimbal command set (`CMD_SET = 0x0E`), tries to handle:
    - `CMD_ID = 0x08`: gimbal parameter push (angles).
    - `CMD_ID = 0x10`: auto-calibration status.

### 22.4 Sleep, wake, record, and focus-center

These one-shot commands print the TX encoding, then the `0x222` reply as `cmd_set`, `cmd_id`, `ret_code`, and payload hex. They execute if the gimbal accepts them.

```text
python dji_gimbal_cli.py COM6 -c sleep
python dji_gimbal_cli.py COM6 -c wake
python dji_gimbal_cli.py COM6 -c rec-start
python dji_gimbal_cli.py COM6 -c rec-stop
python dji_gimbal_cli.py COM6 -c focus-center-start
python dji_gimbal_cli.py COM6 -c focus-center-stop
```

| CLI | CMD_SET | CMD_ID | DATA |
|-----|---------|--------|------|
| `sleep` | `0x0E` | `0x0C` | `23 01 01` |
| `wake` | `0x0E` | `0x0C` | `23 01 00` |
| `rec-start` | `0x0D` | `0x00` | `03 00` |
| `rec-stop` | `0x0D` | `0x00` | `04 00` |
| `focus-center-start` | `0x0D` | `0x00` | `05 00` |
| `focus-center-stop` | `0x0D` | `0x00` | `0B 00` |

---

## 23. Implementing a Custom Client

To implement your own client in another language or environment:

1. **Open CAN at 1 Mbps**:
   - Send frames with ID `0x223`, receive frames with ID `0x222`.
2. **Build SDK packets**:
   - Start with `SOF = 0xAA`.
   - Set `LEN` to total packet length.
   - Set `CMD_TYPE` to `0x03` (reply required) for requests.
   - Set `ENC = 0x00`, `RES = 0x00 0x00 0x00`.
   - Fill `SEQ` with an incrementing little-endian counter.
   - Append CRC-16 over bytes 0–9.
   - Append `CMD_SET`, `CMD_ID`, and command-specific `DATA`.
   - Append CRC-32 over bytes 0..N-5.
3. **Fragment into CAN frames**:
   - Slice packet into 8-byte chunks and send sequentially on ID `0x223`.
4. **Receive and reassemble**:
   - On ID `0x222`, feed bytes into a state machine:
     - Wait for `SOF = 0xAA`.
     - Read length low/high, compute total length.
     - Validate CRC-16 and CRC-32.
   - Treat packets with `(byte3 & 0x20) != 0` as replies or pushes.
5. **Decode commands**:
   - Use `CMD_SET` and `CMD_ID` as documented above (`0x0E` gimbal, `0x0D` camera).
   - Parse payloads according to the sections in this document.
6. **Handle return codes**:
   - Use `ret_code` to classify success vs. parse/execute/undefined errors.

This document, together with the `dji_gimbal_cli.py` implementation, should provide a complete reference for controlling the DJI RS gimbal over CAN using the DJI R SDK packet format.


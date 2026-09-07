# DJI R SDK Gimbal Protocol (CAN Transport)

This document describes the DJI R SDK external protocol used to control a DJI RS gimbal over accessory CAN. It applies to RS 2 / RS 2 Pro / RS 3 Pro / RS 4 / RS 4 Pro / RS 5 (including the RSA port, which carries 5 V and CAN on the same connector).

Two hosts in this repo speak it:

| Host | Transport | Packet layer | Session |
|------|-----------|--------------|---------|
| `dji_gimbal_cli.py` / `dji_can_session.py` | USB CAN adapter (SLCAN / gs_usb, e.g. SH-C31G / Canable 2.0) | `dji_gimbal_cli.py` | CLI waits for matching replies; the session fire-and-forgets |
| ESP32-C3 bridge (`firmware/esp32-gimbal-bridge`) | Seeed XIAO C3 + XIAO CAN Bus Expansion Board (MCP2515 SPI + SN65HVD230). **Not** the C3’s TWAI pins. | `src/dji_can_protocol.*` (port of the CLI) | `src/main.cpp` (port of `dji_can_session.py`) |

The packet bytes, CRCs, CAN IDs, and command payloads are the same on both. Differences are physical (bit timing, RX filter, termination, how the bus is tapped) and session (the ESP32 never waits for a reply, never sends the user-params poll, and only parses a subset of inbound packets).

- **Bitrate**: `1_000_000` bps (1 Mbps), standard 11-bit IDs
- **SOF (start-of-frame)**: `0xAA`
- **Command set (gimbal)**: `0x0E`
- **Command set (camera record / focus-center)**: `0x0D`
- **CAN IDs**:
  - Host → gimbal: `0x223`
  - Gimbal → host: `0x222`

Hosts split each SDK packet into CAN frames of at most 8 data bytes. Those fragments **must not interleave** with another packet’s fragments on `0x223` (section 4.6).

Encodings were checked against canned SDK packets from a working CAN client. CRC-16 and CRC-32 match `dji_gimbal_cli.py` / `dji_can_protocol.cpp` byte-for-byte when the sequence number matches. Host requests use `CMD_TYPE = 0x03` except for a periodic user-params poll (`CMD_TYPE = 0x02`) that only the CLI can send as a one-shot; the ESP32 and the Python session do not send it.

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
             = 18 + len(DATA)
  ```

  Both hosts write byte 2 as `(cmd_length >> 8) & 0xFF`. Every packet they send is shorter than 256 bytes, so byte 2 is `0x00` (no extra flags).

- When parsing a reply:

  ```text
  pack_len = packet[1] | ((packet[2] & 0x03) << 8)
  ```

  Only the low 2 bits of byte 2 are length. Observed gimbal replies also leave the other bits clear.

The actual packet length must match `pack_len`, otherwise the packet is rejected. `validateSdkReply` also requires `len >= 16` and the reply bit (section 2.2). A reply with a return code and empty payload is 19 bytes (`18 + 1`).

### 2.2 CMD_TYPE and Reply Bit

Byte 3 carries the command type and some flags:

- For host-originated commands that expect a reply, the CLI and the working client use:

  ```text
  CMD_TYPE_REPLY_REQUIRED = 0x03
  ```

- One canned poll uses `CMD_TYPE = 0x02` (user-params, section 16). Treat that as "send, do not wait for a matching reply." The ESP32 and `dji_can_session.py` never send `0x02`; they use `0x03` for every request.

- For replies from the gimbal, bit `0x20` must be set; otherwise the packet is not treated as a reply:

  ```text
  is_reply = (packet[3] & 0x20) != 0
  ```

Unsolicited "push" packets also have this bit set; they are distinguished by their command IDs (e.g. `0x08`, `0x10`).

### 2.3 Sequence Number

Bytes 8–9 hold the sequence number in little-endian. Both hosts use the same counter (`_seq` / `g_seq`):

- Starts at `0x2210`.
- `nextSeq()` increments first, then returns, so the first packet on a boot is `0x2211`.
- When the counter is `>= 0xFFFD`, it is set to `0x0002` and then incremented (so the wrap produces `0x0003`).

You may treat the sequence as an opaque counter. The gimbal does not appear to enforce strict matching for basic operations. The ESP32 session and `dji_can_session.py` never match replies by SEQ; they classify inbound packets by `CMD_SET` / `CMD_ID` only. The CLI’s `request_reply()` matches `CMD_SET` / `CMD_ID` and skips pushes, but still ignores SEQ.

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

The gimbal/accessory CAN path (RSA port on current RS gimbals, or the 4-pin connector family used on DJI Focus motor units and other CAN accessories) exposes:

| Pin | Signal   | Notes                                                                 |
|-----|----------|------------------------------------------------------------------------|
| 1   | `VCC_5V` | 5V supplied **by** the gimbal/device. Never drive a different voltage into this pin. Current budget is unpublished — a Wi‑Fi radio can sag it (Focus Wheel LED goes red). |
| 2   | `GND`    | Common ground — always connect this regardless of how you power your adapter. |
| 3   | `CANH`   | CAN bus high                                                            |
| 4   | `CANL`   | CAN bus low                                                             |

Next to the 4-pin accessory port is a slide switch labeled **`S-BUS` / `CAN`**. It must be set to **`CAN`**
for this protocol. In the `S-BUS` position the port speaks analog S-BUS/PWM (legacy Ronin-S / SC / 2 joystick control), not the DJI R SDK packet format. If you wire up an adapter and get nothing but silence on `0x222`, check this switch first.

How this repo actually taps the bus:

- **ESP32-C3 + MCP2515 hat** (the live host): the hat is stacked on the XIAO and wired at the **DJI RS Focus Wheel** 4-pin, which is the same accessory CAN as the gimbal. Pin 1 (`VCC_5V`) goes to the XIAO **`5V`** pad (powers the C3, which then feeds 3V3 to the hat). Pin 2 `GND` common. Pins 3/4 to the hat CANH/CANL screw terminals. Unplug USB while that 5 V rail is connected, or the XIAO backfeeds 5 V onto pin 1.
- **USB Canable / SH-C31G**: `CANH`/`CANL` to the transceiver, `GND` common, `VCC_5V` left unconnected (adapter powered over USB).

The gimbal end is **not terminated**. The Seeed hat’s 120 Ω is pad **P1** on the back, open by default — short it. Do not add a second resistor at the Focus Wheel plug if P1 is already shorted. Bus-off with no ACK is usually CANH/CANL swapped, missing GND, P1 open, the gimbal unplugged, or the `CAN`/`S-BUS` switch.

Hat SPI (official XIAO CAN Bus Expansion Board; the C3 TWAI pins are unused):

| XIAO pin | GPIO | MCP2515 |
|----------|------|---------|
| D6       | 21   | INT     |
| D7       | 20   | CS      |
| D8       | 8    | SCK     |
| D9       | 9    | MISO    |
| D10      | 10   | MOSI    |

D6/D7 are also UART0. The firmware keeps the console on USB CDC so those pins stay on the MCP2515. Firmware assumes a **16 MHz** MCP2515 crystal (`-DMCP2515_CLOCK_MHZ=16`). An 8 MHz part cannot do 1 Mbps.

### 4.2 Other traffic on the same wire

The Focus Wheel (and other accessories) share this bus. Observed IDs that are **not** DJI R SDK:

| ID | Rate (approx.) | Notes |
|----|----------------|-------|
| `0x530` | ~400 Hz | Focus Wheel / accessory. Opaque. |
| `0x531` | with `0x530` | Same family. |
| `0x426` | with `0x530` | Same family. |

SDK replies on `0x222` are sparse unless parameter push is on. A host that accepts every ID will spend most of its RX bandwidth on `0x530`. The MCP2515 has **two** RX buffers; those frames will overwrite `0x222` fragments unless filtered in hardware.

The ESP32 default is a hardware filter on `0x222` only (`MASK = 0x7FF`, all six RX filters). Rebuild with `-DCAN_MCP_ACCEPT_ALL=1` to see the other IDs (then `GET /api/status` `can.rx_ids` lists them). Software still ignores anything that is not `0x222` before reassembly. A Canable has a much deeper USB queue, so the Python CLI can get away without a hardware filter.

### 4.3 Bit timing (1 Mbps)

Nominal bitrate is 1 Mbps. Sample point matters on the MCP2515 hat.

autowp’s 16 MHz / `CAN_1000KBPS` preset is 8 TQ, **62.5%** sample, triple-sample (`CNF 00/D0/82`). On this gimbal bus that drove the controller error-passive after a few seconds (TEC climb). The firmware overwrites CNF after `setBitrate`:

| Register | Value | Meaning (16 MHz, BRP = 0 → 8 TQ/bit) |
|----------|-------|--------------------------------------|
| CNF1     | `0x40` | SJW = 2, BRP = 0 |
| CNF2     | `0x98` | BTLMODE = 1, SAM = 0 (single sample), PS1 = 4 TQ, PropSeg = 1 TQ |
| CNF3     | `0x01` | PS2 = 2 TQ |

Sync (1) + PropSeg (1) + PS1 (4) = sample at **75%**. No triple-sample. This is closer to a typical CANable (~75–80% SP, SAM = 0). USB adapters that already work on the gimbal do not need this tweak; a second MCP2515 host does.

SPI to the MCP2515 is 8 MHz (Fosc/2 for a 16 MHz crystal) so the two RX buffers drain before the next `0x222` fragment.

### 4.4 CAN IDs and Framing

The SDK packet is transported over CAN as follows:

- **Host → gimbal**:
  - Standard 11-bit CAN ID: `0x223`
  - Data: 1–8 bytes per frame (last fragment may be short; never 0, never > 8)
  - Not extended (`is_extended_id = false`; firmware masks `id & 0x7FF`)
- **Gimbal → host**:
  - Standard 11-bit CAN ID: `0x222`
  - Data: 0–8 bytes per frame

The host sends each SDK packet in 8-byte chunks with **no inter-frame delay** beyond however long the controller takes to ACK:

```text
for i in range(0, len(pkt), 8):
    send CAN frame with arbitration_id=0x223 and data=pkt[i : i+8]
```

Request sizes (`cmd_length = 18 + len(DATA)`):

| DATA bytes | Total | Frames | Commands |
|-----------:|------:|--------|----------|
| 0 | 18 | 8+8+2 | stiffness, user-params (empty) |
| 1 | 19 | 8+8+3 | angle, push-enable, ActiveTrack, `cam-cmd`, limits `01` |
| 2 | 20 | 8+8+4 | recenter/selfie, record / focus-center, focus-get |
| 3 | 21 | 8+8+5 | sleep/wake, AutoTune, motor-calib, user-params-poll |
| 5 | 23 | 8+8+7 | focus-set |
| 7 | 25 | 8+8+8+1 | speed |
| 8 | 26 | 8+8+8+2 | position |

Observed reply / push lengths from a working client (it only starts reassembly for these; both of our hosts accept any CRC-valid length):

| Total length | Typical contents |
|-------------:|------------------|
| 20 (`0x14`) | Camera `0x0D/0x01` reply |
| 25 (`0x19`) | Speed reply, or compact limits, or short `0x08` push |
| 26 (`0x1A`) | Angle reply `0x0E/0x02` |
| 28 (`0x1C`) | User-params reply `0x0E/0x0B` |
| 40 (`0x28`) | Parameter push `0x0E/0x08` with extra fields |

### 4.5 Reassembly

Both hosts (`dji_gimbal_cli.py` `reassemble()`, `dji::Reassembler`, `PacketReassembler`) use the same byte state machine on `0x222` payloads only:

1. Wait for `SOF = 0xAA`.
2. Read length low (byte 1).
3. Read byte 2; `pack_len = byte1 | ((byte2 & 0x03) << 8)`.
4. Accumulate until 12 bytes. Verify CRC-16 over bytes 0–9 against bytes 10–11. On mismatch, drop the buffer and return to step 1 (**the current byte is not retried as a new SOF**).
5. Accumulate until `pack_len`. Verify CRC-32 over `packet[:-4]`. On mismatch, drop. On match, emit the packet.

There is no timeout that abandons a half-packet; a lost fragment leaves the machine in step 4 until a later CRC-32 failure or a later CRC-16 failure after a reset.

`validateSdkReply` then requires the reply bit (`byte3 & 0x20`), matching `pack_len`, and both CRCs again. It does not check SEQ.

### 4.6 Sending constraints

The gimbal reassembles `0x223` the same way: one byte stream. If two SDK packets’ fragments interleave, CAN ACKs still succeed and the gimbal sees garbage (CRC-16 fails, packet dropped).

The ESP32 therefore takes a TX mutex around the whole `for i in range(0, len, 8)` loop. `loop()` (held speed, zoom ramp, angle poll) and HTTP/WebSocket handlers all transmit; without the lock they stomp each other. If a chunk fails, it retries the **entire** SDK packet up to 3 times with 50 ms between attempts. The Python session is single-threaded on the bus, so it does not need a lock.

Do not insert unrelated frames (including the ESP32’s diagnostic `0x100` probe) in the middle of an SDK packet. The probe is a lone `0x100` / `{0xA5}` used only to see whether any peer ACKs; it is not an R SDK frame.

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

Camera record and Ronin focus-center use `CMD_SET = 0x0D` (section 22). Those are a different command set, not extra `0x0E` cmd_ids.

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
|  0x0E   | 0x0F   | host ↔ gimbal   | AutoTune (retune motors for payload) |
|  0x0E   | 0x10   | gimbal → host   | AutoTune / calibration status push   |
|  0x0E   | 0x11   | host ↔ gimbal   | ActiveTrack toggle                   |
|  0x0E   | 0x12   | host ↔ gimbal   | Focus motor (position / query / autocal) |
|  0x0D   | 0x00   | host → camera   | Record / center-focus (section 22)   |
|  0x0D   | 0x01   | host → camera   | Camera query, DATA `01`              |

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

Both hosts currently use:

- Absolute mode (`ctrl` bit 0 set).
- All axes valid (bits 1–3 cleared).

The packet builder defaults `time` to 0.2 s. The live session / ESP32 WebSocket and REST APIs default `time_s` to **0.4 s** (clamped to 0.0–25.5 s) unless the client sends a value.

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

- Bit 7 set means the host is taking speed control.
- A working CAN client sends `0x80`. The CLI, Python session, and ESP32 all send `0x80` (`SPEED_CTRL_TAKEOVER`).
- Older notes used `0x88` (bit 3 also set). That still builds; pass `ctrl=0x88` if you need to compare.

Pan/tilt from a joystick or on-screen pad is this command, not position (`0x00`). Position is for absolute/incremental moves (presets). On RS 4 / RS 4 Pro / RS 5, axis endpoints set on the gimbal itself are ignored by the SDK speed/position path unless the host is driving in a joystick-style mode that honours the gimbal's own speed, smoothness, and endpoints.

The live session (Python and ESP32) does not fire-and-forget a non-zero speed:

- Held rates are re-sent at **20 Hz** while deflected (threshold 0.05 °/s per axis).
- A zero-speed packet is sent once when the stick returns to center.
- **Deadman 250 ms**: if no new hold update arrives (Wi‑Fi drop), send one zero-speed packet. Hardware PTZ on the ESP32 uses an explicit stop instead of this timer.
- Rates of (0,0,0) are still a valid SDK packet (25 bytes, ctrl `0x80`).

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

The ESP32 / Python session only request **attitude** (`DATA = 0x01`), and only as the 20 Hz fallback while parameter push is stale (section 12.2). Joint angles (`0x02`) are CLI-only.

---

## 10. Obtain Gimbal Limit Angles – CMD_ID 0x04

**Purpose**: Obtain configured gimbal axis endpoints (min/max pan, roll, tilt).

- **Command set**: `0x0E`
- **Command ID**: `0x04`

These are the software travel limits used to stop cable wrap and overshoot. Resetting them restores full-range motion.

### 10.1 Request Payload

A working CAN client sends a single byte:

```text
query uint8   # 0x01
```

Empty `DATA` is also used by the original DJI R SDK write-up. The CLI sends `01` by default (`build_obtain_gimbal_limit_angle()`). The 13-byte reply layout below has a 1-byte prefix that matches this query byte.

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

All int16 values are scaled by 0.1 when presented in degrees.

3. **6 bytes after `ret_code`** (25-byte packet). A working client stores these six bytes as the endpoint snapshot and does not parse 6×int16. The CLI prints that compact payload as hex when it sees it.

If decoding fails but `ret_code == 0x00`, the CLI prints the raw payload length and hex.

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

The ESP32 has `buildObtainMotorStiffness()` but does not send it (no REST/WS command, no parser).

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

The ESP32 and `dji_can_session.py` send enable (`DATA = 01`) once at connect, then a focus-get. While no `0x08` push has arrived for **>350 ms** (including before the first one), they poll `Obtain gimbal angle` (`0x0E/0x02`, `DATA = 01`) at 20 Hz. Push-off from the UI stops the unsolicited frames; the stale-poll then takes over until push is enabled again. The CLI `-c listen` enables push and prints; it does not run this fallback.

---

## 13. Gimbal Parameter Push – CMD_ID 0x08 (Unsolicited)

**Purpose**: Continuous push of gimbal parameters, including current angles.

- **Command set**: `0x0E`
- **Command ID**: `0x08`
- **Direction**: gimbal → host (unsolicited)

### 13.1 Payload Layout

Unsolicited push does not use byte 14 as a DJI return code. A working RX parser treats it as flags:

```text
flags    uint8   # bit 0 set → the next 6 bytes are yaw/roll/pitch
yaw      int16   (0.1°)
roll     int16   (0.1°)
pitch    int16   (0.1°)
…        optional extra bytes (40-byte / 0x28 pushes)
```

Total packet length 25 (`0x19`) is the short form. Length 40 (`0x28`) carries extra fields after the three angles.

The CLI (`-c listen`), `dji_can_session._push_angles()`, and `dji::parsePushAngles()` all decode flags-then-int16 first. If that does not fit, they fall back to the angle-reply layout (`data_type` + 3×int16 after byte 14). The ESP32 applies a successful parse to telemetry (`yaw`/`roll`/`pitch`) and stamps `lastPushMs` so the 350 ms stale poll (section 12.2) backs off.

---

## 14. AutoTune Status Push – CMD_ID 0x10 (Unsolicited)

**Purpose**: Status updates while AutoTune (`0x0F`) is running.

- **Command set**: `0x0E`
- **Command ID**: `0x10`
- **Direction**: gimbal → host (unsolicited)

Payload structure is currently treated as opaque. The CLI prints:

```text
[push] auto calibration status (0x10): ret=0xRR len=NN hex=...
```

where:

- `RR` is byte 14 (parsed as `ret_code`, but this is a push).
- `NN` is payload length.

The ESP32 receives `0x10` (it passes `validateSdkReply`) and then ignores it — not decoded, not surfaced on the WebSocket.

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

Two encodings are in use:

1. **Explicit query (CLI `user-params`)**  
   Empty `DATA`, `CMD_TYPE = 0x03`.

2. **Periodic poll (CLI `user-params-poll` only)**  
   `CMD_TYPE = 0x02`, `DATA = 00 22 23`. A working client sends this on a ~500 ms cadence (25 ticks of a 20 ms loop) and does not wait for a reply. The three payload bytes look like a dummy byte plus the CAN IDs `0x22` / `0x23`; they have not been decoded further.

The ESP32 has `buildObtainGimbalUserParams()` and `buildObtainGimbalUserParamsPoll()` but **does not call either**. `dji_can_session.py` also never sends `0x0B`. There is no REST/WS command for it.

### 16.2 Reply Payload

The CLI currently does not interpret the structure of the user parameters; instead, it reports:

- `ret_code` (success or error).
- `raw_len` (payload length).
- `hex` (hex representation of the payload).

Future work could decode individual user parameters based on further firmware analysis.

---

## 17. Sleep / Wake – CMD_ID 0x0C

**Purpose**: Sleep the gimbal between takes, or wake it. Instant on/off of the motors, not a full power cycle.

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

Do not confuse this with sleep/wake (`CMD_ID = 0x0C`, payload `23 01 xx`). A working client sends recenter `FE 01` only; selfie `FE 02` is still in the CLI but was not in that client's canned TX list.

---

## 19. AutoTune – CMD_ID 0x0F

**Purpose**: Retune the gimbal motors for the current payload (AutoTune). This is not focus-motor lens calibration (`0x12` `02 00 01`, section 21.3). Status arrives as unsolicited `CMD_ID = 0x10` pushes (section 14).

- **Command set**: `0x0E`
- **Command ID**: `0x0F`

A working CAN client keeps this packet next to recenter (`FE 01`) and ActiveTrack (`03`) in the same template block. CRC-16/CRC-32 check out.

### 19.1 Request Payload

Three bytes. Total SDK packet length is 21.

```text
0x00  uint8
0x01  uint8
0x01  uint8
```

CLI: `build_calibrate()` / `-c calibrate` / `-c autotune` → `0x0E / 0x0F / 00 01 01`

### 19.2 Reply

- Reply command: `CMD_SET = 0x0E`, `CMD_ID = 0x0F`.
- The CLI prints `cmd_set`, `cmd_id`, `ret_code`, and payload hex.
- After success, watch `CMD_ID = 0x10` pushes (`-c listen`) for calibration progress.

This starts a physical motor-tune sequence. Use it when you mean to.

---

## 20. ActiveTrack Toggle – CMD_ID 0x11

**Purpose**: Toggle ActiveTrack on the gimbal. The gimbal does not report whether tracking is currently on or off; send the same packet again to stop.

Requires a tracking accessory (RavenEye, or the Intelligent Tracking Module / Enhanced module on RS 4 / RS 4 Pro / RS 5). Center the subject in frame before toggling.

Speed commands (`0x01`) can still reframe while tracking is on, on gimbals that honour joystick-style control during ActiveTrack.

- **Command set**: `0x0E`
- **Command ID**: `0x11`

### 20.1 Request Payload

Single byte:

```text
0x03  uint8  # toggle ActiveTrack start/stop
```

### 20.2 Reply

- Reply command: `CMD_SET = 0x0E`, `CMD_ID = 0x11`.
- `ret_code == 0x00` means the toggle was accepted, not that tracking is now "on".

---

## 21. Focus Motor Control and Query – CMD_ID 0x12

**Purpose**: Control and query the DJI Focus Motor.

On RS 2 / RS 2 Pro / RS 3 Pro / RS 4 / RS 4 Pro / RS 5 this motor is typically used as **zoom**: after autocalibration, position `0` is wide and `4096` is tight. Set the motor LED to F (Focus) mode even when driving zoom; the SDK only talks to that mode. One motor at a time.

Camera center-focus (tap-to-focus on the body) is `CMD_SET = 0x0D` (section 22), not this command.

- **Command set**: `0x0E`
- **Command ID**: `0x12`

There are three request layouts:

1. Set motor position (`focus-set` / `zoom-set`).
2. Get motor position (`focus-get`).
3. Autocalibrate lens endpoints (`motor-calib`).

### 21.1 Set Focus Position (`focus-set`)

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
  - Range `0–4096` after a successful `motor-calib`.
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

The ESP32 does not send `focus-set` at the WebSocket rate. `zoom` (and PTZ zoom/focus) set a **target**; firmware advances a trapezoidal ramp at 20 Hz (`vmax` default 900 counts/s, `accel` default 1800 counts/s²) and transmits `focus-set` only when the commanded integer position changes. The first `0x12` reply after boot (or after `motor-calib`) seeds the ramp so the motor does not jump. Same algorithm as `zoom_ramp_step()` in `dji_can_session.py`. This ramp is host-side, not part of the gimbal protocol.

### 21.2 Get Focus Position (`focus-get`)

**Request payload**:

```text
0x15  uint8
0x00  uint8
```

These bytes match the focus-position query used by a working CAN client.

**Reply payload**:

Uses the same parsing as `focus-set` replies:

- The last 4 bytes of `data` are interpreted as the current focus position, encoded as little-endian uint32.

If decoding fails, the CLI falls back to printing:

- `ret_code`
- `len(data)`
- `data` in hex.

### 21.3 Motor autocalibrate (`motor-calib`)

**Request payload**:

```text
0x02  uint8
0x00  uint8
0x01  uint8
```

This is the Focus Motor Autocalibration command (lens endpoints). Distinct from gimbal AutoTune (`0x0F`). Total SDK packet length is 21.

CLI: `build_motor_calib()` / `-c motor-calib` (alias `-c focus-02`). The motor runs to both ends of the lens; hold the lens if the gear slips.

The ESP32 `motor-calib` command also clears the zoom-ramp state, then sends this packet and a `focus-get` so the next reply re-seeds the ramp.

---

## 22. Camera Command Set 0x0D

**Purpose**: Trigger record and center-focus on a camera attached through the DJI camera-control cable (the USB-C / control cable that ships with most RS gimbals). No separate network path on the camera is required. Nikon, Fujifilm, Sigma, and similar bodies that the gimbal already supports are the typical targets. Sony / Blackmagic over Wi-Fi or Bluetooth do not use this command set.

- **Command set**: `0x0D`
- **Command ID**: `0x00` for the four CLI commands below

| CLI | DATA |
|-----|------|
| `rec-start` | `03 00` |
| `rec-stop` | `04 00` |
| `focus-center-start` | `05 00` |
| `focus-center-stop` | `0B 00` |

Total SDK packet length is 20 (2-byte payload).

### 22.1 Request Payload

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

### 22.2 Reply

- Reply command: `CMD_SET = 0x0D`, `CMD_ID = 0x00`.
- The CLI prints `cmd_set`, `cmd_id`, `ret_code`, and payload hex from the `0x222` reply.
- These commands execute if accepted. `rec-start` starts recording on a camera the gimbal can already control.

### 22.3 Camera CMD_ID 0x01 (`cam-cmd`)

A working client also sends:

```text
CMD_SET = 0x0D
CMD_ID  = 0x01
DATA    = 01
```

Total SDK packet length is 19. A working client sends this next to record-start and treats a reply whose first payload byte is `0x02` as the interesting state. The CLI exposes it as `cam-cmd` and prints that byte. The ESP32 sends the same packet (`{"cmd":"cam-cmd"}` / `POST /api/command/cam-cmd`) and logs `ret` / first payload byte on serial; it does not latch the value into WebSocket state.

---

## 23. High-Level Interaction Patterns

The CLI (`dji_gimbal_cli.py`) demonstrates several typical flows:

### 23.1 Continuous Angle Streaming

- `angle`:
  - Repeatedly sends `CMD_SET = 0x0E`, `CMD_ID = 0x02`, `DATA = [0x01]` (attitude angles).
  - Parses yaw/roll/pitch and prints them at a given interval.
- `joint`:
  - Same, but `DATA = [0x02]` (joint angles).

### 23.2 One-Shot Info Query

The `info` command performs a sequence:

1. Obtain attitude angles (`0x02`).
2. Obtain module version (`0x09`).
3. Obtain gimbal limit angles (`0x04`, DATA `01`).
4. Obtain motor stiffness (`0x06`).

All results are printed in a human-readable format for quick diagnostics.

### 23.3 Push-Based Telemetry

- `push-on`:
  - Sends `CMD_SET = 0x0E`, `CMD_ID = 0x07`, `DATA = [0x01]`.
  - Enables push messages.
- `listen`:
  - Ensures push is enabled.
  - Continuously reads from CAN ID `0x222`.
  - Reassembles SDK packets.
  - Prints `0x08` pushes (25-byte and 40-byte), `0x10` AutoTune status, and `0x0D/0x01` camera replies.

### 23.4 One-shot motion, camera, and motor commands

These one-shot commands print the TX encoding, then the `0x222` reply as `cmd_set`, `cmd_id`, `ret_code`, and payload hex. They execute if the gimbal accepts them.

```text
python dji_gimbal_cli.py COM6 -c sleep
python dji_gimbal_cli.py COM6 -c wake
python dji_gimbal_cli.py COM6 -c calibrate
python dji_gimbal_cli.py COM6 -c motor-calib
python dji_gimbal_cli.py COM6 -c rec-start
python dji_gimbal_cli.py COM6 -c rec-stop
python dji_gimbal_cli.py COM6 -c focus-center-start
python dji_gimbal_cli.py COM6 -c focus-center-stop
python dji_gimbal_cli.py COM6 -c cam-cmd
python dji_gimbal_cli.py COM6 -c activetrack
```

| CLI | CMD_SET | CMD_ID | DATA | CMD_TYPE |
|-----|---------|--------|------|----------|
| `sleep` | `0x0E` | `0x0C` | `23 01 01` | `0x03` |
| `wake` | `0x0E` | `0x0C` | `23 01 00` | `0x03` |
| `calibrate` / `autotune` | `0x0E` | `0x0F` | `00 01 01` | `0x03` |
| `recenter` | `0x0E` | `0x0E` | `FE 01` | `0x03` |
| `activetrack` | `0x0E` | `0x11` | `03` | `0x03` |
| `limit` | `0x0E` | `0x04` | `01` | `0x03` |
| `user-params-poll` | `0x0E` | `0x0B` | `00 22 23` | `0x02` |
| `motor-calib` | `0x0E` | `0x12` | `02 00 01` | `0x03` |
| `speed` | `0x0E` | `0x01` | 3×int16 + `80` | `0x03` |
| `rec-start` | `0x0D` | `0x00` | `03 00` | `0x03` |
| `rec-stop` | `0x0D` | `0x00` | `04 00` | `0x03` |
| `focus-center-start` | `0x0D` | `0x00` | `05 00` | `0x03` |
| `focus-center-stop` | `0x0D` | `0x00` | `0B 00` | `0x03` |
| `cam-cmd` | `0x0D` | `0x01` | `01` | `0x03` |

Sleep, wake, record, center-focus, recenter, ActiveTrack, and motor-calib match canned packets (CRC included, once SEQ is aligned). AutoTune, limits `01`, `cam-cmd`, speed ctrl `0x80`, and the user-params poll come from the same client. `focus-02` is an alias for `motor-calib`. `zoom-set` is an alias for `focus-set`.

---

## 24. Implementing a Custom Client

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
   - Use `CMD_SET` and `CMD_ID` as documented above (`0x0E` gimbal, `0x0D` camera-control cable).
   - Parse payloads according to the sections in this document.
6. **Handle return codes**:
   - Use `ret_code` to classify success vs. parse/execute/undefined errors.

This document, together with the `dji_gimbal_cli.py` implementation, should provide a complete reference for controlling the DJI RS gimbal over CAN using the DJI R SDK packet format.


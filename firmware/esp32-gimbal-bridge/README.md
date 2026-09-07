# ESP32 WiFi ↔ CAN Gimbal Bridge

ESP32 firmware that speaks the DJI R SDK CAN protocol (documented in
[`docs/DJI_R_SDK_Protocol.md`](../../docs/DJI_R_SDK_Protocol.md), and previously exercised
via [`dji_gimbal_cli.py`](../../dji_gimbal_cli.py) over a wired SH-C31G/Canable adapter) and
exposes it as a phone/browser-friendly WebSocket + REST API with a built-in joystick web UI.
The XIAO is powered from the DJI RS Focus Wheel's 4-pin CAN port (`VCC_5V` + `GND`).

This firmware tracks the **golden Python implementation** as its standard:

- [`src/dji_can_protocol.h`](src/dji_can_protocol.h)/`.cpp` ports the packet framing,
  CRC-16/CRC-32, builders, and parsers from `dji_gimbal_cli.py` (gimbal CmdSet `0x0E`
  and camera CmdSet `0x0D`).
- `src/main.cpp` ports the session behavior of `dji_can_session.py` (startup push-enable +
  focus-position query, 20 Hz held-speed with a 250 ms deadman, 20 Hz angle polling while
  parameter push is stale >350 ms, and a 20 Hz trapezoidal zoom ramp with configurable
  vmax/accel) and the API surface of `dji_gimbal_web.py`.

If you change behavior in one implementation, check whether the other needs the same fix.

## Hardware

**MCU**: [Seeed Studio XIAO ESP32C3](https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/)
plugged into the [XIAO CAN Bus Expansion Board](https://wiki.seeedstudio.com/xiao-can-bus-expansion/).
That hat is an MCP2515 (SPI CAN controller) plus an SN65HVD230 transceiver. It does **not**
use the C3's built-in TWAI peripheral. Firmware talks to the MCP2515 in `src/can_hw.cpp`.

Hat pin map (Seeed wiki + Zephyr overlay):

| XIAO pin | GPIO | Hat |
|----------|------|-----|
| D6       | 21   | MCP2515 INT |
| D7       | 20   | MCP2515 CS |
| D8       | 8    | SCK |
| D9       | 9    | MISO |
| D10      | 10   | MOSI |
| 3V3 / GND | —  | power |

D6/D7 are also UART0 TX/RX. The env sets `-DARDUINO_USB_CDC_ON_BOOT=1` so the serial
console stays on native USB.

**Focus Wheel wiring** — the hat is on the same accessory CAN as the gimbal, tapped at the
DJI RS Focus Wheel's 4-pin CAN port (same pinout as [`docs/DJI_R_SDK_Protocol.md` §4.1](../../docs/DJI_R_SDK_Protocol.md#41-physical-connector)):

| Focus Wheel pin | Signal   | Wire to |
|-----------------|----------|---------|
| 1               | `VCC_5V` | XIAO `5V` (powers the C3 and, through it, the hat's 3V3) |
| 2               | `GND`    | Hat GND (common with XIAO GND) |
| 3               | `CANH`   | Hat CANH screw terminal |
| 4               | `CANL`   | Hat CANL screw terminal |

The slide switch next to the gimbal accessory port must be **`CAN`**, not `S-BUS`.

**Termination.** The gimbal is not terminated. Short pad **P1** on the back of the hat
(120 Ω, open by default). Do not add a second resistor across CANH/CANL at the Focus Wheel
plug if P1 is already shorted.

**1 Mbps.** Firmware assumes a **16 MHz** MCP2515 crystal (`-DMCP2515_CLOCK_MHZ=16`).
An 8 MHz part cannot do 1 Mbps. Check the marking when the board arrives. Rebuild with
`MCP2515_CLOCK_MHZ=8` only if it really is 8 MHz, and then 1 Mbps will not work.

**RX filter.** Default hardware filter is CAN ID `0x222` only. The same accessory wire
carries a second protocol at ~400 Hz (`0x530` / `0x531` / `0x426`). The MCP2515 has two
RX buffers, so letting that flood through would drop SDK frames. For debug, rebuild with
`-DCAN_MCP_ACCEPT_ALL=1`.

**Push telemetry.** At boot the firmware sends `Set parameter push (0x0E/0x07) enable`
once and queries the focus motor position, matching the golden session's connect
sequence. While push frames stop arriving for >350 ms, it falls back to polling
`Obtain gimbal angle` at 20 Hz (same as the Python session). Toggle push from the UI
(Telemetry section) if you want the bus quiet.

## Power

The board runs from the Focus Wheel CAN port: pin 1 (`VCC_5V`) into the XIAO **`5V`** pad,
pin 2 (`GND`) to hat/XIAO GND. That 5 V is the gimbal accessory rail, passed through the
Focus Wheel. The C3's onboard regulator then feeds 3V3 into the MCP2515 hat. Unplug USB
while the accessory rail is connected, or the XIAO will backfeed 5 V onto that pin (Seeed
documents a series Schottky if you need USB and bus power at the same time).

Nobody publishes a current budget for that accessory pin. If the rail sags, the gimbal's
own CAN transceiver (and the Focus Wheel) go with it — if the wheel LED goes red when the
ESP32 Wi-Fi radio wakes, the rail is starving.

Deep-sleep-on-unplug (`GIMBAL_SLEEP_ENABLED`, D3 presence divider, optional LiPo on the
XIAO `BAT` JST) is still in firmware but **off**. This build is bus-powered: unplug the
Focus Wheel cable and the board dies with the 5 V rail.

> **Serial console note**: the XIAO ESP32C3 has no separate USB-serial chip — it uses the
> C3's native USB peripheral. `platformio.ini` already sets `-DARDUINO_USB_CDC_ON_BOOT=1`,
> which is required for `Serial.print()`/`pio device monitor` to work over that port; without
> it you'll see nothing on the console.

## First-time setup

1. Flash the firmware (see Build below).
2. On first boot (or whenever it can't join a known network), the device opens a WiFi
   access point named **`DJI-Gimbal-Setup`**. Connect to it from your phone/laptop; a
   captive portal should pop up (or browse to `192.168.4.1`) to pick your WiFi network
   and enter its password. This is handled by [WiFiManager](https://github.com/tzapu/WiFiManager)
   — no credentials are hardcoded in the firmware.
3. Once connected, check your router's client list (or the serial monitor at 115200 baud)
   for the device's IP address, then open `http://<device-ip>/` in a browser.

## Build & flash (PlatformIO)

```bash
cd firmware/esp32-gimbal-bridge
pio run -t upload            # build + flash firmware
pio run -t uploadfs          # upload the web UI (data/index.html) to LittleFS
pio device monitor           # serial log at 115200 baud
```

`uploadfs` only needs to be re-run when `data/` changes; `upload` is for firmware changes.
These commands target the default env (`seeed_xiao_esp32c3`). Flash from PowerShell or cmd,
not Git Bash — see `.cursor/rules/platformio-windows-flash.mdc`.

Note: XIAO ESP32C3 boards ship with `BOOT` (D9) held for manual bootloader entry on some
USB-driver setups. D9 is also MCP2515 MISO once the hat is on, which does not matter in
the bootloader. If `pio run -t upload` can't find/flash the board, hold `BOOT`, tap
`RESET`, then release `BOOT` once the upload starts.

## Control surface

The API mirrors `dji_gimbal_web.py`. Command names are identical to the Python web
server's `/api/command/<name>` allow-list.

### WebSocket (`ws://<device-ip>/ws`) — use this for continuous/real-time control

Browser → device, JSON text frames:

| `cmd`          | Fields                              | Notes                                                        |
|----------------|--------------------------------------|---------------------------------------------------------------|
| `speed`        | `yaw`, `roll`, `pitch` (°/s), `hold` | `hold:true` re-sends at 20 Hz until released; send at ~20 Hz while deflected (the UI does). `hold:false` fires once. |
| `position`     | `yaw`, `roll`, `pitch` (°), `time_s` | Absolute go-to (CmdSet `0x0E` CmdID `0x00`).                   |
| `zoom`         | `position` (0–4096)                  | Focus-motor target; the device ramps to it (see below).        |
| `zoom_profile` | `vmax` (50–8000 /s), `accel` (50–40000 /s²) | Ramp tuning for `zoom`.                               |
| `<name>`       | —                                    | Any named command below.                                       |

Named commands (same strings over WS `{"cmd": name}` and REST `POST /api/command/<name>`):

| Name                 | Protocol                         | Effect                                   |
|----------------------|-----------------------------------|-------------------------------------------|
| `sleep` / `wake`     | `0x0E/0x0C` `23 01 01` / `23 01 00` | Sleep/wake gimbal motors.               |
| `recenter` / `selfie`| `0x0E/0x0E` `FE 01` / `FE 02`      | Recenter once / selfie pose.            |
| `calibrate`          | `0x0E/0x0F` `00 01 01`             | AutoTune gimbal motors for the payload. |
| `motor-calib`        | `0x0E/0x12` `02 00 01`             | Focus-motor endpoint calibration; clears the zoom ramp state, then re-queries position. |
| `activetrack`        | `0x0E/0x11` `03`                   | Toggle ActiveTrack (no status feedback).|
| `rec-start` / `rec-stop` | `0x0D/0x00` `03 00` / `04 00`  | Camera record (needs DJI camera-control cable). |
| `focus-center-start` / `focus-center-stop` | `0x0D/0x00` `05 00` / `0B 00` | Camera center-focus. |
| `push-on` / `push-off` | `0x0E/0x07` `01` / `00`          | Enable/disable gimbal parameter push.   |
| `cam-cmd`            | `0x0D/0x01` `01`                   | Camera query; reply logged on serial.   |
| `limit`              | `0x0E/0x04` `01`                   | Query limit angles; reply parsed into state. |
| `version`            | `0x0E/0x09`                        | Query module version; parsed into state. |
| `zoom_get`           | `0x0E/0x12` `15 00`                | Query focus-motor position.             |
| `stop`               | —                                  | Release held speed and send one zero-speed packet. |

Device → browser: the state snapshot every 50 ms (20 Hz):

```json
{
  "connected": true, "adapter": "mcp2515", "interface": "can",
  "yaw": 12.3, "roll": -0.4, "pitch": -89.9,
  "zoom": 2048, "zoom_target": 3000, "zoom_vmax": 900, "zoom_accel": 1800,
  "last_rx_age_s": 0.02, "last_error": null, "tx_ok": 1234, "rx_ok": 980,
  "rssi": -58, "can_state": "running", "version": "1.2.3.4"
}
```

(`version` appears after a `version` command reply; `limits` likewise after `limit`.)

### REST (`/api/*`) — JSON bodies, one-shot commands

- `GET  /api/state` → the snapshot above (same keys as the WS stream)
- `POST /api/speed` `{"yaw":0,"roll":0,"pitch":0,"hold":true}`
- `POST /api/position` `{"yaw":0,"roll":0,"pitch":0,"time_s":0.8}`
- `POST /api/zoom` `{"position":2048}`
- `POST /api/zoom/profile` `{"vmax":900,"accel":1800}`
- `POST /api/command/<name>` — any named command from the table above
- `GET  /api/status` → `{wifi_rssi, ip, ws_clients, can:{...}}` (ESP32 CAN diagnostics)
- `POST /api/can/probe` → TX a lone `0x100` frame to check for a bus peer

### Zoom ramp

`zoom` sets a *target*; firmware advances the commanded position toward it at 20 Hz with a
trapezoidal profile (`vmax`, `accel`), sending `focus-set` only when the integer position
changes. The first focus-motor reply after boot (or after `motor-calib`) seeds the ramp so
the motor never jumps on the first command. This is a direct port of `zoom_ramp_step()` in
`dji_can_session.py`.

## Safety: the speed watchdog

Because control now travels over WiFi instead of a wired connection, a dropped connection
must not leave the gimbal spinning at whatever speed it last received. `main.cpp` tracks the
time of the last `speed` command and **zeroes the gimbal's speed if none arrives within
250 ms** while a non-zero speed is active (the same `DEADMAN_S = 0.25` as the Python
session), and also zeroes speed when the last WebSocket client disconnects. If you build a
different client than the bundled web UI, make sure it either sends `speed` updates
continuously while deflected or explicitly sends `stop`/a zero `speed` on release — don't
rely on a single "fire and forget" command for anything continuous.

## Known gaps / left to reverse-engineer

Everything about the packet framing and the commands used here (`0x00`, `0x01`, `0x02`,
`0x04`, `0x07`, `0x08`, `0x09`, `0x0B`, `0x0C`, `0x0E`, `0x0F`, `0x11`, `0x12`, plus camera
`0x0D/0x00` and `0x0D/0x01`) is carried over as-is from the validated Python CLI. Remaining
gaps (see `docs/DJI_R_SDK_Protocol.md` §Status for the full list):

- `0x06` (stiffness) and `0x0B` (user params) have builders (`buildObtainMotorStiffness`,
  `buildObtainGimbalUserParams[Poll]`) but no REST/WS command or parser wired up — same as
  the Python web server, which doesn't expose them either.
- `0x10` (auto-calibration status push) is received but not parsed/surfaced.

## Repo layout

```
firmware/esp32-gimbal-bridge/
  platformio.ini           PlatformIO env (XIAO C3 + MCP2515 hat)
  src/
    main.cpp                WiFi/WebSocket/REST glue, safety watchdog
    can_hw.h/.cpp           MCP2515 CAN backend for the Seeed hat
    dji_can_protocol.h/.cpp  Packet framing ported from dji_gimbal_cli.py
  data/
    index.html               Joystick + telemetry web UI (served via LittleFS)
```

# ESP32 WiFi ↔ CAN Gimbal Bridge

Battery-powered ESP32 firmware that speaks the DJI R SDK CAN protocol (documented in
[`docs/DJI_R_SDK_Protocol.md`](../../docs/DJI_R_SDK_Protocol.md), and previously exercised
via [`dji_gimbal_cli.py`](../../dji_gimbal_cli.py) over a wired SH-C31G/Canable adapter) and
exposes it as a phone/browser-friendly WebSocket + REST API with a built-in joystick web UI.

This firmware is a straight port of the packet framing, CRC-16/CRC-32, and command builders
from `dji_gimbal_cli.py` — see [`src/dji_can_protocol.h`](src/dji_can_protocol.h)/`.cpp`. If you
change something there, check whether the Python CLI needs the same fix (and vice versa).

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

**Gimbal wiring** — see [`docs/DJI_R_SDK_Protocol.md` §4.1](../../docs/DJI_R_SDK_Protocol.md#41-physical-connector):

| Gimbal port pin | Signal   | Wire to |
|------------------|----------|---------|
| 1                | `VCC_5V` | Optional: Schottky into XIAO `5V`, divider tap to D3. See Power Management. Skip until the hat is on the bench. |
| 2                | `GND`    | Hat GND (always) |
| 3                | `CANH`   | Hat CANH screw terminal |
| 4                | `CANL`   | Hat CANL screw terminal |

The slide switch next to that port must be **`CAN`**, not `S-BUS`.

**Termination.** The gimbal is not terminated. Short pad **P1** on the back of the hat
(120 Ω, open by default). Do not add a second resistor across CANH/CANL at the gimbal
plug if P1 is already shorted.

**1 Mbps.** Firmware assumes a **16 MHz** MCP2515 crystal (`-DMCP2515_CLOCK_MHZ=16`).
An 8 MHz part cannot do 1 Mbps. Check the marking when the board arrives. Rebuild with
`MCP2515_CLOCK_MHZ=8` only if it really is 8 MHz, and then 1 Mbps will not work.

**RX filter.** Default hardware filter is CAN ID `0x222` only. The same accessory wire
carries a second protocol at ~400 Hz (`0x530` / `0x531` / `0x426`). The MCP2515 has two
RX buffers, so letting that flood through would drop SDK frames. For debug, rebuild with
`-DCAN_MCP_ACCEPT_ALL=1`.

Do not auto-spam push-enable. Turn telemetry on from the UI when you want `0x222`.

## Power Management

The XIAO ESP32C3 needs a single-cell 3.7V LiPo on its onboard `BAT` JST connector — it has
its own charge management IC built in (fast charge ~380 mA / trickle ~40 mA, not user-
adjustable), so **no separate TP4056 module is needed**. Leave this unwired until the hat
is seated and CAN works. The gimbal's `VCC_5V` pin does two jobs for that battery, both
driven off the same wire:

1. **Charges it while connected.** Per [Seeed's own guidance](https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/)
   for using the `5V` pin as an external power input: put a small Schottky diode (e.g.
   1N5817/SS14) in line, **anode toward the gimbal, cathode toward the XIAO's `5V` pin** —
   this is required, not optional, since it stops the XIAO from ever backfeeding into the
   gimbal's rail. Charging then runs automatically whenever connected, at whatever rate the
   onboard IC and the gimbal's rail can sustain between them — the "very slowly" behavior
   the project asked for isn't something either side lets you dial in explicitly (no
   documented way to reprogram the charge IC's current), so in practice it'll be gentler
   *because* the gimbal's accessory rail is a low-power line, not because anything here is
   throttling it on purpose.

   ⚠️ **Measure before you trust it**: nobody publishes a current budget for the gimbal's
   `VCC_5V` accessory pin. Before wiring this permanently, put a current meter or bench
   supply between the diode and the gimbal port and confirm it can sustain the charge IC's
   draw without the rail sagging — a starved 5V line risks disrupting the gimbal's own CAN
   transceiver, which the entire rest of this project depends on staying stable.

2. **Wakes and (eventually) sleeps the board.** Tap a second connection off the same
   `VCC_5V` wire (before or after the diode, either works) into a resistor divider that
   brings 5V down under the C3's GPIO limit — e.g. 10kΩ from `VCC_5V` to the tap, 15kΩ from
   the tap to `GND`, giving ≈3.0V when connected and a clean 0V (via the same 15kΩ acting as
   a pulldown) when not. Feed that tap into **D3 (GPIO5)**. A small 100nF cap from the tap to
   `GND` is a cheap debounce against connector chatter.

   D3 was picked because the ESP32-C3 can only wake from deep sleep on **GPIO0–5**, and of
   the XIAO's exposed pins that's only D0–D3. The CAN hat occupies D6–D10, so D3 is still
   free electrically, but it is sandwiched under the hat. Sleep stays **disabled**
   (`GIMBAL_SLEEP_ENABLED=0`) until that divider is actually wired. A floating D3 would
   otherwise deep-sleep the board 60s after boot.

   In firmware (`main.cpp`), `loop()` watches `GIMBAL_PRESENT_PIN` and calls
   `enterDeepSleepUntilGimbalPresent()` after `OFF_TIMEOUT_MS` (default 60s) with no high
   reading — that call tears down WiFi/WebSockets and puts the C3 into deep sleep
   (single-digit µA draw), woken only by that same pin going high again, i.e. the CAN cable
   being reconnected to a powered gimbal. **Charging is unaffected by any of this** — the
   charge IC runs off `VCC_5V`/`BAT` independent of whether the MCU is awake, asleep, or
   mid-boot, so the battery still tops up while the board naps between sessions.

   This gets you "plug in → powers on, unplug → powers off after a timeout, charges the
   whole time it's connected" without any transistor/MOSFET latch circuit — deep sleep's
   idle draw is low enough that it isn't worth the added complexity of a true hardware power
   cutoff for a device that's mostly either in active use or sitting in a bag between shoots.
   If you later find µA-scale drain matters (e.g. weeks of unused storage), a hardware
   latch — like the ones documented in [this writeup](https://randomnerdtutorials.com/power-saving-latching-circuit/) —
   is the next step up, at the cost of a P-MOSFET and a couple more passives.

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

### WebSocket (`ws://<device-ip>/ws`) — use this for continuous/real-time control

Browser → device, JSON text frames:

| `cmd`         | Fields                          | Notes                                                   |
|---------------|----------------------------------|----------------------------------------------------------|
| `speed`       | `yaw`, `roll`, `pitch` (°/s)     | Send at ~20 Hz while the joystick is deflected; the UI already does this. |
| `recenter`    | —                                | One-shot recenter.                                        |
| `selfie`      | —                                | One-shot selfie pose.                                      |
| `activetrack` | —                                | Toggles ActiveTrack.                                        |
| `focus`       | `position` (0–4096)              | Absolute focus motor position.                              |
| `push`        | `enable` (bool)                  | Enable/disable telemetry push from the gimbal.               |

Device → browser: `{"type":"telemetry","yaw":..,"roll":..,"pitch":..,"rssi":..}` at ~10 Hz
whenever the gimbal is pushing angle data.

### REST (`/api/*`) — one-shot commands, no persistent connection needed

- `GET  /api/status` → `{wifi_rssi, ip, ws_clients}`
- `POST /api/recenter`, `/api/selfie`, `/api/activetrack`
- `POST /api/focus?position=2048`
- `POST /api/push?enable=1`
- `POST /api/speed?yaw=0&roll=0&pitch=0` (one-shot; for continuous control use the WebSocket)

## Safety: the speed watchdog

Because control now travels over WiFi instead of a wired connection, a dropped connection
must not leave the gimbal spinning at whatever speed it last received. `main.cpp` tracks the
time of the last `speed` command and **zeroes the gimbal's speed if none arrives within
250 ms** while a non-zero speed is active (see `sendSpeedIfDue()`), and also zeroes speed
immediately when the last WebSocket client disconnects. If you build a different client than
the bundled web UI, make sure it either sends `speed` updates continuously while deflected or
explicitly sends a zero `speed` command on release — don't rely on a single "fire and forget"
command for anything continuous.

## Known gaps / left to reverse-engineer

Everything about the packet framing and the commands used here (`0x00`, `0x01`, `0x02`,
`0x07`, `0x08`, `0x0E`, `0x11`, `0x12`) is carried over as-is from the validated Python CLI.
Not yet ported/decoded (see `docs/DJI_R_SDK_Protocol.md` §Status for the full list):

- `0x04` (limit angles), `0x06` (stiffness), `0x0B` (user params) — `dji_can_protocol.h`
  intentionally doesn't include builders for these yet; add them the same way as the CLI's
  versions if you need them from the web UI.
- `0x10` (auto-calibration status push) is received but not parsed/surfaced.
- Only one `data_type` value (`0x00`, attitude) has ever been observed on the `0x08` push;
  other values are unhandled.

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

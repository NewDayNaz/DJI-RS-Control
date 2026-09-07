# DJI RS Control

Drive a DJI RS gimbal (RS 2 through RS 5) over accessory CAN using the DJI R SDK (`SOF 0xAA`, 1 Mbps). This repo is a Wi‑Fi joystick on a [Seeed XIAO ESP32C3](https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/) + [XIAO CAN Bus Expansion Board](https://wiki.seeedstudio.com/xiao-can-bus-expansion/), plus the protocol notes and Python tools used to reverse it.

If you only want to copy this build: parts, wiring, flash. If you want your own host: skip to [Speak the protocol](#speak-the-protocol) and [`docs/DJI_R_SDK_Protocol.md`](docs/DJI_R_SDK_Protocol.md).

```text
Phone  --Wi‑Fi-->  XIAO ESP32C3  --SPI-->  MCP2515 hat  --CAN-->  gimbal
                   5 V / GND / CANH / CANL from the Focus Wheel 4-pin
```

This is **not** S-BUS and **not** the Ronin app’s BLE (`SOF 0x55`). Wrong switch or wrong SOF and you get silence.

---

## What you need

- Seeed XIAO ESP32C3
- Official XIAO CAN Bus Expansion Board — **16 MHz** crystal (8 MHz cannot do 1 Mbps)
- DJI RS Focus Wheel (4-pin CAN tap and 5 V)
- Four wires: `5V`, `GND`, `CANH`, `CANL`

The hat is MCP2515 SPI + SN65HVD230. It does not use the C3’s TWAI pins.

---

## Wire it

1. Seat the XIAO on the hat.
2. Short pad **P1** on the back of the hat (120 Ω). The gimbal is not terminated. Do not add a second resistor at the plug.
3. On the gimbal, set the accessory slide switch to **`CAN`**, not `S-BUS`.
4. From the Focus Wheel 4-pin (same as protocol [§4.1](docs/DJI_R_SDK_Protocol.md#41-physical-connector)):

| Pin | Signal | To |
|----:|--------|----|
| 1 | `VCC_5V` | XIAO **`5V`** |
| 2 | `GND` | Hat / XIAO GND |
| 3 | `CANH` | Hat CANH |
| 4 | `CANL` | Hat CANL |

**Unplug USB** while that 5 V rail is connected, or the XIAO backfeeds 5 V onto pin 1. If the Focus Wheel LED goes red when Wi‑Fi starts, the rail cannot feed the radio — power the XIAO separately and leave pin 1 disconnected.

Hat SPI (only needed if you are not stacking the official board):

| XIAO | Hat |
|------|-----|
| D6 (GPIO21) | MCP2515 INT |
| D7 (GPIO20) | MCP2515 CS |
| D8 | SCK |
| D9 | MISO |
| D10 | MOSI |

D6/D7 are also UART0. This firmware uses USB CDC for serial so those pins stay on the MCP2515.

---

## Flash and use

From `firmware/esp32-gimbal-bridge`, in **PowerShell or cmd** (not Git Bash):

```text
pio run -e seeed_xiao_esp32c3 -t upload
pio run -e seeed_xiao_esp32c3 -t uploadfs
```

`uploadfs` is the web UI in `data/`. Re-run it when that folder changes. If the board is not found: hold **BOOT**, tap **RESET**, release **BOOT** once upload starts.

First boot opens AP **`DJI-Gimbal-Setup`**. Join it (or open `192.168.4.1`), pick your Wi‑Fi, then browse to `http://<device-ip>/`.

Held speed is re-sent at 20 Hz and **zeroed after 250 ms** of silence (and when the last WebSocket client drops). Any custom UI should stream `speed` while the stick is deflected, or send `stop`.

Command list and diagnostics: [`firmware/esp32-gimbal-bridge/README.md`](firmware/esp32-gimbal-bridge/README.md).

---

## Speak the protocol

You do not need this ESP32. Any 1 Mbps CAN adapter works. The Python CLI is the reference encoder.

| | |
|---|---|
| Bitrate | 1 Mbps, 11-bit |
| Host → gimbal | **`0x223`** |
| Gimbal → host | **`0x222`** |
| Packet | `SOF 0xAA`, then header, CRC-16, `CMD_SET`/`CMD_ID`/payload, CRC-32 |
| Gimbal cmds | CmdSet **`0x0E`** |
| Camera record | CmdSet **`0x0D`** (camera-control cable) |

Build one SDK packet, then send it as successive **8-byte** frames on `0x223`. Reassemble `0x222` the same way (start at `0xAA`, read length, check both CRCs). Host requests use cmd-type `0x03`; replies/pushes have bit `0x20` in byte 3. Angles are int16 in **0.1°**. Joystick speed uses `0x0E/0x01` with takeover `0x80`; go-to position is `0x0E/0x00`.

The same wire also carries Focus Wheel frames (~400 Hz on `0x530` / `0x531` / `0x426`). Filter RX to **`0x222`** or those will overwrite SDK fragments on a two-buffer controller like the MCP2515.

Payloads, CRCs, and every command: [`docs/DJI_R_SDK_Protocol.md`](docs/DJI_R_SDK_Protocol.md).

USB adapter instead of the XIAO:

```text
pip install -r requirements.txt
python dji_gimbal_cli.py COM6
python dji_gimbal_web.py
```

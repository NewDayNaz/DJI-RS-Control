# Bitfocus Companion Integration Guide

This guide shows how to control the DJI gimbal from Bitfocus Companion using the **built-in HTTP API** instead of PTZ protocols (VISCA/Pelco). This approach is **more reliable, simpler, and more flexible** than protocol translation.

## Why Use HTTP Instead of VISCA?

| Feature | VISCA | HTTP API |
|---------|-------|----------|
| **Reliability** | ❌ UDP = packet loss → runaway movement | ✅ TCP = reliable delivery |
| **Protocol** | Binary (complex) | JSON (human-readable) |
| **Feedback** | None | Full telemetry |
| **Commands** | Limited PTZ set | All gimbal features |
| **Debugging** | Binary hex dumps | `curl` / browser |
| **Extensions** | Impossible | Easy to add |

**Bottom line:** VISCA was designed for 1990s hardware joysticks. Your API is designed for modern software control.

---

## Quick Start

### 1. Find Your Gimbal IP

Check your router's DHCP leases or serial console (115200 baud) for output like:
```
[wifi] connected, IP = 192.168.1.42
```

### 2. Test HTTP Endpoints

```bash
# Get current state
curl http://192.168.1.42/api/state

# Pan left at 30°/s
curl -X POST http://192.168.1.42/api/speed \
  -H "Content-Type: application/json" \
  -d '{"yaw": -30, "pitch": 0, "roll": 0, "hold": true}'

# Stop
curl -X POST http://192.168.1.42/api/command/stop

# Recenter
curl -X POST http://192.168.1.42/api/command/recenter
```

---

## Companion Configuration

### Method 1: Generic HTTP Request Module (Recommended)

This is the simplest approach for buttons.

#### Setup in Companion:

1. **Add Connection:**
   - Module: `Generic HTTP Request`
   - Label: `DJI Gimbal`

2. **Create Buttons:**

#### Pan Left Button
- **Action:** HTTP Request
- **Method:** `POST`
- **URL:** `http://192.168.1.42/api/speed`
- **Headers:** `Content-Type: application/json`
- **Body:**
  ```json
  {"yaw": -30, "pitch": 0, "roll": 0, "hold": true}
  ```

#### Pan Right Button
- **Action:** HTTP Request
- **Method:** `POST`
- **URL:** `http://192.168.1.42/api/speed`
- **Headers:** `Content-Type: application/json`
- **Body:**
  ```json
  {"yaw": 30, "pitch": 0, "roll": 0, "hold": true}
  ```

#### Tilt Up Button
- **Action:** HTTP Request
- **Method:** `POST`
- **URL:** `http://192.168.1.42/api/speed`
- **Headers:** `Content-Type: application/json`
- **Body:**
  ```json
  {"yaw": 0, "pitch": 20, "roll": 0, "hold": true}
  ```

#### Tilt Down Button
- **Action:** HTTP Request
- **Method:** `POST`
- **URL:** `http://192.168.1.42/api/speed`
- **Headers:** `Content-Type: application/json`
- **Body:**
  ```json
  {"yaw": 0, "pitch": -20, "roll": 0, "hold": true}
  ```

#### Stop Button
- **Action:** HTTP Request
- **Method:** `POST`
- **URL:** `http://192.168.1.42/api/command/stop`

#### Recenter Button
- **Action:** HTTP Request
- **Method:** `POST`
- **URL:** `http://192.168.1.42/api/command/recenter`

#### Sleep Button
- **Action:** HTTP Request
- **Method:** `POST`
- **URL:** `http://192.168.1.42/api/command/sleep`

#### Wake Button
- **Action:** HTTP Request
- **Method:** `POST`
- **URL:** `http://192.168.1.42/api/command/wake`

---

## Available Endpoints

### Movement Commands

#### Speed Control (Continuous Movement)
```http
POST /api/speed
Content-Type: application/json

{
  "yaw": -30.0,      // deg/s, negative = left, positive = right
  "pitch": 20.0,     // deg/s, negative = down, positive = up
  "roll": 0.0,       // usually 0
  "hold": true       // true = continuous, false = single shot
}
```

**Speeds:** Configurable max rate (default 30°/s). Adjust in UI under "PTZ Settings" or:
```http
POST /api/ptz/profile
{"max_rate": 60.0}
```

**Auto-stop:** Speed is auto-zeroed after 250ms if no new command (safety deadman).

#### Position Control (Go-to)
```http
POST /api/position
Content-Type: application/json

{
  "yaw": 45.0,       // target angle in degrees
  "pitch": -10.0,
  "roll": 0.0,
  "time_s": 2.0      // movement duration (0.0-25.5s)
}
```

### Zoom/Focus Control

#### Set Zoom Position
```http
POST /api/zoom
Content-Type: application/json

{"position": 2000}   // motor counts: 1-4095 (1=tele, 4095=wide)
```

Or by millimeters (if lens table calibrated):
```json
{"mm": 24.0}
```

#### Hold Current Zoom
```http
POST /api/zoom/hold
```
Stops zoom ramp and holds current position.

#### Zoom Profile (Speed/Acceleration)
```http
POST /api/zoom/profile
Content-Type: application/json

{
  "vmax": 900.0,     // max speed (counts/s)
  "accel": 4500.0,   // acceleration (counts/s²)
  "vcut": 120.0,     // cutoff speed (optional)
  "decel": 4500.0    // deceleration (optional, defaults to accel)
}
```

### Named Commands

```http
POST /api/command/<name>
```

| Command | Description |
|---------|-------------|
| `stop` | Stop all movement |
| `recenter` | Return to center position |
| `selfie` | Flip to selfie mode |
| `sleep` | Put gimbal to sleep |
| `wake` | Wake gimbal |
| `calibrate` | Run gimbal calibration |
| `motor-calib` | Calibrate focus motor |
| `activetrack` | Toggle ActiveTrack |
| `rec-start` | Start camera recording |
| `rec-stop` | Stop camera recording |

### Telemetry

#### Get State
```http
GET /api/state
```

Returns:
```json
{
  "connected": true,
  "adapter": "mcp2515",
  "interface": "can",
  "yaw": 45.2,
  "roll": 0.1,
  "pitch": -10.3,
  "zoom": 2000,
  "zoom_target": 2000,
  "zoom_moving": false,
  "zoom_mm": 35.0,
  "last_rx_age_s": 0.05,
  "tx_ok": 1234,
  "rx_ok": 5678
}
```

#### Get Full Status (Diagnostics)
```http
GET /api/status
```

Includes WiFi, CAN bus details, PTZ protocol counters, etc.

---

## WebSocket API (Alternative)

For applications that need continuous telemetry or want bidirectional communication.

### Connect
```javascript
ws://192.168.1.42/ws
```

### Send Commands (same as HTTP body)
```json
{"cmd": "speed", "yaw": -30, "pitch": 0, "roll": 0, "hold": true}
{"cmd": "position", "yaw": 45, "pitch": -10, "roll": 0, "time_s": 2.0}
{"cmd": "zoom", "position": 2000}
{"cmd": "recenter"}
{"cmd": "stop"}
```

### Receive Telemetry (20 Hz stream)
The gimbal broadcasts state at 20 Hz to all connected clients:
```json
{
  "connected": true,
  "yaw": 45.2,
  "pitch": -10.3,
  "zoom": 2000,
  "zoom_moving": false,
  ...
}
```

**Companion WebSocket Module:** If Companion has a Generic WebSocket module, you can use this for feedback (e.g., show current zoom position on button).

---

## Advanced: Companion Custom Module

For a polished integration, you could create a custom Companion module:

### Module Structure
```
companion-module-dji-rs-gimbal/
├── index.js           // Main module
├── actions.js         // Button actions
├── feedbacks.js       // Button feedbacks (colors, text)
├── variables.js       // Dynamic variables (current position, etc.)
└── package.json
```

### Example Features:
- **Actions:** Pre-configured speed/position/zoom buttons
- **Feedbacks:** Button color changes when gimbal is moving
- **Variables:** `$(dji-gimbal:yaw)`, `$(dji-gimbal:zoom)`
- **Presets:** Common button layouts

**Template:** Use `companion-module-ptzoptics-visca` as reference, but replace VISCA with HTTP/WebSocket.

---

## Example Stream Deck Layout

```
┌─────────┬─────────┬─────────┬─────────┐
│   UP    │ RECENTER│  SLEEP  │  WAKE   │
│  ↑ 20   │   ⊙     │   😴    │   👁️   │
├─────────┼─────────┼─────────┼─────────┤
│  LEFT   │  STOP   │  RIGHT  │  ZOOM   │
│  ← 30   │   ■     │  30 →   │  WIDE   │
├─────────┼─────────┼─────────┼─────────┤
│  DOWN   │ SELFIE  │ RECORD  │  ZOOM   │
│  ↓ 20   │   👤    │   ⏺️    │  TELE   │
└─────────┴─────────┴─────────┴─────────┘
```

---

## Testing & Debugging

### Check if Gimbal is Reachable
```bash
curl http://192.168.1.42/api/state
```

Should return JSON with gimbal state. If timeout:
- Check gimbal IP address
- Check firewall rules
- Verify WiFi connection

### Monitor Serial Console
Connect to USB (115200 baud):
```bash
pio device monitor
```

Watch for:
```
[ptz] Pelco 4000  AW UDP 49152  (VISCA disabled at compile time)
[main] ready
```

### Enable Debug Logging
Set in `platformio.ini`:
```ini
-DCORE_DEBUG_LEVEL=4
```

---

## Alternative: Using VISCA Module with Repeat-While-Held

If you prefer using the native Sony VISCA Companion module instead of HTTP, you can work around the watchdog limitation using Companion's Duration Group feature.

### How It Works

The VISCA watchdog (1000ms) stops movement if no new command arrives. To keep the watchdog satisfied, configure Companion to **continuously re-send** the movement command while the button is held:

#### Configuration Steps

1. **Add Sony VISCA Connection:**
   - Module: `Sony VISCA`
   - Protocol: `TCP` (recommended) or `UDP`
   - IP: `192.168.1.42`
   - Port: `5678` (TCP) or `1259` (UDP raw) or `52381` (UDP with IP header)

2. **Configure Pan Left Button:**

   **Press Actions:**
   - Action: Sony VISCA → `Pan Left` (speed: 0x14)

   **Add Duration Group:**
   - Click "Add duration group"
   - Set delay: `100` ms
   - Enable ✅ **Execute while held**
   - Inside the group, add:
     - Action: `internal: Button: Trigger Press` (this button)
     - Option: ✅ **Force press if already pressed**

   **Release Actions:**
   - Action: Sony VISCA → `Pan/Tilt Stop`
   - Action: `internal: Wait` → `50` ms
   - Action: Sony VISCA → `Pan/Tilt Stop`
   - Action: `internal: Wait` → `50` ms
   - Action: Sony VISCA → `Pan/Tilt Stop`

3. **Result:**
   - Button press → Sends "Pan Left" 
   - After 100ms (while still held) → Re-triggers the button → Sends "Pan Left" again
   - This loops every 100ms while button is held (10 Hz)
   - Button release → Sends "Pan/Tilt Stop" **three times with 50ms gaps**

**Why send stop multiple times?** UDP packets can be lost. Sending the stop command 3 times dramatically increases reliability. If one packet is lost, the others will still get through. The 1000ms watchdog provides backup safety if all stop packets are lost.

### Modern Approach (Companion 5.x)

Alternatively, use the newer logic actions:

1. **Press Actions:**
   - `internal: Logic While`
     - Condition: `internal:buttonPushed`
     - Inside loop:
       - Sony VISCA → `Pan Left`
       - `internal: Wait` → `100` ms

2. **Release Actions:**
   - Sony VISCA → `Pan/Tilt Stop`
   - `internal: Wait` → `50` ms
   - Sony VISCA → `Pan/Tilt Stop`
   - `internal: Wait` → `50` ms
   - Sony VISCA → `Pan/Tilt Stop`

**Note:** Sending stop commands multiple times ensures reliable stopping even with UDP packet loss.

### Pros & Cons of VISCA vs HTTP

| Feature | VISCA Module | HTTP API |
|---------|--------------|----------|
| **Setup** | Native PTZ module | Generic HTTP module |
| **Button Config** | Duration Group loop (workaround) | Single action per button |
| **Reliability** | UDP (most VISCA) = packet loss | TCP = guaranteed delivery |
| **Latency** | ~5-15ms | ~10-50ms |
| **Features** | PTZ subset only | All gimbal commands |
| **Watchdog** | 1000ms (requires 10 Hz loop) | 250ms deadman (no loop needed) |
| **Stop Safety** | Send 3x to handle UDP loss | TCP guarantees delivery |
| **Debugging** | Binary protocol, hex dumps | JSON, test with curl |

**Recommendation:** Use HTTP API unless you have a specific need for VISCA compatibility with existing hardware workflows.

---

## Comparison: HTTP vs PTZ Protocols

### VISCA Example (Complex, Unreliable)
```
Hardware Controller sends:
  UDP packet: 81 01 06 01 14 14 03 01 FF (pan right)
  [PACKET LOST] 
  UDP packet: 81 01 06 01 14 14 03 03 FF (stop)
  
Result: Gimbal keeps panning because stop packet was lost! ❌
```

### HTTP API Example (Simple, Reliable)
```bash
# Pan right
curl -X POST http://gimbal/api/speed -d '{"yaw": 30, "hold": true}'

# Stop (TCP ensures delivery)
curl -X POST http://gimbal/api/command/stop

# Even if app crashes, gimbal stops after 250ms deadman timeout ✅
```

---

## Performance Notes

### Latency
- **HTTP:** ~10-50ms per request (depends on network)
- **WebSocket:** ~5-20ms (persistent connection)
- **VISCA/Pelco:** ~5-15ms (but unreliable UDP)

For interactive control (Stream Deck buttons), HTTP latency is imperceptible.

### Deadman Safety
The firmware auto-stops movement after **250ms** of no commands. This means:
- Even if Companion crashes, gimbal stops
- No need for explicit stop on every button release
- But you CAN send explicit stops for immediate response

### Rate Limiting
No rate limiting implemented. Send commands as fast as needed. The gimbal processes them at 20 Hz internally.

---

## Troubleshooting

### Buttons Don't Work
1. Test with `curl` first to isolate Companion vs API
2. Check Companion logs for HTTP errors
3. Verify JSON syntax (use JSON validator)
4. Check gimbal serial log for received commands

### Gimbal Doesn't Stop
- Verify 250ms deadman timeout is working
- Check if `hold: false` was accidentally used
- Send explicit `/api/command/stop`

### Telemetry Not Updating
- `/api/state` is polled, not pushed (use WebSocket for push)
- Check CAN bus connection (Focus Wheel cable)
- Verify parameter push is enabled (Telemetry section in UI)

---

## Next Steps

1. **Test with curl:** Verify all endpoints work
2. **Create Companion buttons:** Start with basic pan/tilt/stop
3. **Add feedbacks (optional):** Poll `/api/state` to show current position
4. **Consider custom module:** If you need polish and reusability

**Questions?** Check the main firmware README or open an issue.

---

## Benefits Summary

✅ **More Reliable:** TCP vs UDP, no packet loss issues  
✅ **Simpler:** JSON vs binary protocols  
✅ **Full-Featured:** All gimbal commands, not just PTZ subset  
✅ **Debuggable:** Use browser/curl to test  
✅ **Telemetry:** Get current position, zoom, etc.  
✅ **Extensible:** Add custom endpoints easily  
✅ **Safe:** 250ms deadman auto-stop  

**Recommendation:** Use the HTTP API with Companion's Generic HTTP Request module. It's the best of both worlds: reliability of your custom API + flexibility of Companion.

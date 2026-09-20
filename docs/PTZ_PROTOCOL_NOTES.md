# PTZ Protocol Implementation Notes

## Default Configuration (as of 2026-09-20)

**Enabled by default:**
- `ENABLE_VISCA=1` (enabled with 1000ms watchdog)
- HTTP REST API at `/api/*`
- WebSocket at `/ws`
- Panasonic AW (UDP 49152) - ASCII protocol, lightweight
- HTTP CGI (port 80) - PTZOptics/Sony/Panasonic CGI endpoints

**Disabled by default:**
- `ENABLE_PELCO=0` (disabled)

## Rationale

### Why Disable Binary PTZ Protocols?

**1. Modern Software Control (Bitfocus Companion, etc.)**
- HTTP/WebSocket API is more reliable (TCP vs UDP)
- JSON is human-readable and debuggable
- Full feature set, not limited to PTZ subset
- Bidirectional telemetry

**2. Resource Usage**
- VISCA: 3 UDP ports + 1 TCP server + watchdog state
- Pelco: 2 UDP/TCP servers + frame parsing
- Combined: ~8KB RAM for buffers and state machines
- Saves resources when not needed

**3. Binary Protocols are 1990s Hardware-Focused**
- Designed for RS-485 serial joysticks
- Fixed command sets
- No telemetry/feedback
- Awkward to debug (hex dumps)

## When to Enable

### Enable VISCA (`ENABLE_VISCA=1`)
**Use case:** Physical hardware PTZ controllers that only speak VISCA
- Sony RM-IP series
- PTZOptics controllers (some models)
- Some broadcast switchers with PTZ control

**Note:** Includes 1000ms watchdog to prevent runaway movement from dropped UDP packets.

### Enable Pelco-D/P (`ENABLE_PELCO=1`)
**Use case:** Physical hardware PTZ controllers that prefer Pelco
- Pelco keyboards/joysticks
- Many security/broadcast control panels
- PTZOptics controllers (most models support both)

**Advantage:** Stateless protocol — each frame contains complete state, self-correcting on packet loss.

### Keep Disabled (Default)
**Use case:** Software control via Companion, custom apps, or web UI
- Use HTTP REST API: `POST /api/speed {"yaw": 30, ...}`
- Use WebSocket: `{"cmd": "speed", "yaw": 30, ...}`
- See `docs/COMPANION_INTEGRATION.md` for full guide

## Protocol Comparison

| Feature | HTTP API | WebSocket | VISCA | Pelco-D/P | Panasonic AW |
|---------|----------|-----------|-------|-----------|--------------|
| **Transport** | TCP (reliable) | TCP | UDP (unreliable) | UDP+TCP | UDP |
| **Format** | JSON | JSON | Binary | Binary | ASCII |
| **Telemetry** | Pull | Push (20Hz) | None | None | Limited |
| **Commands** | All | All | PTZ subset | PTZ subset | PTZ subset |
| **Debugging** | curl/browser | Browser console | Wireshark | Wireshark | Wireshark |
| **Packet Loss** | Retransmits | Retransmits | **Runaway!** | Self-corrects | Self-corrects |
| **Compile Flag** | N/A (always on) | N/A (always on) | `ENABLE_VISCA` | `ENABLE_PELCO` | N/A (always on) |

## Serial Console Output

### Default (Both Disabled)
```
[ptz] AW UDP 49152  (PTZ protocols disabled - use HTTP/WebSocket API)
```

### VISCA Enabled
```
[ptz] VISCA UDP 52381/1259 TCP 5678 (with 1s watchdog)  AW UDP 49152
```

### Pelco Enabled
```
[ptz] Pelco 4000  AW UDP 49152
```

### Both Enabled
```
[ptz] VISCA UDP 52381/1259 TCP 5678 (with 1s watchdog)  Pelco 4000  AW UDP 49152
```

## Status API

The `/api/status` endpoint reports which protocols are enabled:

```json
{
  "ptz": {
    "visca_enabled": false,
    "pelco_enabled": false,
    "panasonic": 0,
    "cgi": 0,
    "ports": {
      "panasonic_udp": 49152
    }
  }
}
```

When enabled:
```json
{
  "ptz": {
    "visca_enabled": true,
    "visca_udp": 42,
    "visca_tcp": 7,
    "pelco_enabled": true,
    "pelco": 15,
    "panasonic": 0,
    "cgi": 128,
    "ports": {
      "visca_ip_udp": 52381,
      "visca_raw_udp": 1259,
      "visca_raw_tcp": 5678,
      "pelco": 4000,
      "panasonic_udp": 49152
    }
  }
}
```

## Memory/Resource Impact

Approximate memory savings when protocols are disabled:

| Component | VISCA | Pelco | Combined Savings |
|-----------|-------|-------|------------------|
| UDP buffers (3×1KB) | 3 KB | - | 3 KB |
| TCP buffers (2×64B) | 128 B | 128 B | 256 B |
| State machines | ~512 B | ~256 B | ~768 B |
| Code size (estimate) | ~8 KB | ~4 KB | ~12 KB |
| **Total RAM saved** | **~3.6 KB** | **~0.4 KB** | **~4 KB** |

*On ESP32-C3 with 400KB RAM, 4KB is ~1% savings — not critical but meaningful.*

## Implementation Details

### VISCA Watchdog
When `ENABLE_VISCA=1`, tracks last movement command:
```cpp
static uint32_t g_viscaLastMoveMs = 0;
static bool g_viscaMoving = false;

void checkViscaTimeout() {
    if (g_viscaMoving && millis() - g_viscaLastMoveMs > 500) {
        g_viscaMoving = false;
        if (g_sink.stop) g_sink.stop();
        Serial.println("[visca] watchdog timeout - auto-stopped movement");
    }
}
```

Called every 5ms from `ptzTask()` loop.

### Conditional Compilation
All VISCA/Pelco code wrapped in:
```cpp
#if ENABLE_VISCA
    // VISCA UDP/TCP servers, parsing, watchdog
#endif

#if ENABLE_PELCO
    // Pelco UDP/TCP servers, frame parsing
#endif
```

## Migration Guide

### If You Were Using VISCA/Pelco

**Before (binary PTZ):**
```
Hardware Controller → VISCA/Pelco → Gimbal
```

**After (disable and migrate to HTTP):**
```
Companion → HTTP API → Gimbal
```

**Companion button configuration:**
```http
POST http://gimbal-ip/api/speed
Content-Type: application/json

{"yaw": 30, "pitch": 0, "roll": 0, "hold": true}
```

See `docs/COMPANION_INTEGRATION.md` for complete migration guide.

### If You Need Physical PTZ Hardware

**Enable in platformio.ini:**
```ini
-DENABLE_VISCA=1    # For Sony RM-IP, etc.
-DENABLE_PELCO=1    # For Pelco keyboards, etc.
```

Then rebuild:
```bash
pio run -t upload
```

## Recommendations

| Use Case | Recommendation |
|----------|----------------|
| **Bitfocus Companion** | HTTP API (default) |
| **Custom software** | HTTP or WebSocket API (default) |
| **Web browser control** | WebSocket (default) |
| **Sony RM-IP controller** | Enable VISCA |
| **Pelco keyboard** | Enable Pelco |
| **PTZOptics SuperJoy** | Enable Pelco (preferred) or VISCA |
| **Panasonic AW-RP** | Panasonic AW (always enabled) |
| **Mixed environment** | Enable what hardware requires |
| **No hardware controllers** | Keep disabled (default) |

## Future Considerations

Potential improvements:
1. **Runtime enable/disable** via web UI (currently compile-time only)
2. **Protocol auto-detection** based on incoming traffic
3. **Per-protocol packet counters** in UI
4. **VISCA packet retry** for UDP reliability
5. **Custom module for Companion** (dedicated integration)

## Questions?

See main README or open an issue on GitHub.

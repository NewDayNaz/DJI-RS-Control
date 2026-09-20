# VISCA Protocol Improvements - Summary

## Changes Implemented

### 1. **Compile-Time Flag for VISCA Support**
- Added `ENABLE_VISCA` flag in `platformio.ini` (default: **0** = disabled)
- VISCA is now **opt-in** to prevent runaway movement issues by default

### 2. **500ms Watchdog Timeout**
- Tracks last VISCA movement command timestamp
- Auto-stops gimbal if no command received within 500ms
- Prevents indefinite movement from dropped UDP stop packets
- Logged to serial when triggered: `[visca] watchdog timeout - auto-stopped movement`

### 3. **Protocol Recommendation: Pelco-D/P**
Pelco-D/P is now the recommended protocol because:
- **Stateless**: Each frame contains complete state (all button/joystick positions)
- **Self-correcting**: Missed packets don't cause persistent errors
- **Widely supported**: Professional PTZ hardware (Sony RM-IP, PTZOptics SuperJoy, Panasonic AW-RP, Pelco keyboards)

**VISCA Problem:**
- Stateful start/stop commands
- Lost UDP stop packet → gimbal keeps moving indefinitely
- Inherent protocol design issue, not implementation bug

### 4. **Bitfocus Companion Compatibility**
**Key Finding:** Bitfocus Companion does NOT have native Pelco-D/P modules.

**For Companion Users:**
- Primary modules are VISCA-based (sony-visca, PTZOptics)
- To use Pelco: Generic TCP/UDP with raw hex, or TCP-Serial bridge
- **Recommendation**: Enable VISCA with `ENABLE_VISCA=1` — watchdog provides safety

### 5. **Status API Update**
`/api/status` now includes:
```json
{
  "ptz": {
    "visca_enabled": true/false,
    "visca_udp": <count>,     // only if enabled
    "visca_tcp": <count>,     // only if enabled
    "pelco": <count>,
    "panasonic": <count>
  }
}
```

## Code Changes

### Files Modified:
1. **`platformio.ini`**
   - Added `-DENABLE_VISCA=0` build flag with documentation

2. **`src/ptz_bridge.h`**
   - Updated header comments to note VISCA is compile-time optional
   - Marked Pelco-D/P as recommended

3. **`src/ptz_bridge.cpp`**
   - Wrapped all VISCA code in `#if ENABLE_VISCA` blocks
   - Added watchdog state tracking (`g_viscaLastMoveMs`, `g_viscaMoving`)
   - Implemented `checkViscaTimeout()` function
   - Updated movement commands to track timestamp
   - Modified `ptzTask()` to call watchdog check
   - Updated `ptzBridgeFillStatus()` to report enabled state
   - Modified `ptzBridgeBegin()` to conditionally initialize VISCA

4. **`firmware/esp32-gimbal-bridge/README.md`**
   - Added "⚠️ PTZ Protocol Selection" section
   - Documented VISCA issues and watchdog
   - Added Bitfocus Companion compatibility notes
   - Listed always-available protocols vs optional

## Usage

### To Enable VISCA:
Edit `firmware/esp32-gimbal-bridge/platformio.ini`:
```ini
-DENABLE_VISCA=1
```

Then rebuild and flash:
```bash
pio run -t upload
```

### Protocol Ports:
| Protocol | Port | Status |
|----------|------|--------|
| Pelco-D/P | UDP+TCP 4000 | ✅ Always enabled (recommended) |
| Panasonic AW | UDP 49152 | ✅ Always enabled |
| HTTP CGI | 80 | ✅ Always enabled |
| VISCA over IP | UDP 52381 | ⚠️ Opt-in (watchdog included) |
| VISCA raw | UDP 1259 | ⚠️ Opt-in (watchdog included) |
| VISCA TCP | TCP 5678 | ⚠️ Opt-in (watchdog included) |

## Testing Notes

### When VISCA Disabled (default):
- VISCA ports not bound (saves resources)
- Serial log: `[ptz] Pelco 4000  AW UDP 49152  (VISCA disabled at compile time)`
- Status API returns `"visca_enabled": false`

### When VISCA Enabled:
- All VISCA ports active with watchdog protection
- Serial log: `[ptz] VISCA UDP 52381/1259 TCP 5678 (ENABLED with 500ms watchdog)  Pelco 4000  AW UDP 49152`
- Status API includes VISCA counters and `"visca_enabled": true`

## Recommendations by Use Case

| Use Case | Recommended Protocol |
|----------|---------------------|
| Professional PTZ controllers | **Pelco-D/P** |
| Bitfocus Companion | **VISCA** (enable flag) or Pelco workaround |
| Panasonic AW-series controllers | **Panasonic AW** |
| Web browser/custom apps | **WebSocket / REST API** |
| Unreliable networks | **Pelco-D/P** or **Panasonic AW** |

## Technical Details

### Watchdog Implementation
```cpp
static constexpr uint32_t kViscaTimeoutMs = 500;
static uint32_t g_viscaLastMoveMs = 0;
static bool g_viscaMoving = false;

void checkViscaTimeout() {
    if (g_viscaMoving && millis() - g_viscaLastMoveMs > kViscaTimeoutMs) {
        g_viscaMoving = false;
        if (g_sink.stop) g_sink.stop();
        Serial.println("[visca] watchdog timeout - auto-stopped movement");
    }
}
```

Called every 5ms from `ptzTask()` loop.

### Pelco-D/P Frame Structure
Complete state snapshot (7 bytes for Pelco-D, 8 for Pelco-P):
- Address, Command bytes (all button states), Pan/Tilt speed, Checksum
- Each frame is self-contained — no "start" then "stop" pattern
- Missed frame → next frame corrects it

## Future Enhancements

Potential improvements for consideration:
1. **Configurable watchdog timeout** via web UI
2. **Redundant stop commands** for VISCA (send 2-3 stop frames)
3. **Packet loss statistics** per protocol
4. **Protocol auto-detection** based on incoming traffic
5. **VISCA "safe mode"** requiring periodic refresh commands

## Commit

```
commit e22dfa0
Author: Cloud Agent
Date: Sunday Sep 20, 2026

    Add compile-time VISCA flag with watchdog and Pelco-D/P recommendation
    
    - VISCA is now disabled by default (ENABLE_VISCA=0)
    - Added 500ms watchdog timeout to auto-stop runaway movement from dropped packets
    - VISCA uses stateful start/stop commands that fail unsafely on packet loss
    - Pelco-D/P and Panasonic AW are recommended (stateless, self-correcting)
    - Updated documentation with Bitfocus Companion notes (no native Pelco support)
    - Updated status JSON to report visca_enabled flag
```

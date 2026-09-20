# VISCA Watchdog Implementation Notes

## The Problem

VISCA is a **stateful protocol** - controllers send separate "start" and "stop" commands:

```
Press joystick:   Controller → "Start moving up" → Gimbal moves
Release joystick: Controller → "Stop" → Gimbal stops
```

**Issue:** If the UDP stop packet is lost, the gimbal continues moving indefinitely.

## The Watchdog Solution

A watchdog timer auto-stops movement if no VISCA command is received within a timeout period (default: 1000ms).

## The Fundamental Limitation

**The watchdog only works if controllers send continuous commands while held.**

### Scenario A: Continuous Command Controller (WORKS)
```
t=0ms:    Press up    → "Move up" → Watchdog reset
t=100ms:  Hold up     → "Move up" → Watchdog reset
t=200ms:  Hold up     → "Move up" → Watchdog reset
t=300ms:  Hold up     → "Move up" → Watchdog reset
...
t=1000ms: Release     → "Stop"    → Gimbal stops
```
✅ **Watchdog never fires** - continuous commands keep resetting it.

### Scenario B: Single Command Controller (BROKEN)
```
t=0ms:    Press up    → "Move up" → Watchdog reset
t=1-1000ms: Hold up   → (nothing sent)
t=1000ms: Watchdog fires! → Gimbal stops ❌
t=1000ms: Release     → "Stop" → (Already stopped)
```
❌ **Movement stops after 1 second** even though user is still holding joystick!

### Scenario C: Packet Loss (WATCHDOG INTENDED USE)
```
t=0ms:    Press up    → "Move up" → Gimbal moves
t=100ms:  Release     → "Stop" [PACKET LOST]
t=1100ms: Watchdog fires → Gimbal stops ✅
```
✅ **Prevents runaway** - gimbal stops after timeout instead of moving forever.

## How Real Controllers Behave

### Controllers That Send Continuous Commands (Compatible)
- **Sony RM-IP series** - ~10 Hz while held
- **PTZOptics hardware controllers** - ~5-10 Hz while held
- **Marshall CV-series controllers** - ~10 Hz while held
- **Panasonic AW-RP series** (on VISCA mode) - ~5 Hz while held

These work perfectly with the watchdog.

### Controllers That May NOT Send Continuous Commands
- **Software implementations** - Depends on programmer
- **Cheap generic VISCA controllers** - Varies by manufacturer
- **Custom scripts** - Often send only on state change
- **Bitfocus Companion VISCA module** - Likely sends only on button press/release

These may have movements stop prematurely.

## Configuration

### Default Settings (platformio.ini)
```ini
-DENABLE_VISCA=1           # Enable VISCA protocol
-DVISCA_WATCHDOG_MS=1000   # 1 second timeout (default)
```

### Adjusting the Timeout

**Shorter timeout (500ms):**
```ini
-DVISCA_WATCHDOG_MS=500
```
- **Pros:** Faster safety stop on packet loss
- **Cons:** More likely to stop legitimate movements with slow controllers

**Longer timeout (2000ms):**
```ini
-DVISCA_WATCHDOG_MS=2000
```
- **Pros:** More forgiving for slower controllers
- **Cons:** Runaway movement lasts 2 seconds before auto-stop

**Disable watchdog (unsafe):**
```ini
-DVISCA_WATCHDOG_MS=0
```
- **Pros:** No false positives
- **Cons:** Lost stop packets cause indefinite runaway ⚠️

## Testing Your Controller

To check if your VISCA controller sends continuous commands:

1. Enable VISCA debug logging
2. Hold joystick in one direction for 5 seconds
3. Check serial console

**If you see messages every 100-200ms:**
```
[visca] drive from 192.168.1.100
[visca] drive from 192.168.1.100
[visca] drive from 192.168.1.100
```
✅ **Your controller is compatible** - continuous commands

**If you see only one message:**
```
[visca] drive from 192.168.1.100
(nothing for 5 seconds)
[visca] stop from 192.168.1.100
```
❌ **Your controller is NOT compatible** - single commands only

## Recommendations

### For Hardware VISCA Controllers
Keep watchdog **enabled** with default 1000ms:
```ini
-DVISCA_WATCHDOG_MS=1000
```

### For Software/Companion VISCA Control
**Don't use VISCA!** Use the HTTP/WebSocket API instead:
```http
POST http://gimbal-ip/api/speed
{"yaw": 30, "pitch": 0, "hold": true}
```

Benefits:
- ✅ TCP reliability (no packet loss)
- ✅ JSON (human readable)
- ✅ Built-in 250ms deadman timeout (no watchdog needed)
- ✅ All gimbal features, not just PTZ subset

See `docs/COMPANION_INTEGRATION.md` for complete guide.

### For Continuous-Command Software Implementations
If you're writing software that sends VISCA:
```python
# Send movement command continuously at 10 Hz
while joystick.held:
    send_visca_move_command()
    time.sleep(0.1)  # 100ms = 10 Hz
send_visca_stop_command()
```

## Alternative: Pelco-D/P Protocol

Pelco doesn't have this problem because it's **stateless** - every frame contains the complete joystick state:

```ini
-DENABLE_PELCO=1
```

**Each Pelco frame sent at ~10-30 Hz:**
```
Frame: up=1, down=0, left=0, right=0, speed=20 → Gimbal moves up
Frame: up=1, down=0, left=0, right=0, speed=20 → Still moving
Frame: up=0, down=0, left=0, right=0, speed=0  → Gimbal stops
```

Even if frames are lost, the next frame corrects the state. No watchdog needed!

**Trade-off:** Most software (Companion) doesn't have native Pelco support, while VISCA modules exist.

## The Dilemma

There's no perfect solution:

| Option | Pros | Cons |
|--------|------|------|
| **Watchdog enabled (default)** | Prevents runaway from packet loss | May stop legitimate movements on single-command controllers |
| **Watchdog disabled** | No false stops | Allows indefinite runaway on packet loss |
| **HTTP/WebSocket API** | Reliable, no issues | Requires Companion HTTP module instead of VISCA |
| **Pelco-D/P** | Self-correcting, no watchdog needed | No native Companion support |

## Conclusion

**The 1000ms watchdog is a compromise:**
- Protects against dangerous runaway movement
- Works with most hardware VISCA controllers
- May cause false stops with single-command controllers

**If you experience premature stops:**
1. Test if your controller sends continuous commands
2. If not, increase timeout: `-DVISCA_WATCHDOG_MS=2000` or `3000`
3. Or switch to HTTP API for software control (recommended for Companion)
4. Or enable Pelco-D/P if your hardware supports it

**If you never use VISCA controllers:**
```ini
-DENABLE_VISCA=0   # Save 3.6KB RAM, disable all VISCA ports
```

## Questions?

See main README or open an issue on GitHub.

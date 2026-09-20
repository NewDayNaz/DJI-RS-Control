# Stop Command Reliability for VISCA Protocol

## The Problem

VISCA uses UDP for most implementations (TCP is available but less common). UDP packets can be lost in transit, which creates a critical safety issue:

```
User releases joystick → Controller sends "Stop" → [PACKET LOST] → Gimbal keeps moving
```

## The Solution: Multi-Layer Safety

We implement **three layers of safety** to handle stop command failures:

### Layer 1: Watchdog Timer (1000ms)
The firmware automatically stops movement if no VISCA command is received within 1 second.

```
t=0ms:    Movement command received → Watchdog reset
t=100ms:  Movement command received → Watchdog reset
t=200ms:  Movement command received → Watchdog reset
...
t=1500ms: No command received for 1000ms → Watchdog fires → Auto-stop
```

✅ **Catches:** Completely lost stop packets  
❌ **Limitation:** Requires continuous commands (10+ Hz) while held

### Layer 2: Multiple Stop Commands
Controllers (like Companion) send the stop command **3 times** with 50ms delays:

```
Release Actions:
1. Send "Stop" command
2. Wait 50ms
3. Send "Stop" command
4. Wait 50ms
5. Send "Stop" command
```

**Probability math:**
- Packet loss rate: ~1% (typical WiFi)
- Single stop command success: 99%
- Three stop commands all fail: 0.01³ = 0.0001% (1 in 1 million)

✅ **Dramatic reliability improvement**  
✅ **Fast response** (first packet usually arrives in <10ms)  
✅ **Low overhead** (150ms total retry window)

### Layer 3: State Management (Firmware)
The firmware tracks movement state and can detect stale commands.

## Implementation in Bitfocus Companion

### Duration Group Approach

**Press Actions:**
```
1. Sony VISCA → Pan Left (speed: 0x14)
```

**Duration Group (100ms, Execute while held):**
```
1. internal: Button: Trigger Press (this button, force if already pressed)
```

**Release Actions:**
```
1. Sony VISCA → Pan/Tilt Stop
2. internal: Wait → 50ms
3. Sony VISCA → Pan/Tilt Stop
4. internal: Wait → 50ms
5. Sony VISCA → Pan/Tilt Stop
```

### Logic While Approach (Companion 5.x)

**Press Actions:**
```
1. internal: Logic While
   - Condition: internal:buttonPushed
   - Actions:
     a. Sony VISCA → Pan Left
     b. internal: Wait → 100ms
```

**Release Actions:**
```
1. Sony VISCA → Pan/Tilt Stop
2. internal: Wait → 50ms
3. Sony VISCA → Pan/Tilt Stop
4. internal: Wait → 50ms
5. Sony VISCA → Pan/Tilt Stop
```

## Why 50ms Between Retries?

**Network considerations:**
- Typical WiFi RTT: 5-20ms
- Packet processing time: <5ms
- Total round trip: ~10-25ms

**50ms spacing provides:**
- Enough time for first packet to arrive and be processed
- Not so long that it delays the final stop
- Total retry window: 150ms (imperceptible to user)

**Alternative timings:**
- **25ms:** Faster, but might send duplicates before first arrives
- **100ms:** Safer, but user notices the delay
- **50ms:** Sweet spot

## Why Not Just Use TCP?

TCP VISCA (port 5678) does exist, but:

1. **Most VISCA controllers only support UDP**
   - Legacy protocol from 1990s
   - Hardware controllers are UDP-only
   
2. **TCP adds overhead**
   - Connection management
   - Retransmission delays (can be 200-1000ms)
   - Head-of-line blocking

3. **Multiple retries are simpler**
   - Works with both TCP and UDP
   - No connection state to manage
   - Predictable timing

## Comparison: HTTP vs VISCA

| Feature | HTTP API | VISCA + Retries |
|---------|----------|-----------------|
| **Transport** | TCP | UDP |
| **Retransmission** | Automatic (TCP layer) | Manual (app layer) |
| **Stop reliability** | 99.99%+ | ~99.9999% (with 3x) |
| **Latency** | 10-50ms | 5-15ms |
| **Complexity** | Single command | Retry loop required |
| **Protocol** | JSON | Binary |
| **Debugging** | curl/browser | Wireshark |

## Testing Stop Reliability

### Test Setup
1. Configure Companion button with VISCA + 3x stop retries
2. Enable packet loss simulation (netem on Linux):
   ```bash
   sudo tc qdisc add dev wlan0 root netem loss 5%
   ```
3. Press and release button rapidly 100 times
4. Monitor gimbal behavior

### Expected Results

**Without retries (single stop):**
- 5% packet loss = ~5 runaway stops per 100 presses
- Watchdog catches them after 1 second

**With 3x retries:**
- 5% packet loss = ~0.0125% failure rate
- Expect <1 runaway per 8000 presses
- Watchdog still provides backup

## Recommendations

### For Companion Users (Software Control)
**Best:** Use HTTP API
- Native TCP reliability
- No retry logic needed
- Full feature set

**Good:** VISCA with 3x stop retries
- Works if you need VISCA compatibility
- Requires careful button configuration
- Almost as reliable as TCP

### For Hardware Controllers
**Best:** Use controllers that send continuous commands
- Sony RM-IP series
- PTZOptics hardware controllers
- Watchdog provides safety net

**Alternative:** Enable longer watchdog timeout
```ini
-DVISCA_WATCHDOG_MS=2000  ; 2 seconds for slower controllers
```

### For Critical Applications
**Consider Pelco-D/P:**
- Each frame contains complete state
- Self-correcting on packet loss
- No stop command needed (sends position continuously)
- Enable with: `-DENABLE_PELCO=1`

## Conclusion

The combination of:
1. 1000ms watchdog timer
2. 3x stop command retries with 50ms spacing
3. Firmware state management

Provides **99.9999%+ stop reliability** even over unreliable UDP transport.

For new applications, **HTTP API is still recommended** for simplicity and native TCP reliability, but VISCA with proper retry logic is now a viable option for compatibility with existing hardware workflows.

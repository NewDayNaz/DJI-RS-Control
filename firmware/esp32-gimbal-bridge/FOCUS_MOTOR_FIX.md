# Focus Motor Control - Idle Release & Drift Detection

## Problem

The original implementation would "fight" for focus motor control:

1. **Seeding only on first reply**: `applyZoom()` only processed the first focus position reply, then ignored all subsequent updates
2. **Continuous commanding**: Once a target was set, the firmware would command that position at 20 Hz indefinitely
3. **Manual adjustments blocked**: If you tried to manually adjust the Focus Wheel, the firmware would keep overriding it back to its target
4. **No external control**: The gimbal couldn't respond to other control inputs (Focus Wheel, Ronin app, etc.)

## Solution

### 1. Idle Release Mode

After **3 seconds** of inactivity (no new zoom commands and arrived at target):
- Firmware **releases control** and stops sending `focus-set` commands
- Manual Focus Wheel adjustments now work immediately
- Other controllers can take over without conflict

```cpp
// After arriving at target and 3s idle:
g_s.zoomActiveControl = false;  // Release control
// tickZoom() now returns early, stops commanding
```

### 2. Drift Detection

Firmware periodically queries focus position (every **1.5 seconds**):
- Compares actual gimbal position vs. commanded position
- If drift > **50 counts** detected while idle or not moving:
  - Automatically resyncs target to actual position
  - Logs the drift amount for diagnostics
  - Allows external control to take precedence

```cpp
// If actual position differs significantly:
int drift = abs(actual - commanded);
if (drift > 50 && !activelyControlling) {
    // Resync to actual position
    target = actual;
    activeControl = false;
}
```

### 3. Smart State Tracking

New state fields:
- `zoomActual` - Last position reported by gimbal (always updated)
- `zoomActiveControl` - Are we currently controlling the motor?
- `lastZoomCommandMs` - Timestamp of last user command

This allows the firmware to distinguish between:
- **Active control**: User is commanding via web/API/PTZ
- **Passive monitoring**: Just tracking position, not commanding
- **External control**: Someone else (Focus Wheel, app) is controlling

## Behavior

### Normal Operation (Web UI / API)
1. User sets zoom target → `zoomActiveControl = true`
2. Firmware ramps to target at 20 Hz
3. Arrives at target
4. After 3s idle → `zoomActiveControl = false`
5. Manual adjustments now work

### Manual Focus Wheel
1. User adjusts Focus Wheel manually
2. Firmware queries position (every 1.5s)
3. Drift detected (>50 counts)
4. Firmware resyncs: `target = actual`
5. No fighting, manual control works

### PTZ Controller
1. Hardware joystick sends zoom rate command
2. `zoomActiveControl = true` (takes control)
3. User releases joystick
4. Firmware coasts to stop with deceleration
5. After 3s idle → releases control

## Configuration

Constants in `main.cpp`:

```cpp
static constexpr uint32_t ZOOM_IDLE_RELEASE_MS = 1000;  // 1 second idle
static constexpr uint32_t ZOOM_QUERY_INTERVAL_MS = 500; // Query every 500ms
static constexpr int ZOOM_DRIFT_THRESHOLD = 30;         // 30 counts = resync
```

Adjust these if needed:
- Shorter idle timeout = faster handoff to manual control (currently 1s)
- More frequent queries = faster drift detection (currently 500ms)
- Lower threshold = more sensitive to external changes (currently 30 counts)

## API Changes

The state JSON now includes:
- `zoom_actual` - Actual position from gimbal
- `zoom_active_control` - Boolean indicating if we're commanding

This lets clients see:
1. Whether firmware is actively controlling
2. If position differs between commanded and actual

## Serial Log Output

```
[zoom] seeded at position 2048
[zoom] drift detected (120 counts), resynced to 2168
[zoom] idle release - external control allowed
```

Monitor these logs to verify the fix is working correctly.

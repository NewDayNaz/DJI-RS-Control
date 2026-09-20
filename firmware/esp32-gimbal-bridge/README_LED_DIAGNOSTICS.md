# LED Status Diagnostics

The firmware includes LED-based diagnostics using a WS2812 addressable RGB LED connected to **D0 (GPIO2)**.

## Hardware Connection

```
WS2812 LED          XIAO ESP32C3
───────────         ────────────
VCC (5V)     ──────  5V
GND          ──────  GND
DIN          ──────  D0 (GPIO2)
```

**Note:** WS2812 LEDs work with 5V logic but the ESP32C3 outputs 3.3V. This usually works fine for short connections (<1m), but for reliable operation you may want to add a 330Ω resistor in series with DIN or use a level shifter.

## LED Status Patterns

The LED uses color and pattern combinations to indicate system status:

### Normal Operation

| Color | Pattern | Meaning |
|-------|---------|---------|
| **Green** | Solid | ✓ Healthy - CAN active, receiving gimbal data |
| **Blue** | Breathing | Startup sequence |
| **Blue** | Fast blink | WiFi connecting |

### Warnings

| Color | Pattern | Meaning |
|-------|---------|---------|
| **Yellow** | Slow blink (1 Hz) | CAN initialized but no gimbal frames yet |
| **Yellow** | Slow blink | Stale data (>2s since last frame) |

### Errors

| Color | Pattern | Meaning |
|-------|---------|---------|
| **Orange** | Fast blink (4 Hz) | CAN bus-off error (check wiring/termination) |
| **Red** | Solid | CAN controller failed to initialize |
| **Magenta** | Triple flash | Safety watchdog triggered (deadman timer) |

## Troubleshooting with LED

### LED shows Yellow (slow blink)
- **Issue:** CAN initialized but not receiving gimbal frames
- **Check:**
  - Gimbal accessory switch set to **CAN** (not S-BUS)
  - Focus Wheel cable connected
  - P1 termination pad shorted on CAN hat
  - 5V power present (Focus Wheel LED should be on)

### LED shows Orange (fast blink)
- **Issue:** CAN bus-off error - no acknowledgment on bus
- **Check:**
  - CANH/CANL wiring correct (not swapped)
  - GND connected between gimbal and ESP32
  - Termination resistor present (P1 pad shorted)
  - Gimbal powered on

### LED shows Red (solid)
- **Issue:** MCP2515 CAN controller failed to initialize
- **Check:**
  - CAN hat properly seated on XIAO
  - SPI connections (D8/D9/D10/D7/D6)
  - 3.3V power to hat

### LED shows Magenta (triple flash)
- **Issue:** Safety watchdog triggered - gimbal speed zeroed
- **Cause:** WiFi connection dropped or WebSocket client stopped sending speed updates
- **Action:** This is normal safety behavior. Reconnect and resume control.

## Performance Impact

The LED update is non-blocking and lightweight:
- Updates at 50 Hz (20ms interval)
- FastLED.show() takes ~30µs for 1 LED
- Total CPU impact: <0.2%
- No interference with CAN timing or WebSocket performance

## Configuration

To change the LED pin, edit `src/led_status.h`:

```cpp
#define LED_STATUS_PIN 2  // Change to your desired GPIO
```

To disable LED diagnostics entirely, comment out the LED update in `loop()`:

```cpp
// updateLedDiagnostics();
// ledStatusUpdate();
```

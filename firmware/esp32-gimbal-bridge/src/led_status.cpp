// LED status indicator implementation using WS2812 RGB LED.
// Non-blocking, lightweight, designed for real-time systems.

#include "led_status.h"
#include <FastLED.h>

// Single WS2812 LED
#define NUM_LEDS 1
static CRGB g_leds[NUM_LEDS];

// Current state
static LedStatus g_currentStatus = LED_OFF;
static LedPattern g_currentPattern = PATTERN_SOLID;
static uint32_t g_lastUpdateMs = 0;
static uint32_t g_patternStateMs = 0;
static uint8_t g_brightness = 255;
static uint8_t g_flashCount = 0;

// Color definitions (bright, obvious colors)
static const CRGB COLOR_OFF = CRGB::Black;
static const CRGB COLOR_GREEN = CRGB(0, 255, 0);      // Healthy
static const CRGB COLOR_BLUE = CRGB(0, 100, 255);     // Startup/WiFi
static const CRGB COLOR_YELLOW = CRGB(255, 200, 0);   // Warning/No data
static const CRGB COLOR_RED = CRGB(255, 0, 0);        // Error/Critical
static const CRGB COLOR_MAGENTA = CRGB(255, 0, 255);  // Safety event
static const CRGB COLOR_ORANGE = CRGB(255, 80, 0);    // Bus-off

// Get the base color for the current status
static CRGB getStatusColor(LedStatus status) {
    switch (status) {
        case LED_HEALTHY:         return COLOR_GREEN;
        case LED_STARTUP:         return COLOR_BLUE;
        case LED_WIFI_CONNECTING: return COLOR_BLUE;
        case LED_CAN_NO_DATA:     return COLOR_YELLOW;
        case LED_WARNING:         return COLOR_YELLOW;
        case LED_BUS_OFF:         return COLOR_ORANGE;
        case LED_SAFETY_EVENT:    return COLOR_MAGENTA;
        case LED_CRITICAL:        return COLOR_RED;
        case LED_OFF:
        default:                  return COLOR_OFF;
    }
}

// Apply pattern to get the actual LED color
static CRGB applyPattern(CRGB baseColor, LedPattern pattern, uint32_t stateMs) {
    switch (pattern) {
        case PATTERN_SOLID:
            return baseColor;

        case PATTERN_BLINK_SLOW: {
            // 1 Hz: 1000ms on, 1000ms off
            uint32_t phase = stateMs % 2000;
            return phase < 1000 ? baseColor : COLOR_OFF;
        }

        case PATTERN_BLINK_FAST: {
            // 4 Hz: 250ms on, 250ms off
            uint32_t phase = stateMs % 500;
            return phase < 250 ? baseColor : COLOR_OFF;
        }

        case PATTERN_PULSE: {
            // Brief 100ms flash every 2 seconds
            uint32_t phase = stateMs % 2000;
            return phase < 100 ? baseColor : COLOR_OFF;
        }

        case PATTERN_BREATHE: {
            // Smooth sine-wave breathing over 2 seconds
            uint32_t phase = stateMs % 2000;
            float t = phase / 2000.0f;
            float brightness = (sin(t * 2.0f * PI - PI / 2.0f) + 1.0f) / 2.0f;
            return CRGB(
                (uint8_t)(baseColor.r * brightness),
                (uint8_t)(baseColor.g * brightness),
                (uint8_t)(baseColor.b * brightness)
            );
        }

        case PATTERN_FLASH_3X: {
            // Three 100ms flashes, then 2s pause
            uint32_t phase = stateMs % 2600;
            if (phase < 100 || (phase >= 200 && phase < 300) || (phase >= 400 && phase < 500)) {
                return baseColor;
            }
            return COLOR_OFF;
        }

        default:
            return baseColor;
    }
}

void ledStatusInit() {
    FastLED.addLeds<WS2812, LED_STATUS_PIN, GRB>(g_leds, NUM_LEDS);
    FastLED.setBrightness(255);
    g_leds[0] = COLOR_OFF;
    FastLED.show();
    g_lastUpdateMs = millis();
    g_patternStateMs = 0;
}

void ledStatusUpdate() {
    uint32_t now = millis();
    uint32_t dt = now - g_lastUpdateMs;
    
    // Update at 50 Hz (20ms) for smooth patterns, but this is still very lightweight
    if (dt < 20) return;
    
    g_lastUpdateMs = now;
    g_patternStateMs += dt;

    // Get color with pattern applied
    CRGB baseColor = getStatusColor(g_currentStatus);
    CRGB color = applyPattern(baseColor, g_currentPattern, g_patternStateMs);

    // Only update if changed (FastLED.show() takes ~30us for 1 LED, negligible)
    if (g_leds[0] != color) {
        g_leds[0] = color;
        FastLED.show();
    }
}

void ledStatusSet(LedStatus status, LedPattern pattern) {
    // Reset pattern timer when status changes
    if (status != g_currentStatus || pattern != g_currentPattern) {
        g_patternStateMs = 0;
    }
    g_currentStatus = status;
    g_currentPattern = pattern;
}

// Convenience functions
void ledStatusHealthy() {
    ledStatusSet(LED_HEALTHY, PATTERN_SOLID);
}

void ledStatusWarning() {
    ledStatusSet(LED_WARNING, PATTERN_BLINK_SLOW);
}

void ledStatusError() {
    ledStatusSet(LED_BUS_OFF, PATTERN_BLINK_FAST);
}

void ledStatusCritical() {
    ledStatusSet(LED_CRITICAL, PATTERN_SOLID);
}

void ledStatusSafetyEvent() {
    ledStatusSet(LED_SAFETY_EVENT, PATTERN_FLASH_3X);
}

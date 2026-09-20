// LED status indicator for diagnostics using WS2812 RGB LED.
// Non-blocking implementation - safe to call from loop() without delays.

#ifndef LED_STATUS_H
#define LED_STATUS_H

#include <Arduino.h>

// GPIO pin for WS2812 LED (D0 = GPIO2 on XIAO ESP32C3)
#ifndef LED_STATUS_PIN
#define LED_STATUS_PIN 2
#endif

// Status levels (priority order - higher values override lower)
enum LedStatus {
    LED_OFF = 0,
    LED_STARTUP = 1,           // Initial boot
    LED_WIFI_CONNECTING = 2,   // WiFi connection in progress
    LED_CAN_NO_DATA = 3,       // CAN initialized but no gimbal frames
    LED_HEALTHY = 4,           // Normal operation - CAN receiving data
    LED_WARNING = 5,           // Minor issues (TX failures, stale push)
    LED_BUS_OFF = 6,           // CAN bus-off error
    LED_SAFETY_EVENT = 7,      // Deadman timer activated
    LED_CRITICAL = 8,          // CAN init failed or critical error
};

// Pattern types
enum LedPattern {
    PATTERN_SOLID,             // Constant color
    PATTERN_BLINK_SLOW,        // 1 Hz blink (1s on, 1s off)
    PATTERN_BLINK_FAST,        // 4 Hz blink (0.25s on, 0.25s off)
    PATTERN_PULSE,             // Brief flash every 2s
    PATTERN_BREATHE,           // Smooth fade in/out
    PATTERN_FLASH_3X,          // Triple flash then pause
};

// Initialize LED system (call once in setup)
void ledStatusInit();

// Update LED state (call frequently in loop, non-blocking)
void ledStatusUpdate();

// Set the current status level and pattern
void ledStatusSet(LedStatus status, LedPattern pattern = PATTERN_SOLID);

// Convenience functions for common states
void ledStatusHealthy();       // Green solid - normal operation
void ledStatusWarning();       // Yellow blink - minor issues
void ledStatusError();         // Red blink - bus-off or errors
void ledStatusCritical();      // Red solid - fatal error
void ledStatusSafetyEvent();   // Magenta flash 3x - watchdog triggered

#endif // LED_STATUS_H

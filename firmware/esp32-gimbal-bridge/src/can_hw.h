// CAN controller backend. The DJI R SDK packet layer (dji_can_protocol.*) is
// transport-agnostic; this file is the only place that talks to silicon.
//
// Production hardware is the Seeed XIAO CAN Bus Expansion Board (MCP2515 over
// SPI + SN65HVD230). That hat does not connect the ESP32-C3 TWAI pins, so the
// default backend is MCP2515.
//
// [env:esp32_c6_devkit_nx] still compiles the ESP-IDF TWAI driver for the
// Waveshare C6 bench board. Do not drive the gimbal accessory bus from that
// setup; it was a bring-up target only.
#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

struct CanHwFrame {
    uint32_t id = 0;
    uint8_t dlc = 0;
    bool extd = false;
    uint8_t data[8] = {};
};

bool canHwInit();
bool canHwStarted();
bool canHwHealthy();
bool canHwSend(uint32_t id, const uint8_t *data, uint8_t len, uint32_t timeoutMs);
bool canHwReceive(CanHwFrame &out, uint32_t timeoutMs);
void canHwPollHealth();
void canHwFillStatus(JsonObject can);
const char *canHwBackendName();
const char *canHwStateName();
uint32_t canHwBusOffEvents();
uint32_t canHwRxOverflow();

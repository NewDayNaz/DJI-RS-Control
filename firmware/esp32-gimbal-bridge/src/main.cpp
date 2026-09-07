// ESP32 WiFi <-> CAN bridge for the DJI R SDK gimbal (CmdSet 0x0E) and camera (0x0D)
// protocols — a port of the golden Python implementation (dji_gimbal_cli.py for packet
// framing, dji_can_session.py for session behavior, dji_gimbal_web.py for the API).
//
// - First boot (or after a long button-free WiFi failure) opens a "DJI-Gimbal-Setup"
//   captive portal (WiFiManager) so you configure WiFi without reflashing.
// - Serves a single-page joystick/telemetry UI from LittleFS at "/".
// - WebSocket at "/ws": browser -> device JSON commands, device -> browser state at 20 Hz.
// - REST at "/api/*" mirrors dji_gimbal_web.py (state/speed/position/zoom/command/<name>).
// - Session behavior mirrors dji_can_session.py:
//     * startup: enable parameter push, then query the focus motor position
//     * held speed re-sent at 20 Hz; deadman zeroes speed 250 ms after the last update
//     * angle polled at 20 Hz whenever parameter push has been stale for >350 ms
//     * zoom driven to its target by a trapezoidal ramp (vmax/accel) at 20 Hz
// - Power: the gimbal's CAN-port VCC_5V (see docs/DJI_R_SDK_Protocol.md §4.1) both charges
//   the XIAO's battery via its onboard charge IC and, through a divider onto
//   GIMBAL_PRESENT_PIN, drives an auto power-on/off cycle: no "gimbal present" signal for
//   OFF_TIMEOUT_MS drops the board into deep sleep, woken only when that pin goes high
//   again (i.e. reconnected to the gimbal) — see README.md "Power Management". Opt-in via
//   GIMBAL_SLEEP_ENABLED; off by default because the CAN hat leaves D3 hard to reach.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include <cmath>
#include <cstring>

#include "dji_can_protocol.h"
#include "can_hw.h"

// GIMBAL_SLEEP_ENABLED gates the battery-power-management block below (presence sensing +
// deep sleep) — see README.md "Power Management". The Seeed CAN hat uses D6–D10 (INT/CS/SPI)
// and leaves D3 free, but the pin is sandwiched under the hat. Default off so a floating
// INPUT does not deep-sleep the board 60s after boot. Set to 1 once a presence divider is
// actually wired to D3.
#ifndef GIMBAL_SLEEP_ENABLED
#define GIMBAL_SLEEP_ENABLED 0
#endif

#if GIMBAL_SLEEP_ENABLED
// D3 (GPIO5 on the XIAO) senses gimbal VCC_5V through a resistor divider (see README.md).
// GPIO5 is one of the RTC-capable pins broken out on the XIAO (D0-D3 = GPIO2-5). The CAN
// hat occupies D6–D10, so D3 is still the wake pin if you can get a wire to it.
#ifndef GIMBAL_PRESENT_GPIO_NUM
#define GIMBAL_PRESENT_GPIO_NUM 5
#endif
static constexpr gpio_num_t GIMBAL_PRESENT_PIN = (gpio_num_t) GIMBAL_PRESENT_GPIO_NUM;
#endif

static constexpr uint32_t CAN_ID_TX = 0x223; // host -> gimbal
static constexpr uint32_t CAN_ID_RX = 0x222; // gimbal -> host

// ---- Timing (mirrors dji_can_session.py) ----
static constexpr float SPEED_HZ = 20.0f;
static constexpr float ZOOM_HZ = 20.0f;
static constexpr uint32_t SPEED_SEND_INTERVAL_MS = (uint32_t) (1000.0f / SPEED_HZ);
static constexpr uint32_t ZOOM_INTERVAL_MS = (uint32_t) (1000.0f / ZOOM_HZ);
static constexpr uint32_t SPEED_TIMEOUT_MS = 250;   // DEADMAN_S = 0.25
static constexpr uint32_t ANGLE_POLL_MS = 50;       // ANGLE_POLL_S = 0.05
static constexpr uint32_t PUSH_STALE_MS = 350;      // PUSH_STALE_S = 0.35
static constexpr uint32_t STATE_BROADCAST_MS = 50;  // golden web streams snapshot at 20 Hz
static constexpr uint32_t OFF_TIMEOUT_MS = 60000;   // deep-sleep after this long unplugged

// ---- Zoom ramp limits (dji_can_session.py) ----
static constexpr float ZOOM_MIN = 0.0f;
static constexpr float ZOOM_MAX = 4096.0f;
static constexpr float ZOOM_VMAX_DEFAULT = 900.0f;
static constexpr float ZOOM_ACCEL_DEFAULT = 1800.0f;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

dji::Reassembler g_reassembler;

// ---------------------------------------------------------------------------
// Session state (mirrors GimbalCanSession). All fields guarded by g_stateLock.
// ---------------------------------------------------------------------------
struct SessionState {
    // held speed; valid while heldActive
    bool heldActive = false;
    float heldYaw = 0.0f, heldRoll = 0.0f, heldPitch = 0.0f;
    uint32_t heldAtMs = 0;
    bool sentZero = true;

    // telemetry
    bool haveAngles = false;
    dji::GimbalAngles angles = {};
    uint32_t lastRxMs = 0;   // last valid packet that carried telemetry
    uint32_t lastPushMs = 0; // last 0x08 push specifically
    uint32_t rxOk = 0;

    // zoom (focus motor position, 0-4096) with trapezoidal ramp
    bool haveZoomCmd = false;    // false until seeded by the first focus reply
    float zoomCmd = 0.0f;
    float zoomVel = 0.0f;
    bool haveZoomTarget = false;
    float zoomTarget = 0.0f;
    bool haveZoomSent = false;
    int zoomSent = 0;
    int zoomReported = 0;        // last commanded position (golden `_zoom`)
    bool haveZoomReported = false;
    float zoomVmax = ZOOM_VMAX_DEFAULT;
    float zoomAccel = ZOOM_ACCEL_DEFAULT;

    // last error surfaced to the UI
    char lastError[96] = "";
};
static SessionState g_s;
static SemaphoreHandle_t g_stateLock = nullptr;

static uint32_t g_lastSpeedSendMs = 0;
static uint32_t g_lastZoomMs = 0;
static uint32_t g_lastAnglePollMs = 0;
static uint32_t g_lastBroadcastMs = 0;

static char g_version[24] = "";
static bool g_haveVersion = false;
static dji::GimbalLimits g_limits = {};
static bool g_haveLimits = false;

// Controller/transceiver bring-up counters. TX success means some other node ACKed
// (gimbal, Canable, etc). RX of 0x222 means the gimbal SDK path is alive.
struct CanStats {
    bool started = false;
    uint32_t txOk = 0;
    uint32_t txFail = 0;
    uint32_t rxFrames = 0;
    uint32_t rx222 = 0;
    uint32_t lastRxId = 0;
    uint32_t lastRxDlc = 0;
    uint32_t busOffEvents = 0;
    uint32_t rxQueueFull = 0;
    bool lastExtd = false;
    uint8_t lastData[8] = {};
    static constexpr int kHist = 12;
    uint32_t histId[kHist] = {};
    uint32_t histN[kHist] = {};
    uint32_t histOther = 0;
    char lastTxHex[96] = "";
};
static CanStats g_can;

static void noteCanRxId(uint32_t id) {
    for (int i = 0; i < CanStats::kHist; i++) {
        if (g_can.histN[i] == 0) {
            g_can.histId[i] = id;
            g_can.histN[i] = 1;
            return;
        }
        if (g_can.histId[i] == id) {
            g_can.histN[i]++;
            return;
        }
    }
    g_can.histOther++;
}

#if GIMBAL_SLEEP_ENABLED
// Updated to millis() any time GIMBAL_PRESENT_PIN reads high; drives the deep-sleep timeout.
static uint32_t g_lastGimbalPresentMillis = 0;

// ---------------------------------------------------------------------------
// Power management: gimbal-present sensing + deep sleep
// ---------------------------------------------------------------------------

// Drops the board into deep sleep, to be woken only by GIMBAL_PRESENT_PIN going high
// again (i.e. the CAN cable being reconnected to a powered gimbal). Does not return.
static void enterDeepSleepUntilGimbalPresent() {
    Serial.println("[power] no gimbal power sensed for the timeout period; deep-sleeping "
                    "until reconnected");
    Serial.flush();

    ws.closeAll();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);

    rtc_gpio_pullup_dis(GIMBAL_PRESENT_PIN);
    rtc_gpio_pulldown_dis(GIMBAL_PRESENT_PIN); // external divider already sets the idle level
    // NOTE: ESP32-C3 does NOT support the classic esp_sleep_enable_ext0_wakeup() API used on
    // original ESP32/S2/S3 (no ULP/RTC controller of that shape) — it needs this GPIO-specific
    // call instead, which requires an IDF 5.x-based arduino-esp32 core (>=3.0.0). If this fails
    // to link, your resolved arduino-esp32 core is too old; update the platform in platformio.ini.
    esp_deep_sleep_enable_gpio_wakeup(1ULL << GIMBAL_PRESENT_PIN, ESP_GPIO_WAKEUP_GPIO_HIGH);
    esp_deep_sleep_start();
    // unreachable
}
#endif // GIMBAL_SLEEP_ENABLED

// ---------------------------------------------------------------------------
// CAN send/receive
// ---------------------------------------------------------------------------

static SemaphoreHandle_t g_canTxLock = nullptr;

static bool waitCanHealthy(uint32_t timeoutMs) {
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        canHwPollHealth();
        if (canHwHealthy()) return true;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}

static bool canSendPacketOnce(const std::vector<uint8_t> &pkt) {
    if (!waitCanHealthy(200)) return false;
    for (size_t i = 0; i < pkt.size(); i += 8) {
        size_t chunkLen = min((size_t) 8, pkt.size() - i);
        if (!canHwSend(CAN_ID_TX, pkt.data() + i, (uint8_t) chunkLen, 50)) {
            g_can.txFail++;
            return false;
        }
        g_can.txOk++;
        if (!canHwHealthy()) return false;
    }
    return true;
}

// One SDK packet is several 8-byte 0x223 frames. loop() (push/speed) and the
// HTTP/WS handlers all transmit; without a lock those frames interleave and
// the gimbal's 0x223 reassembler sees garbage (CAN ACK still succeeds).
static bool canSendPacket(const std::vector<uint8_t> &pkt) {
    if (!canHwStarted() || !g_canTxLock) return false;
    if (xSemaphoreTake(g_canTxLock, pdMS_TO_TICKS(400)) != pdTRUE) return false;
    g_can.lastTxHex[0] = '\0';
    for (size_t i = 0; i < pkt.size() && strlen(g_can.lastTxHex) + 3 < sizeof(g_can.lastTxHex); i++) {
        snprintf(g_can.lastTxHex + strlen(g_can.lastTxHex),
                 sizeof(g_can.lastTxHex) - strlen(g_can.lastTxHex),
                 "%s%02X", i ? " " : "", pkt[i]);
    }
    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; attempt++) {
        if (attempt > 0) vTaskDelay(pdMS_TO_TICKS(50));
        ok = canSendPacketOnce(pkt);
    }
    xSemaphoreGive(g_canTxLock);
    return ok;
}

// ---------------------------------------------------------------------------
// Zoom ramp — direct port of zoom_ramp_step() in dji_can_session.py.
// ---------------------------------------------------------------------------

static bool zoomRampStep(float &pos, float &vel, float target, float vmax, float accel, float dt) {
    vmax = fmaxf(1.0f, vmax);
    accel = fmaxf(1.0f, accel);
    dt = fminf(fmaxf(dt, 0.0f), 0.1f);
    float remaining = target - pos;
    if (fabsf(remaining) < 0.5f && fabsf(vel) < 8.0f) {
        pos = target;
        vel = 0.0f;
        return true;
    }
    if (dt <= 0.0f) return false;

    float want = remaining > 0.0f ? 1.0f : -1.0f;
    float stopDist = (vel * vel) / (2.0f * accel);
    float acc;
    if (vel * remaining < 0.0f) {
        acc = -copysignf(accel, vel);
    } else if (stopDist >= fabsf(remaining)) {
        acc = fabsf(vel) > 1e-6f ? -copysignf(accel, vel) : 0.0f;
    } else if (fabsf(vel) < vmax) {
        acc = want * accel;
    } else {
        acc = 0.0f;
        vel = copysignf(vmax, vel);
    }

    vel = fmaxf(-vmax, fminf(vmax, vel + acc * dt));
    pos = pos + vel * dt;
    if ((remaining > 0.0f && pos >= target) || (remaining < 0.0f && pos <= target)) {
        pos = target;
        vel = 0.0f;
        return true;
    }
    return false;
}

// Mirrors GimbalCanSession._tick_zoom: advance the ramp at ZOOM_HZ and transmit
// focus-set only when the commanded integer position changes.
static void tickZoom(uint32_t now) {
    float dt = (now - g_lastZoomMs) / 1000.0f;
    g_lastZoomMs = now;

    xSemaphoreTake(g_stateLock, portMAX_DELAY);
    if (!g_s.haveZoomTarget || !g_s.haveZoomCmd) {
        xSemaphoreGive(g_stateLock);
        return;
    }
    float pos = g_s.zoomCmd;
    float vel = g_s.zoomVel;
    float target = g_s.zoomTarget;
    float vmax = g_s.zoomVmax;
    float accel = g_s.zoomAccel;
    bool arrived = zoomRampStep(pos, vel, target, vmax, accel, dt);
    pos = fmaxf(ZOOM_MIN, fminf(ZOOM_MAX, pos));
    int commanded = (int) lroundf(pos);
    g_s.zoomCmd = pos;
    g_s.zoomVel = arrived ? 0.0f : vel;
    g_s.zoomReported = commanded;
    g_s.haveZoomReported = true;
    bool needSend = !g_s.haveZoomSent || commanded != g_s.zoomSent;
    if (needSend) {
        g_s.zoomSent = commanded;
        g_s.haveZoomSent = true;
    }
    xSemaphoreGive(g_stateLock);

    if (needSend) {
        canSendPacket(dji::buildFocusSet((uint16_t) commanded));
    }
}

// ---------------------------------------------------------------------------
// Held speed — mirrors hold_speed/release_speed + the SPEED_HZ retransmit loop
// ---------------------------------------------------------------------------

static void setHeldSpeed(float yaw, float roll, float pitch) {
    xSemaphoreTake(g_stateLock, portMAX_DELAY);
    g_s.heldYaw = yaw;
    g_s.heldRoll = roll;
    g_s.heldPitch = pitch;
    g_s.heldAtMs = millis();
    g_s.heldActive = true;
    xSemaphoreGive(g_stateLock);
}

// Release always emits one zero-speed packet, like GimbalCanSession.release_speed().
static void releaseHeldSpeed() {
    xSemaphoreTake(g_stateLock, portMAX_DELAY);
    g_s.heldActive = false;
    g_s.sentZero = true;
    xSemaphoreGive(g_stateLock);
    canSendPacket(dji::buildControlSpeed(0.0f, 0.0f, 0.0f));
}

static void tickSpeed(uint32_t now) {
    xSemaphoreTake(g_stateLock, portMAX_DELAY);
    if (!g_s.heldActive) {
        xSemaphoreGive(g_stateLock);
        return;
    }
    if (now - g_s.heldAtMs > SPEED_TIMEOUT_MS) {
        // Deadman: WiFi drops must not leave the gimbal spinning.
        g_s.heldActive = false;
        g_s.sentZero = true;
        xSemaphoreGive(g_stateLock);
        Serial.println("[safety] speed watchdog: no update in time, zeroed gimbal speed");
        canSendPacket(dji::buildControlSpeed(0.0f, 0.0f, 0.0f));
        g_lastSpeedSendMs = now;
        return;
    }
    float yaw = g_s.heldYaw, roll = g_s.heldRoll, pitch = g_s.heldPitch;
    bool moving = fabsf(yaw) > 0.05f || fabsf(roll) > 0.05f || fabsf(pitch) > 0.05f;
    bool sendZero = !moving && !g_s.sentZero;
    bool sendMove = moving && (now - g_lastSpeedSendMs >= SPEED_SEND_INTERVAL_MS);
    if (sendZero) g_s.sentZero = true;
    else if (sendMove) g_s.sentZero = false;
    xSemaphoreGive(g_stateLock);

    if (sendMove) {
        canSendPacket(dji::buildControlSpeed(yaw, roll, pitch));
        g_lastSpeedSendMs = now;
    } else if (sendZero) {
        canSendPacket(dji::buildControlSpeed(0.0f, 0.0f, 0.0f));
    }
}

// Golden angle-poll fallback: while parameter push has been stale for PUSH_STALE_MS,
// poll Obtain gimbal angle (attitude) at ANGLE_POLL_MS.
static void tickAnglePoll(uint32_t now) {
    xSemaphoreTake(g_stateLock, portMAX_DELAY);
    uint32_t lastPush = g_s.lastPushMs;
    xSemaphoreGive(g_stateLock);
    if (now - g_lastAnglePollMs < ANGLE_POLL_MS) return;
    if (lastPush != 0 && now - lastPush <= PUSH_STALE_MS) return;
    g_lastAnglePollMs = now;
    canSendPacket(dji::buildObtainGimbalAngle(0x01));
}

// ---------------------------------------------------------------------------
// Zoom / command API (called from WS and REST handlers)
// ---------------------------------------------------------------------------

static void setZoomTarget(int position) {
    float pos = (float) constrain(position, (int) ZOOM_MIN, (int) ZOOM_MAX);
    xSemaphoreTake(g_stateLock, portMAX_DELAY);
    g_s.zoomTarget = pos;
    g_s.haveZoomTarget = true;
    xSemaphoreGive(g_stateLock);
}

static void setZoomProfile(float vmax, float accel) {
    xSemaphoreTake(g_stateLock, portMAX_DELAY);
    g_s.zoomVmax = fmaxf(50.0f, fminf(8000.0f, vmax));
    g_s.zoomAccel = fmaxf(50.0f, fminf(40000.0f, accel));
    xSemaphoreGive(g_stateLock);
}

// Mirrors GimbalCanSession._exec's motor-calib branch: clear the ramp state so the
// next focus reply re-seeds it, then run autocal and re-query the position.
static void motorCalibrate() {
    xSemaphoreTake(g_stateLock, portMAX_DELAY);
    g_s.haveZoomCmd = false;
    g_s.zoomVel = 0.0f;
    g_s.haveZoomTarget = false;
    g_s.haveZoomSent = false;
    g_s.haveZoomReported = false;
    xSemaphoreGive(g_stateLock);
    canSendPacket(dji::buildMotorCalibrate());
    canSendPacket(dji::buildFocusGet());
}

// The named command set from dji_gimbal_web.py's /api/command/<name> allow-list,
// plus "version" from the CLI. Returns false for an unknown name.
static bool execNamedCommand(const char *name) {
    if (strcmp(name, "stop") == 0) {
        releaseHeldSpeed();
        return true;
    }
    if (strcmp(name, "motor-calib") == 0) {
        motorCalibrate();
        return true;
    }
    if (strcmp(name, "sleep") == 0) return canSendPacket(dji::buildSleep());
    if (strcmp(name, "wake") == 0) return canSendPacket(dji::buildWake());
    if (strcmp(name, "recenter") == 0) return canSendPacket(dji::buildRecenterSelfie(0x01));
    if (strcmp(name, "selfie") == 0) return canSendPacket(dji::buildRecenterSelfie(0x02));
    if (strcmp(name, "calibrate") == 0) return canSendPacket(dji::buildCalibrate());
    if (strcmp(name, "activetrack") == 0) return canSendPacket(dji::buildActiveTrackToggle());
    if (strcmp(name, "rec-start") == 0) return canSendPacket(dji::buildRecordStart());
    if (strcmp(name, "rec-stop") == 0) return canSendPacket(dji::buildRecordStop());
    if (strcmp(name, "focus-center-start") == 0) return canSendPacket(dji::buildFocusCenterStart());
    if (strcmp(name, "focus-center-stop") == 0) return canSendPacket(dji::buildFocusCenterStop());
    if (strcmp(name, "push-on") == 0) return canSendPacket(dji::buildSetParameterPush(true));
    if (strcmp(name, "push-off") == 0) return canSendPacket(dji::buildSetParameterPush(false));
    if (strcmp(name, "cam-cmd") == 0) return canSendPacket(dji::buildCameraCmd());
    if (strcmp(name, "limit") == 0) return canSendPacket(dji::buildObtainGimbalLimitAngle());
    if (strcmp(name, "version") == 0) return canSendPacket(dji::buildObtainModuleVersion());
    if (strcmp(name, "zoom_get") == 0) return canSendPacket(dji::buildFocusGet());
    return false;
}

static const char *kCommandNames[] = {
    "sleep", "wake", "recenter", "selfie", "calibrate", "motor-calib", "activetrack",
    "rec-start", "rec-stop", "focus-center-start", "focus-center-stop",
    "push-on", "push-off", "cam-cmd", "limit", "version", "zoom_get", "stop",
};

// ---------------------------------------------------------------------------
// State snapshot — same keys as dji_gimbal_web.py's _snap_dict, plus ESP32 extras.
// ---------------------------------------------------------------------------

static void fillStateJson(JsonObject doc) {
    uint32_t now = millis();
    xSemaphoreTake(g_stateLock, portMAX_DELAY);
    bool connected = canHwStarted() && canHwHealthy();
    doc["connected"] = connected;
    doc["adapter"] = canHwBackendName();
    doc["interface"] = "can";
    if (g_s.haveAngles) {
        doc["yaw"] = g_s.angles.yawDeg;
        doc["roll"] = g_s.angles.rollDeg;
        doc["pitch"] = g_s.angles.pitchDeg;
    } else {
        doc["yaw"] = nullptr;
        doc["roll"] = nullptr;
        doc["pitch"] = nullptr;
    }
    if (g_s.haveZoomReported) doc["zoom"] = g_s.zoomReported; else doc["zoom"] = nullptr;
    if (g_s.haveZoomTarget) doc["zoom_target"] = (int) lroundf(g_s.zoomTarget);
    else doc["zoom_target"] = nullptr;
    doc["zoom_vmax"] = g_s.zoomVmax;
    doc["zoom_accel"] = g_s.zoomAccel;
    if (g_s.lastRxMs != 0) doc["last_rx_age_s"] = (now - g_s.lastRxMs) / 1000.0f;
    else doc["last_rx_age_s"] = nullptr;
    doc["last_error"] = g_s.lastError[0] ? g_s.lastError : nullptr;
    doc["tx_ok"] = g_can.txOk;
    doc["rx_ok"] = g_s.rxOk;
    xSemaphoreGive(g_stateLock);

    // ESP32-specific extras.
    doc["rssi"] = WiFi.RSSI();
    doc["can_state"] = canHwStateName();
    if (g_haveVersion) doc["version"] = g_version;
    if (g_haveLimits) {
        JsonObject lim = doc["limits"].to<JsonObject>();
        lim["yaw_min"] = g_limits.yawMin;
        lim["yaw_max"] = g_limits.yawMax;
        lim["roll_min"] = g_limits.rollMin;
        lim["roll_max"] = g_limits.rollMax;
        lim["pitch_min"] = g_limits.pitchMin;
        lim["pitch_max"] = g_limits.pitchMax;
    }
}

static void broadcastState(uint32_t now) {
    if (ws.count() == 0) return;
    if (now - g_lastBroadcastMs < STATE_BROADCAST_MS) return;
    g_lastBroadcastMs = now;
    JsonDocument doc;
    fillStateJson(doc.to<JsonObject>());
    String out;
    serializeJson(doc, out);
    ws.textAll(out);
}

// ---------------------------------------------------------------------------
// CAN diagnostics (ESP32-specific; no golden equivalent)
// ---------------------------------------------------------------------------

static void fillCanStatusJson(JsonObject can) {
    canHwFillStatus(can);
    can["tx_ok"] = g_can.txOk;
    can["tx_fail"] = g_can.txFail;
    can["rx"] = g_can.rxFrames;
    can["rx_222"] = g_can.rx222;
    can["last_rx_id"] = g_can.lastRxId;
    char hexId[12];
    snprintf(hexId, sizeof(hexId), "0x%lX", (unsigned long) g_can.lastRxId);
    can["last_rx_id_hex"] = hexId;
    can["last_rx_extd"] = g_can.lastExtd;
    char hexData[24];
    hexData[0] = '\0';
    for (uint32_t i = 0; i < g_can.lastRxDlc && i < 8; i++) {
        snprintf(hexData + strlen(hexData), sizeof(hexData) - strlen(hexData),
                 "%s%02X", i ? " " : "", g_can.lastData[i]);
    }
    can["last_rx_data"] = hexData;
    JsonArray ids = can["rx_ids"].to<JsonArray>();
    for (int i = 0; i < CanStats::kHist; i++) {
        if (g_can.histN[i] == 0) break;
        JsonObject row = ids.add<JsonObject>();
        row["id"] = g_can.histId[i];
        row["n"] = g_can.histN[i];
    }
    can["rx_ids_other"] = g_can.histOther;
    can["rx_queue_full"] = canHwRxOverflow();
    can["bus_off_events"] = canHwBusOffEvents();
    can["last_tx"] = g_can.lastTxHex;
    xSemaphoreTake(g_stateLock, portMAX_DELAY);
    if (g_s.haveAngles) {
        can["yaw"] = g_s.angles.yawDeg;
        can["roll"] = g_s.angles.rollDeg;
        can["pitch"] = g_s.angles.pitchDeg;
    }
    xSemaphoreGive(g_stateLock);

    const char *state = can["state"] | "unknown";
    const char *hint = "no traffic yet";
    char initHint[96] = "";
    if (!canHwStarted()) {
        const char *initError = can["init_error"] | "";
        if (initError[0]) {
            strncpy(initHint, initError, sizeof(initHint) - 1);
            hint = initHint;
        } else {
            hint = "CAN controller failed to start";
        }
    } else if (strcmp(state, "bus_off") == 0 || strcmp(state, "recovering") == 0) {
        hint = "bus-off: nobody ACKed. CANH/CANL/GND, P1 terminator, "
               "gimbal unplugged, or CAN/S-BUS switch.";
    } else if (g_can.rx222 > 0) {
        hint = "gimbal frames on 0x222 — RX path is good";
    } else if (g_can.rxFrames > 0) {
        hint = "bus is alive but not 0x222 — see rx_ids. SDK replies are sparse; "
               "0x530/0x531/0x426 are a second protocol on this wire.";
    } else if (g_can.txOk > 0) {
        hint = "TX ACKed (a peer is on the bus) but no 0x222 yet. Enable telemetry push.";
    } else if (g_can.txFail > 0) {
        hint = "TX failing, RX silent — no peer, or the hat is not on the bus";
    }
    can["hint"] = hint;
}

// ---------------------------------------------------------------------------
// RX path
// ---------------------------------------------------------------------------

static void applyAngles(const dji::GimbalAngles &a, bool isPush) {
    xSemaphoreTake(g_stateLock, portMAX_DELAY);
    g_s.angles = a;
    g_s.haveAngles = true;
    g_s.lastRxMs = millis();
    if (isPush) g_s.lastPushMs = g_s.lastRxMs;
    g_s.rxOk++;
    xSemaphoreGive(g_stateLock);
}

// Mirrors GimbalCanSession._apply_zoom: the first focus reply seeds the ramp state.
static void applyZoom(uint32_t zoom) {
    uint32_t z = min(zoom, (uint32_t) ZOOM_MAX);
    xSemaphoreTake(g_stateLock, portMAX_DELAY);
    if (!g_s.haveZoomCmd) {
        g_s.zoomCmd = (float) z;
        if (!g_s.haveZoomTarget) {
            g_s.zoomTarget = (float) z;
            g_s.haveZoomTarget = true;
        }
        g_s.zoomVel = 0.0f;
        g_s.zoomReported = (int) z;
        g_s.haveZoomReported = true;
        g_s.haveZoomCmd = true;
    }
    g_s.lastRxMs = millis();
    xSemaphoreGive(g_stateLock);
}

static void handleCanFrame(const CanHwFrame &rx) {
    g_can.rxFrames++;
    g_can.lastRxId = rx.id;
    g_can.lastRxDlc = rx.dlc;
    g_can.lastExtd = rx.extd;
    memset(g_can.lastData, 0, sizeof(g_can.lastData));
    uint8_t n = rx.dlc;
    if (n > sizeof(g_can.lastData)) n = sizeof(g_can.lastData);
    memcpy(g_can.lastData, rx.data, n);
    noteCanRxId(rx.id);
    if (rx.id != CAN_ID_RX) return;
    g_can.rx222++;

    std::vector<uint8_t> pkt;
    if (!g_reassembler.feed(rx.data, rx.dlc, pkt)) return;

    dji::SdkReply reply;
    if (!dji::validateSdkReply(pkt.data(), pkt.size(), reply)) return;

    if (reply.cmdSet == dji::CMD_SET_CAMERA && reply.cmdId == dji::CMD_ID_CAMERA_CMD) {
        // A working client latches a first payload byte of 0x02 here.
        Serial.printf("[cam] 0x0D/0x01 ret=0x%02X first=0x%02X len=%u\n",
                      reply.retCode, reply.dataLen ? reply.data[0] : 0,
                      (unsigned) reply.dataLen);
        return;
    }
    if (reply.cmdSet != dji::CMD_SET_GIMBAL) return;

    switch (reply.cmdId) {
        case dji::CMD_ID_PUSH_PARAMS: {
            dji::GimbalAngles a;
            if (dji::parsePushAngles(reply, a)) applyAngles(a, true);
            break;
        }
        case dji::CMD_ID_ANGLE: {
            dji::GimbalAngles a;
            if (dji::parseGimbalAngles(reply, a)) applyAngles(a, false);
            break;
        }
        case dji::CMD_ID_FOCUS: {
            uint32_t pos;
            if (dji::parseFocusPosition(reply, pos)) applyZoom(pos);
            break;
        }
        case dji::CMD_ID_MODULE_VERSION: {
            uint32_t deviceId, ver;
            if (dji::parseModuleVersion(reply, deviceId, ver)) {
                dji::formatModuleVersion(ver, g_version, sizeof(g_version));
                g_haveVersion = true;
                Serial.printf("[gimbal] device_id=0x%08lX version=%s\n",
                              (unsigned long) deviceId, g_version);
            }
            break;
        }
        case dji::CMD_ID_LIMIT_ANGLE: {
            dji::GimbalLimits lim;
            if (dji::parseLimitAngles(reply, lim)) {
                g_limits = lim;
                g_haveLimits = true;
                Serial.printf("[gimbal] limits yaw[%.1f,%.1f] roll[%.1f,%.1f] pitch[%.1f,%.1f]\n",
                              lim.yawMin, lim.yawMax, lim.rollMin, lim.rollMax,
                              lim.pitchMin, lim.pitchMax);
            }
            break;
        }
        case dji::CMD_ID_CALIB_STATUS:
            // Auto-calibration status push — observed but not decoded (docs §Status).
            break;
        default:
            break;
    }
}

// Drain on its own task so Wi-Fi / HTTP in loop() cannot stall RX. The MCP2515
// only has two buffers; with the 0x222 hardware filter that is enough. Do not
// Serial-print per frame.
static void canRxTask(void *) {
    CanHwFrame rx;
    for (;;) {
        if (canHwReceive(rx, 50)) {
            handleCanFrame(rx);
            while (canHwReceive(rx, 0)) handleCanFrame(rx);
        }
    }
}

// ---------------------------------------------------------------------------
// Command handling (shared by WebSocket and REST)
// ---------------------------------------------------------------------------

static void handleSpeedCommand(float yaw, float roll, float pitch, bool hold) {
    if (hold) {
        setHeldSpeed(yaw, roll, pitch);
        // Send immediately so the first deflection isn't delayed by the cadence.
        uint32_t now = millis();
        bool moving = fabsf(yaw) > 0.05f || fabsf(roll) > 0.05f || fabsf(pitch) > 0.05f;
        if (moving && now - g_lastSpeedSendMs >= SPEED_SEND_INTERVAL_MS) {
            canSendPacket(dji::buildControlSpeed(yaw, roll, pitch));
            g_lastSpeedSendMs = now;
        }
    } else {
        // Golden hold=false path: release the hold, then fire this speed once.
        xSemaphoreTake(g_stateLock, portMAX_DELAY);
        g_s.heldActive = false;
        g_s.sentZero = true;
        xSemaphoreGive(g_stateLock);
        canSendPacket(dji::buildControlSpeed(yaw, roll, pitch));
    }
}

static void handlePositionCommand(float yaw, float roll, float pitch, float timeS) {
    timeS = fmaxf(0.0f, fminf(25.5f, timeS));
    canSendPacket(dji::buildControlPosition(yaw, roll, pitch, true, timeS));
}

static bool handleJsonCommand(JsonDocument &doc) {
    const char *cmd = doc["cmd"] | "";
    if (strcmp(cmd, "speed") == 0) {
        handleSpeedCommand(doc["yaw"] | 0.0f, doc["roll"] | 0.0f, doc["pitch"] | 0.0f,
                           doc["hold"] | true);
    } else if (strcmp(cmd, "position") == 0) {
        handlePositionCommand(doc["yaw"] | 0.0f, doc["roll"] | 0.0f, doc["pitch"] | 0.0f,
                              doc["time_s"] | 0.4f);
    } else if (strcmp(cmd, "zoom") == 0) {
        setZoomTarget(doc["position"] | 0);
    } else if (strcmp(cmd, "zoom_profile") == 0) {
        setZoomProfile(doc["vmax"] | ZOOM_VMAX_DEFAULT, doc["accel"] | ZOOM_ACCEL_DEFAULT);
    } else if (*cmd) {
        bool ok = execNamedCommand(cmd);
        if (!ok) Serial.printf("[ws] unknown cmd: %s\n", cmd);
        return ok;
    } else {
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// WebSocket event handling
// ---------------------------------------------------------------------------

static void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                       AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("[ws] client #%u connected from %s\n", client->id(),
                      client->remoteIP().toString().c_str());
        // Send a snapshot immediately so the UI doesn't wait for the next tick.
        JsonDocument doc;
        fillStateJson(doc.to<JsonObject>());
        String out;
        serializeJson(doc, out);
        client->text(out);
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[ws] client #%u disconnected\n", client->id());
        // If nobody is connected any more, stop driving speed for safety.
        if (ws.count() == 0) {
            releaseHeldSpeed();
        }
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo *) arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, (const char *) data, len);
            if (!err) {
                handleJsonCommand(doc);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// REST API (mirrors dji_gimbal_web.py; speed streaming should use the WebSocket)
// ---------------------------------------------------------------------------

static void sendJsonOk(AsyncWebServerRequest *req, const char *name) {
    JsonDocument doc;
    doc["ok"] = name;
    String out;
    serializeJson(doc, out);
    req->send(200, "application/json", out);
}

static void sendState(AsyncWebServerRequest *req) {
    JsonDocument doc;
    fillStateJson(doc.to<JsonObject>());
    String out;
    serializeJson(doc, out);
    req->send(200, "application/json", out);
}

static void setupRestApi() {
    server.on("/api/state", HTTP_GET, [](AsyncWebServerRequest *req) { sendState(req); });

    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *req) {
        JsonDocument doc;
        doc["wifi_rssi"] = WiFi.RSSI();
        doc["ip"] = WiFi.localIP().toString();
        doc["ws_clients"] = ws.count();
        fillCanStatusJson(doc["can"].to<JsonObject>());
        String out;
        serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    auto *speed = new AsyncCallbackJsonWebHandler("/api/speed",
        [](AsyncWebServerRequest *req, JsonVariant &json) {
            JsonObject body = json.as<JsonObject>();
            handleSpeedCommand(body["yaw"] | 0.0f, body["roll"] | 0.0f,
                               body["pitch"] | 0.0f, body["hold"] | true);
            sendJsonOk(req, "speed");
        });
    server.addHandler(speed);

    auto *position = new AsyncCallbackJsonWebHandler("/api/position",
        [](AsyncWebServerRequest *req, JsonVariant &json) {
            JsonObject body = json.as<JsonObject>();
            handlePositionCommand(body["yaw"] | 0.0f, body["roll"] | 0.0f,
                                  body["pitch"] | 0.0f, body["time_s"] | 0.4f);
            sendJsonOk(req, "position");
        });
    server.addHandler(position);

    auto *zoom = new AsyncCallbackJsonWebHandler("/api/zoom",
        [](AsyncWebServerRequest *req, JsonVariant &json) {
            JsonObject body = json.as<JsonObject>();
            setZoomTarget(body["position"] | 0);
            sendJsonOk(req, "zoom");
        });
    server.addHandler(zoom);

    auto *zoomProfile = new AsyncCallbackJsonWebHandler("/api/zoom/profile",
        [](AsyncWebServerRequest *req, JsonVariant &json) {
            JsonObject body = json.as<JsonObject>();
            setZoomProfile(body["vmax"] | ZOOM_VMAX_DEFAULT,
                           body["accel"] | ZOOM_ACCEL_DEFAULT);
            sendJsonOk(req, "zoom-profile");
        });
    server.addHandler(zoomProfile);

    // ESPAsyncWebServer has no path parameters, so /api/command/<name> is one
    // registration per allowed name (the same allow-list as dji_gimbal_web.py).
    for (const char *name : kCommandNames) {
        String uri = String("/api/command/") + name;
        server.on(uri.c_str(), HTTP_POST, [name](AsyncWebServerRequest *req) {
            execNamedCommand(name);
            sendJsonOk(req, name);
        });
    }

    // One-shot CAN ping: a lone 0x100 frame. Any other node on the bus (gimbal,
    // Canable, ...) ACKs it. TX success means the transceiver TX path + a peer.
    server.on("/api/can/probe", HTTP_POST, [](AsyncWebServerRequest *req) {
        uint8_t payload[1] = {0xA5};
        bool locked = g_canTxLock &&
                      xSemaphoreTake(g_canTxLock, pdMS_TO_TICKS(400)) == pdTRUE;
        bool ok = locked && canHwSend(0x100, payload, 1, 50);
        if (locked) xSemaphoreGive(g_canTxLock);
        if (ok) {
            g_can.txOk++;
        } else {
            g_can.txFail++;
        }
        JsonDocument doc;
        doc["ok"] = ok;
        fillCanStatusJson(doc["can"].to<JsonObject>());
        String out;
        serializeJson(doc, out);
        req->send(ok ? 200 : 503, "application/json", out);
    });
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------

static void setupWifi() {
    WiFiManager wm;
    // Uncomment to force the config portal on every boot while bringing this up:
    // wm.resetSettings();
    wm.setConfigPortalTimeout(180); // give up and retry later if nobody configures it
    if (!wm.autoConnect("DJI-Gimbal-Setup")) {
        Serial.println("[wifi] failed to connect and portal timed out; restarting");
        delay(3000);
        ESP.restart();
    }
    Serial.printf("[wifi] connected, IP = %s\n", WiFi.localIP().toString().c_str());
}

static bool g_sessionStarted = false;

static void setupCan() {
    g_canTxLock = xSemaphoreCreateMutex();
    g_can.started = canHwInit();
    xTaskCreate(canRxTask, "can_rx", 4096, nullptr, 5, nullptr);
    if (!g_can.started) {
        strncpy(g_s.lastError, "CAN controller failed to start", sizeof(g_s.lastError) - 1);
        return;
    }
    g_sessionStarted = true;
}

// Mirrors GimbalCanSession._run's startup: enable parameter push, then query the
// focus motor position so the zoom ramp seeds from the real motor position.
static void sessionStart() {
    if (!g_can.started) return;
    canSendPacket(dji::buildSetParameterPush(true));
    canSendPacket(dji::buildFocusGet());
}

void setup() {
    Serial.begin(115200);
    delay(200);

    g_stateLock = xSemaphoreCreateMutex();

#if GIMBAL_SLEEP_ENABLED
    // Every boot (cold power-on OR waking from deep sleep) counts as "gimbal present"
    // for timeout purposes — avoids racing straight back to sleep before the sense
    // pin has been read even once in loop().
    pinMode(GIMBAL_PRESENT_PIN, INPUT);
    g_lastGimbalPresentMillis = millis();
#endif

    if (!LittleFS.begin(true)) {
        Serial.println("[fs] LittleFS mount failed");
    }

    setupWifi();
    setupCan();
    sessionStart();

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    setupRestApi();
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
    server.begin();

    uint32_t now = millis();
    g_lastSpeedSendMs = now;
    g_lastZoomMs = now;
    g_lastAnglePollMs = now;
    g_lastBroadcastMs = now;

    Serial.println("[main] ready");
}

void loop() {
    canHwPollHealth();
    if (canHwStarted() && !g_sessionStarted) {
        g_can.started = true;
        g_sessionStarted = true;
        g_s.lastError[0] = '\0';
        sessionStart();
    }
    uint32_t now = millis();
    tickSpeed(now);
    if (now - g_lastZoomMs >= ZOOM_INTERVAL_MS) tickZoom(now);
    tickAnglePoll(now);
    broadcastState(now);

#if GIMBAL_SLEEP_ENABLED
    if (digitalRead(GIMBAL_PRESENT_PIN) == HIGH) {
        g_lastGimbalPresentMillis = now;
    } else if (now - g_lastGimbalPresentMillis >= OFF_TIMEOUT_MS) {
        enterDeepSleepUntilGimbalPresent(); // does not return
    }
#endif

    ws.cleanupClients();
}

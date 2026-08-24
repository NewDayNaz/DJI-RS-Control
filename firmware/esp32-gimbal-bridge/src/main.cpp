// ESP32 WiFi <-> CAN bridge for the DJI R SDK gimbal protocol (CmdSet 0x0E).
//
// - First boot (or after a long button-free WiFi failure) opens a "DJI-Gimbal-Setup"
//   captive portal (WiFiManager) so you configure WiFi without reflashing.
// - Serves a single-page joystick/telemetry UI from LittleFS at "/".
// - WebSocket at "/ws": browser -> device JSON commands (speed/recenter/selfie/
//   activetrack/focus/push), device -> browser JSON telemetry broadcasts.
// - REST fallback at "/api/*" for one-shot commands (see README.md).
// - Safety: a deadman watchdog zeroes gimbal speed if no "speed" command arrives
//   within SPEED_TIMEOUT_MS — WiFi drops must not leave the gimbal spinning.
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
#include "driver/rtc_io.h"
#include "esp_sleep.h"
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

// ---- Timing ----
static constexpr uint32_t SPEED_SEND_INTERVAL_MS = 50;   // 20 Hz outgoing speed commands
static constexpr uint32_t SPEED_TIMEOUT_MS = 250;        // deadman: zero speed if stale
static constexpr uint32_t TELEMETRY_BROADCAST_MS = 100;  // 10 Hz telemetry to browser
static constexpr uint32_t OFF_TIMEOUT_MS = 60000;        // deep-sleep after this long unplugged

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

dji::Reassembler g_reassembler;

// Desired speed, updated by the latest WS "speed" command; actually transmitted
// at a fixed cadence by sendSpeedIfDue() below (coalesces bursty WS input).
static float g_desiredYawDegS = 0.0f;
static float g_desiredRollDegS = 0.0f;
static float g_desiredPitchDegS = 0.0f;
static uint32_t g_lastSpeedCmdMillis = 0;
static uint32_t g_lastSpeedSendMillis = 0;
static bool g_speedActive = false; // true while a non-zero deflection is being driven

static uint32_t g_lastTelemetryBroadcastMillis = 0;
static dji::GimbalAngles g_lastAngles = {};
static bool g_haveAngles = false;
static volatile bool g_anglesDirty = false;

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
    if (!g_canTxLock) return false;
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

// Broadcasts current gimbal angles to all connected WS clients.
static void broadcastTelemetry(const dji::GimbalAngles &a) {
    if (ws.count() == 0) return;
    JsonDocument doc;
    doc["type"] = "telemetry";
    doc["yaw"] = a.yawDeg;
    doc["roll"] = a.rollDeg;
    doc["pitch"] = a.pitchDeg;
    doc["rssi"] = WiFi.RSSI();
    String out;
    serializeJson(doc, out);
    ws.textAll(out);
}

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
    if (g_haveAngles) {
        can["yaw"] = g_lastAngles.yawDeg;
        can["roll"] = g_lastAngles.rollDeg;
        can["pitch"] = g_lastAngles.pitchDeg;
    }

    const char *state = can["state"] | "unknown";
    const char *hint = "no traffic yet";
    if (!canHwStarted()) {
        hint = "CAN controller failed to start";
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
    if (reply.cmdSet != dji::CMD_SET_GIMBAL) return;

    if (reply.cmdId == dji::CMD_ID_PUSH_PARAMS || reply.cmdId == dji::CMD_ID_ANGLE) {
        dji::GimbalAngles angles;
        if (dji::parseGimbalAngles(reply, angles)) {
            g_lastAngles = angles;
            g_haveAngles = true;
            g_anglesDirty = true;
        }
    }
    // CMD_ID_CALIB_STATUS (0x10) and other pushes are not yet decoded — see
    // docs/DJI_R_SDK_Protocol.md for the reverse-engineering status.
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

// Sends the current desired speed at a fixed cadence; applies the deadman
// watchdog if no fresh "speed" command has arrived recently.
static void sendSpeedIfDue() {
    uint32_t now = millis();

    if (g_speedActive && (now - g_lastSpeedCmdMillis > SPEED_TIMEOUT_MS)) {
        g_desiredYawDegS = 0.0f;
        g_desiredRollDegS = 0.0f;
        g_desiredPitchDegS = 0.0f;
        g_speedActive = false;
        auto pkt = dji::buildControlSpeed(0, 0, 0);
        canSendPacket(pkt);
        g_lastSpeedSendMillis = now;
        Serial.println("[safety] speed watchdog: no WS update in time, zeroed gimbal speed");
        return;
    }

    if (!g_speedActive) return; // nothing to stream while idle
    if (now - g_lastSpeedSendMillis < SPEED_SEND_INTERVAL_MS) return;

    g_lastSpeedSendMillis = now;
    auto pkt = dji::buildControlSpeed(g_desiredYawDegS, g_desiredRollDegS, g_desiredPitchDegS);
    canSendPacket(pkt);
}

// ---------------------------------------------------------------------------
// Command handling (shared by WebSocket and REST)
// ---------------------------------------------------------------------------

static void handleSpeedCommand(float yaw, float roll, float pitch) {
    g_desiredYawDegS = yaw;
    g_desiredRollDegS = roll;
    g_desiredPitchDegS = pitch;
    g_lastSpeedCmdMillis = millis();
    g_speedActive = (yaw != 0.0f || roll != 0.0f || pitch != 0.0f);
    // Send immediately so the first deflection isn't delayed by the send cadence.
    if (millis() - g_lastSpeedSendMillis >= SPEED_SEND_INTERVAL_MS || !g_speedActive) {
        auto pkt = dji::buildControlSpeed(yaw, roll, pitch);
        canSendPacket(pkt);
        g_lastSpeedSendMillis = millis();
    }
}

static void handleJsonCommand(JsonDocument &doc) {
    const char *cmd = doc["cmd"] | "";
    if (strcmp(cmd, "speed") == 0) {
        handleSpeedCommand(doc["yaw"] | 0.0f, doc["roll"] | 0.0f, doc["pitch"] | 0.0f);
    } else if (strcmp(cmd, "recenter") == 0) {
        canSendPacket(dji::buildRecenterSelfie(0x01));
    } else if (strcmp(cmd, "selfie") == 0) {
        canSendPacket(dji::buildRecenterSelfie(0x02));
    } else if (strcmp(cmd, "activetrack") == 0) {
        canSendPacket(dji::buildActiveTrackToggle());
    } else if (strcmp(cmd, "focus") == 0) {
        int pos = constrain((int) (doc["position"] | 0), 0, 4096);
        canSendPacket(dji::buildFocusSet((uint16_t) pos));
    } else if (strcmp(cmd, "push") == 0) {
        bool enable = doc["enable"] | true;
        canSendPacket(dji::buildSetParameterPush(enable));
    } else {
        Serial.printf("[ws] unknown cmd: %s\n", cmd);
    }
}

// ---------------------------------------------------------------------------
// WebSocket event handling
// ---------------------------------------------------------------------------

static void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                       AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("[ws] client #%u connected from %s\n", client->id(),
                      client->remoteIP().toString().c_str());
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[ws] client #%u disconnected\n", client->id());
        // If nobody is connected any more, stop driving speed for safety.
        if (ws.count() == 0) {
            g_speedActive = false;
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
// REST API (one-shot commands; speed streaming should use the WebSocket)
// ---------------------------------------------------------------------------

static void sendJsonOk(AsyncWebServerRequest *req) {
    req->send(200, "application/json", "{\"ok\":true}");
}

static void setupRestApi() {
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

    server.on("/api/recenter", HTTP_POST, [](AsyncWebServerRequest *req) {
        bool ok = canSendPacket(dji::buildRecenterSelfie(0x01));
        req->send(ok ? 200 : 503, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
    });

    server.on("/api/selfie", HTTP_POST, [](AsyncWebServerRequest *req) {
        bool ok = canSendPacket(dji::buildRecenterSelfie(0x02));
        req->send(ok ? 200 : 503, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
    });

    server.on("/api/activetrack", HTTP_POST, [](AsyncWebServerRequest *req) {
        canSendPacket(dji::buildActiveTrackToggle());
        sendJsonOk(req);
    });

    server.on("/api/push", HTTP_POST, [](AsyncWebServerRequest *req) {
        bool enable = true;
        if (req->hasParam("enable")) {
            enable = req->getParam("enable")->value() != "0";
        }
        canSendPacket(dji::buildSetParameterPush(enable));
        sendJsonOk(req);
    });

    // /api/focus?position=0..4096
    server.on("/api/focus", HTTP_POST, [](AsyncWebServerRequest *req) {
        int pos = 0;
        if (req->hasParam("position")) {
            pos = req->getParam("position")->value().toInt();
        }
        pos = constrain(pos, 0, 4096);
        canSendPacket(dji::buildFocusSet((uint16_t) pos));
        sendJsonOk(req);
    });

    // /api/speed?yaw=..&roll=..&pitch=.. (one-shot; for continuous control use the WS)
    server.on("/api/speed", HTTP_POST, [](AsyncWebServerRequest *req) {
        float yaw = req->hasParam("yaw") ? req->getParam("yaw")->value().toFloat() : 0.0f;
        float roll = req->hasParam("roll") ? req->getParam("roll")->value().toFloat() : 0.0f;
        float pitch = req->hasParam("pitch") ? req->getParam("pitch")->value().toFloat() : 0.0f;
        handleSpeedCommand(yaw, roll, pitch);
        sendJsonOk(req);
    });

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

static void setupCan() {
    g_canTxLock = xSemaphoreCreateMutex();
    g_can.started = canHwInit();
    if (!g_can.started) return;
    xTaskCreate(canRxTask, "can_rx", 4096, nullptr, 5, nullptr);
}

void setup() {
    Serial.begin(115200);
    delay(200);

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

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    setupRestApi();
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
    server.begin();

    Serial.println("[main] ready");
}

void loop() {
    canHwPollHealth();
    sendSpeedIfDue();
    if (g_anglesDirty && g_haveAngles) {
        g_anglesDirty = false;
        uint32_t t = millis();
        if (t - g_lastTelemetryBroadcastMillis >= TELEMETRY_BROADCAST_MS) {
            g_lastTelemetryBroadcastMillis = t;
            broadcastTelemetry(g_lastAngles);
        }
    }

    // Do not periodic-TX push enable. Every 0x223 frame on this bus risks
    // bus-off, and a truncated push packet poisons the gimbal's 0x223 reassembler.
    // Enable push from the UI when telemetry is needed.

#if GIMBAL_SLEEP_ENABLED
    uint32_t now = millis();
    if (digitalRead(GIMBAL_PRESENT_PIN) == HIGH) {
        g_lastGimbalPresentMillis = now;
    } else if (now - g_lastGimbalPresentMillis >= OFF_TIMEOUT_MS) {
        enterDeepSleepUntilGimbalPresent(); // does not return
    }
#endif

    ws.cleanupClients();
}

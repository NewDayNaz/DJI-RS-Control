#include "can_hw.h"

#include <cstring>

#if !defined(CAN_BACKEND_MCP2515) && !defined(CAN_BACKEND_TWAI)
#error "Set CAN_BACKEND_MCP2515 or CAN_BACKEND_TWAI in platformio.ini"
#endif

#if defined(CAN_BACKEND_MCP2515) && defined(CAN_BACKEND_TWAI)
#error "Pick exactly one CAN backend"
#endif

static SemaphoreHandle_t g_lock = nullptr;
static TaskHandle_t g_rxWaiter = nullptr;
static bool g_started = false;
static uint32_t g_busOffEvents = 0;
static uint32_t g_rxOverflow = 0;
static uint32_t g_lastLogMs = 0;
static const char *g_state = "stopped";

bool canHwStarted() { return g_started; }
uint32_t canHwBusOffEvents() { return g_busOffEvents; }
uint32_t canHwRxOverflow() { return g_rxOverflow; }
const char *canHwStateName() { return g_state; }

static void lockCreate() {
    if (!g_lock) g_lock = xSemaphoreCreateMutex();
}

static bool lockTake(uint32_t timeoutMs) {
    if (!g_lock) return false;
    return xSemaphoreTake(g_lock, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

static void lockGive() {
    if (g_lock) xSemaphoreGive(g_lock);
}

#if defined(CAN_BACKEND_MCP2515)

#include <SPI.h>
#include <mcp2515.h>

#ifndef MCP2515_CS_PIN
#define MCP2515_CS_PIN D7
#endif
#ifndef MCP2515_INT_PIN
#define MCP2515_INT_PIN D6
#endif
#ifndef MCP2515_SCK_PIN
#define MCP2515_SCK_PIN D8
#endif
#ifndef MCP2515_MISO_PIN
#define MCP2515_MISO_PIN D9
#endif
#ifndef MCP2515_MOSI_PIN
#define MCP2515_MOSI_PIN D10
#endif
#ifndef MCP2515_CLOCK_MHZ
#define MCP2515_CLOCK_MHZ 16
#endif
#ifndef CAN_MCP_ACCEPT_ALL
#define CAN_MCP_ACCEPT_ALL 0
#endif

// MCP2515 SPI is rated Fosc/2. 8 MHz stays in spec for a 16 MHz crystal.
static constexpr uint32_t kSpiHz = 8000000;

static MCP2515 g_mcp(MCP2515_CS_PIN, kSpiHz, &SPI);
static uint8_t g_cachedTec = 0;
static uint8_t g_cachedRec = 0;
static uint8_t g_cachedEflg = 0;

#if MCP2515_CLOCK_MHZ == 8
static constexpr CAN_CLOCK kClock = MCP_8MHZ;
#elif MCP2515_CLOCK_MHZ == 20
static constexpr CAN_CLOCK kClock = MCP_20MHZ;
#else
static constexpr CAN_CLOCK kClock = MCP_16MHZ;
#endif

const char *canHwBackendName() { return "mcp2515"; }

static void IRAM_ATTR mcpIntIsr() {
    BaseType_t woken = pdFALSE;
    if (g_rxWaiter) {
        vTaskNotifyGiveFromISR(g_rxWaiter, &woken);
    }
#if defined(ARDUINO_ARCH_ESP32)
    if (woken) portYIELD_FROM_ISR();
#endif
}

static bool mcpApplyConfig() {
    if (g_mcp.setBitrate(CAN_1000KBPS, kClock) != MCP2515::ERROR_OK) {
        Serial.println("[can] MCP2515 setBitrate(1 Mbps) failed — check crystal marking");
        return false;
    }

#if CAN_MCP_ACCEPT_ALL
    // Mask 0 = accept every 11-bit ID. Only for debug: the MCP2515 has two RX
    // buffers, and the accessory bus runs ~400 Hz of 0x530. The RX task must
    // drain continuously or frames (including 0x222) get overwritten.
    g_mcp.setFilterMask(MCP2515::MASK0, false, 0);
    g_mcp.setFilterMask(MCP2515::MASK1, false, 0);
    Serial.println("[can] MCP2515 filter=accept_all");
#else
    // Hardware-filter to gimbal -> host only. Everything else on this wire
    // (0x530/0x531/0x426) stays out of the two RX buffers.
    const uint32_t kRxId = 0x222;
    g_mcp.setFilterMask(MCP2515::MASK0, false, 0x7FF);
    g_mcp.setFilterMask(MCP2515::MASK1, false, 0x7FF);
    g_mcp.setFilter(MCP2515::RXF0, false, kRxId);
    g_mcp.setFilter(MCP2515::RXF1, false, kRxId);
    g_mcp.setFilter(MCP2515::RXF2, false, kRxId);
    g_mcp.setFilter(MCP2515::RXF3, false, kRxId);
    g_mcp.setFilter(MCP2515::RXF4, false, kRxId);
    g_mcp.setFilter(MCP2515::RXF5, false, kRxId);
    Serial.println("[can] MCP2515 filter=0x222");
#endif

    if (g_mcp.setNormalMode() != MCP2515::ERROR_OK) {
        Serial.println("[can] MCP2515 setNormalMode failed");
        return false;
    }
    g_state = "running";
    return true;
}

static void mcpRefreshCounters() {
    g_cachedTec = g_mcp.errorCountTX();
    g_cachedRec = g_mcp.errorCountRX();
    g_cachedEflg = g_mcp.getErrorFlags();
    if (g_cachedEflg & MCP2515::EFLG_RX0OVR || g_cachedEflg & MCP2515::EFLG_RX1OVR) {
        g_rxOverflow++;
        g_mcp.clearRXnOVR();
    }
}

bool canHwInit() {
    lockCreate();

    pinMode(MCP2515_CS_PIN, OUTPUT);
    digitalWrite(MCP2515_CS_PIN, HIGH);
    SPI.begin(MCP2515_SCK_PIN, MCP2515_MISO_PIN, MCP2515_MOSI_PIN, MCP2515_CS_PIN);

    if (g_mcp.reset() != MCP2515::ERROR_OK) {
        Serial.println("[can] MCP2515 reset failed — CS/SPI wiring or no hat?");
        g_state = "fail_init";
        return false;
    }
    if (!mcpApplyConfig()) {
        g_state = "fail_init";
        return false;
    }

    if ((int) MCP2515_INT_PIN >= 0) {
        pinMode(MCP2515_INT_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(MCP2515_INT_PIN), mcpIntIsr, FALLING);
    }

    g_started = true;
    Serial.printf("[can] MCP2515 started at 1 Mbps  CS=%d INT=%d SPI=%d/%d/%d  xtal=%d MHz\n",
                  (int) MCP2515_CS_PIN, (int) MCP2515_INT_PIN,
                  (int) MCP2515_SCK_PIN, (int) MCP2515_MISO_PIN, (int) MCP2515_MOSI_PIN,
                  MCP2515_CLOCK_MHZ);
    return true;
}

bool canHwHealthy() {
    if (!g_started) return false;
    if (!lockTake(20)) return false;
    mcpRefreshCounters();
    uint8_t eflg = g_cachedEflg;
    uint8_t tec = g_cachedTec;
    lockGive();
    if (eflg & MCP2515::EFLG_TXBO) return false;
    return tec < 96;
}

bool canHwSend(uint32_t id, const uint8_t *data, uint8_t len, uint32_t timeoutMs) {
    if (!g_started || len > 8) return false;
    struct can_frame frame = {};
    frame.can_id = id & 0x7FFu;
    frame.can_dlc = len;
    if (len) memcpy(frame.data, data, len);

    uint32_t start = millis();
    do {
        if (!lockTake(timeoutMs)) return false;
        MCP2515::ERROR err = g_mcp.sendMessage(&frame);
        mcpRefreshCounters();
        bool busOff = g_cachedEflg & MCP2515::EFLG_TXBO;
        lockGive();
        if (err == MCP2515::ERROR_OK) return true;
        if (busOff) return false;
        vTaskDelay(pdMS_TO_TICKS(5));
    } while (millis() - start < timeoutMs);
    return false;
}

bool canHwReceive(CanHwFrame &out, uint32_t timeoutMs) {
    if (!g_started) {
        vTaskDelay(pdMS_TO_TICKS(timeoutMs ? timeoutMs : 10));
        return false;
    }
    if (g_rxWaiter == nullptr) g_rxWaiter = xTaskGetCurrentTaskHandle();

    uint32_t start = millis();
    for (;;) {
        if (lockTake(timeoutMs ? timeoutMs : 5)) {
            struct can_frame frame = {};
            MCP2515::ERROR err = g_mcp.readMessage(&frame);
            lockGive();
            if (err == MCP2515::ERROR_OK) {
                out.id = frame.can_id & 0x7FFu;
                out.extd = (frame.can_id & CAN_EFF_FLAG) != 0;
                out.dlc = frame.can_dlc > 8 ? 8 : frame.can_dlc;
                memset(out.data, 0, sizeof(out.data));
                memcpy(out.data, frame.data, out.dlc);
                return true;
            }
        }
        if (timeoutMs == 0) return false;
        uint32_t elapsed = millis() - start;
        if (elapsed >= timeoutMs) return false;
        uint32_t remain = timeoutMs - elapsed;
        uint32_t wait = remain < 5 ? remain : 5;
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait));
    }
}

void canHwPollHealth() {
    if (!g_started) return;
    if (!lockTake(20)) return;
    mcpRefreshCounters();
    uint8_t eflg = g_cachedEflg;
    uint8_t tec = g_cachedTec;
    uint8_t rec = g_cachedRec;
    bool busOff = eflg & MCP2515::EFLG_TXBO;
    if (busOff) {
        g_busOffEvents++;
        g_state = "bus_off";
        Serial.println("[can] BUS-OFF (no ACK). Check CANH/CANL/GND, P1 terminator, "
                       "gimbal CAN/S-BUS switch. Resetting MCP2515.");
        g_mcp.reset();
        if (mcpApplyConfig()) {
            Serial.println("[can] MCP2515 recovered");
        } else {
            Serial.println("[can] MCP2515 recover failed");
            g_state = "fail_init";
        }
        mcpRefreshCounters();
        eflg = g_cachedEflg;
        tec = g_cachedTec;
        rec = g_cachedRec;
    } else if (eflg & MCP2515::EFLG_TXEP) {
        g_state = "error_passive";
    } else if (g_started) {
        g_state = "running";
    }
    lockGive();

    uint32_t now = millis();
    if (now - g_lastLogMs < 2000) return;
    g_lastLogMs = now;
    Serial.printf("[can] backend=mcp2515 state=%s tec=%u rec=%u eflg=0x%02X "
                  "bus_off_ev=%lu rx_ovr=%lu\n",
                  g_state, tec, rec, eflg,
                  (unsigned long) g_busOffEvents, (unsigned long) g_rxOverflow);
}

void canHwFillStatus(JsonObject can) {
    can["backend"] = canHwBackendName();
    can["started"] = g_started;
    can["state"] = g_state;
    can["cs_pin"] = (int) MCP2515_CS_PIN;
    can["int_pin"] = (int) MCP2515_INT_PIN;
    can["sck_pin"] = (int) MCP2515_SCK_PIN;
    can["miso_pin"] = (int) MCP2515_MISO_PIN;
    can["mosi_pin"] = (int) MCP2515_MOSI_PIN;
    can["clock_mhz"] = MCP2515_CLOCK_MHZ;
    can["filter"] = CAN_MCP_ACCEPT_ALL ? "accept_all" : "0x222";
    can["tec"] = g_cachedTec;
    can["rec"] = g_cachedRec;
    can["eflg"] = g_cachedEflg;
    can["bus_errors"] = 0;
    can["bus_off_events"] = g_busOffEvents;
    can["rx_missed"] = g_rxOverflow;
    can["rx_queued"] = 0;
}

#else // CAN_BACKEND_TWAI

#include "driver/twai.h"

#ifndef CAN_TX_GPIO_NUM
#define CAN_TX_GPIO_NUM 6
#endif
#ifndef CAN_RX_GPIO_NUM
#define CAN_RX_GPIO_NUM 7
#endif

static constexpr gpio_num_t kTxPin = (gpio_num_t) CAN_TX_GPIO_NUM;
static constexpr gpio_num_t kRxPin = (gpio_num_t) CAN_RX_GPIO_NUM;

const char *canHwBackendName() { return "twai"; }

static const char *twaiStateName(twai_state_t s) {
    switch (s) {
        case TWAI_STATE_STOPPED: return "stopped";
        case TWAI_STATE_RUNNING: return "running";
        case TWAI_STATE_BUS_OFF: return "bus_off";
        case TWAI_STATE_RECOVERING: return "recovering";
        default: return "unknown";
    }
}

bool canHwInit() {
    lockCreate();
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(kTxPin, kRxPin, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 128;
    g_config.tx_queue_len = 8;
    g_config.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                              TWAI_ALERT_ERR_PASS | TWAI_ALERT_ABOVE_ERR_WARN |
                              TWAI_ALERT_RX_QUEUE_FULL;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[can] TWAI driver install failed");
        g_state = "fail_init";
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[can] TWAI start failed");
        g_state = "fail_init";
        return false;
    }
    g_started = true;
    g_state = "running";
    Serial.printf("[can] TWAI started at 1 Mbps  TX=GPIO%d RX=GPIO%d  filter=accept_all\n",
                  (int) kTxPin, (int) kRxPin);
    return true;
}

bool canHwHealthy() {
    twai_status_info_t st = {};
    if (twai_get_status_info(&st) != ESP_OK) return false;
    g_state = twaiStateName(st.state);
    return st.state == TWAI_STATE_RUNNING && st.tx_error_counter < 96;
}

bool canHwSend(uint32_t id, const uint8_t *data, uint8_t len, uint32_t timeoutMs) {
    if (!g_started || len > 8) return false;
    twai_message_t msg = {};
    msg.identifier = id;
    msg.data_length_code = len;
    msg.extd = 0;
    msg.rtr = 0;
    if (len) memcpy(msg.data, data, len);
    return twai_transmit(&msg, pdMS_TO_TICKS(timeoutMs)) == ESP_OK;
}

bool canHwReceive(CanHwFrame &out, uint32_t timeoutMs) {
    twai_message_t rx = {};
    if (twai_receive(&rx, pdMS_TO_TICKS(timeoutMs)) != ESP_OK) return false;
    out.id = rx.identifier;
    out.dlc = rx.data_length_code > 8 ? 8 : rx.data_length_code;
    out.extd = rx.extd;
    memset(out.data, 0, sizeof(out.data));
    memcpy(out.data, rx.data, out.dlc);
    return true;
}

void canHwPollHealth() {
    uint32_t alerts = 0;
    if (twai_read_alerts(&alerts, 0) == ESP_OK) {
        if (alerts & TWAI_ALERT_BUS_OFF) {
            g_busOffEvents++;
            Serial.println("[can] BUS-OFF (no ACK). Check CTX/CRX swap, GND, 120R, "
                           "gimbal CAN/S-BUS switch. Recovering.");
            twai_initiate_recovery();
        }
        if (alerts & TWAI_ALERT_BUS_RECOVERED) {
            if (twai_start() == ESP_OK) {
                Serial.println("[can] recovered, TWAI restarted");
            } else {
                Serial.println("[can] recovered but twai_start failed");
            }
        }
        if (alerts & TWAI_ALERT_ERR_PASS) {
            Serial.println("[can] error-passive (still no ACKs)");
        }
        if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
            Serial.println("[can] error warning limit");
        }
        if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
            g_rxOverflow++;
            Serial.println("[can] RX queue full — frame dropped");
        }
    }

    twai_status_info_t st = {};
    if (twai_get_status_info(&st) == ESP_OK) {
        g_state = twaiStateName(st.state);
        if (st.state == TWAI_STATE_BUS_OFF) twai_initiate_recovery();
    }

    uint32_t now = millis();
    if (now - g_lastLogMs < 2000) return;
    g_lastLogMs = now;
    if (twai_get_status_info(&st) != ESP_OK) return;
    Serial.printf("[can] backend=twai state=%s tec=%lu rec=%lu bus_err=%lu "
                  "bus_off_ev=%lu rx_missed=%lu qfull=%lu queued=%lu rx_gpio=%d\n",
                  twaiStateName(st.state),
                  (unsigned long) st.tx_error_counter,
                  (unsigned long) st.rx_error_counter,
                  (unsigned long) st.bus_error_count,
                  (unsigned long) g_busOffEvents,
                  (unsigned long) st.rx_missed_count,
                  (unsigned long) g_rxOverflow,
                  (unsigned long) st.msgs_to_rx,
                  digitalRead(kRxPin));
}

void canHwFillStatus(JsonObject can) {
    twai_status_info_t st = {};
    twai_get_status_info(&st);
    can["backend"] = canHwBackendName();
    can["started"] = g_started;
    can["state"] = twaiStateName(st.state);
    can["tx_pin"] = (int) kTxPin;
    can["rx_pin"] = (int) kRxPin;
    can["rx_gpio"] = digitalRead(kRxPin);
    can["tec"] = st.tx_error_counter;
    can["rec"] = st.rx_error_counter;
    can["bus_errors"] = st.bus_error_count;
    can["bus_off_events"] = g_busOffEvents;
    can["rx_missed"] = st.rx_missed_count;
    can["rx_queued"] = st.msgs_to_rx;
    can["filter"] = "accept_all";
}

#endif

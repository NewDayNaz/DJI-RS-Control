#include "can_hw.h"

#include <SPI.h>
#include <mcp2515.h>
#include <cstring>
#include "driver/gpio.h"
#include "freertos/queue.h"

static SemaphoreHandle_t g_lock = nullptr;
static QueueHandle_t g_rxq = nullptr;
static TaskHandle_t g_rxWaiter = nullptr;
static bool g_started = false;
static bool g_intAttached = false;
static uint32_t g_busOffEvents = 0;
static uint32_t g_rxOverflow = 0;
static uint32_t g_lastLogMs = 0;
static uint32_t g_lastInitTryMs = 0;
static const char *g_state = "stopped";
static char g_initError[96] = "";
static uint8_t g_canstat = 0;
static uint8_t g_canctrl = 0;

bool canHwStarted() { return g_started; }
uint32_t canHwBusOffEvents() { return g_busOffEvents; }
uint32_t canHwRxOverflow() { return g_rxOverflow; }
const char *canHwStateName() { return g_state; }

static void lockCreate() {
    if (!g_lock) g_lock = xSemaphoreCreateMutex();
    if (!g_rxq) g_rxq = xQueueCreate(32, sizeof(CanHwFrame));
}

static bool lockTake(uint32_t timeoutMs) {
    if (!g_lock) return false;
    return xSemaphoreTake(g_lock, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

static void lockGive() {
    if (g_lock) xSemaphoreGive(g_lock);
}

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

// MCP2515 SPI max is Fosc/2. 8 MHz is in spec for the 16 MHz hat crystal and
// drains the two RX buffers faster so 0x222 fragments are less likely to drop.
static constexpr uint32_t kSpiHz = 8000000;
static constexpr uint8_t kMcpInstrWrite = 0x02;
static constexpr uint8_t kMcpInstrRead = 0x03;
static constexpr uint8_t kMcpInstrReset = 0xC0;
static constexpr uint8_t kMcpCnf3 = 0x28;
static constexpr uint8_t kMcpCnf2 = 0x29;
static constexpr uint8_t kMcpCnf1 = 0x2A;
static constexpr uint8_t kMcpCanstat = 0x0E;
static constexpr uint8_t kMcpCanctrl = 0x0F;
static constexpr uint8_t kMcpOpModeMask = 0xE0;
static constexpr uint8_t kMcpOpConfig = 0x80;

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

static void mcpSetInitError(const char *msg) {
    strncpy(g_initError, msg, sizeof(g_initError) - 1);
    g_initError[sizeof(g_initError) - 1] = '\0';
}

static uint8_t mcpReadReg(uint8_t reg) {
    SPI.beginTransaction(SPISettings(kSpiHz, MSBFIRST, SPI_MODE0));
    digitalWrite(MCP2515_CS_PIN, LOW);
    SPI.transfer(kMcpInstrRead);
    SPI.transfer(reg);
    uint8_t value = SPI.transfer(0x00);
    digitalWrite(MCP2515_CS_PIN, HIGH);
    SPI.endTransaction();
    return value;
}

static void mcpWriteReg(uint8_t reg, uint8_t value) {
    SPI.beginTransaction(SPISettings(kSpiHz, MSBFIRST, SPI_MODE0));
    digitalWrite(MCP2515_CS_PIN, LOW);
    SPI.transfer(kMcpInstrWrite);
    SPI.transfer(reg);
    SPI.transfer(value);
    digitalWrite(MCP2515_CS_PIN, HIGH);
    SPI.endTransaction();
}

static void mcpRefreshIds() {
    g_canstat = mcpReadReg(kMcpCanstat);
    g_canctrl = mcpReadReg(kMcpCanctrl);
}

static bool mcpApplyConfig() {
    if (g_mcp.setBitrate(CAN_1000KBPS, kClock) != MCP2515::ERROR_OK) {
        mcpRefreshIds();
        mcpSetInitError("setBitrate(1 Mbps) failed — crystal/SPI");
        Serial.printf("[can] MCP2515 setBitrate failed  CANSTAT=0x%02X CANCTRL=0x%02X\n",
                      g_canstat, g_canctrl);
        return false;
    }
    // autowp's 16 MHz / 1 Mbps preset is 8 TQ, 62.5% sample, triple-sample
    // (CNF 00/D0/82). That is unlike a CANable (typically ~75–80% SP, no
    // triple-sample) and is why TEC climbed to error-passive after a few
    // seconds on this bus. Same 1 Mbps; later sample point, SAM=0, SJW=2.
    mcpWriteReg(kMcpCnf1, 0x40);
    mcpWriteReg(kMcpCnf2, 0x98);
    mcpWriteReg(kMcpCnf3, 0x01);

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
        mcpRefreshIds();
        mcpSetInitError("setNormalMode failed");
        Serial.printf("[can] MCP2515 setNormalMode failed  CANSTAT=0x%02X CANCTRL=0x%02X\n",
                      g_canstat, g_canctrl);
        return false;
    }
    g_initError[0] = '\0';
    g_state = "running";
    mcpRefreshIds();
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

// MCP2515 has two RX buffers. TX and health share the same SPI lock, so drain
// both into a queue whenever we already hold it — same packets as the Canable,
// just don't let the hat overwrite 0x222 fragments.
static void mcpDrainRxLocked() {
    struct can_frame frame = {};
    while (g_mcp.readMessage(&frame) == MCP2515::ERROR_OK) {
        CanHwFrame out;
        out.id = frame.can_id & 0x7FFu;
        out.extd = (frame.can_id & CAN_EFF_FLAG) != 0;
        out.dlc = frame.can_dlc > 8 ? 8 : frame.can_dlc;
        memset(out.data, 0, sizeof(out.data));
        memcpy(out.data, frame.data, out.dlc);
        if (!g_rxq || xQueueSend(g_rxq, &out, 0) != pdTRUE) {
            g_rxOverflow++;
        }
    }
    mcpRefreshCounters();
}

bool canHwInit() {
    lockCreate();
    g_started = false;
    g_state = "fail_init";

    // ESP32-C3 boot UART0 is GPIO20/21 — the same pins the hat uses for CS/INT.
    gpio_reset_pin((gpio_num_t) MCP2515_CS_PIN);
    gpio_reset_pin((gpio_num_t) MCP2515_INT_PIN);

    pinMode(MCP2515_CS_PIN, OUTPUT);
    digitalWrite(MCP2515_CS_PIN, HIGH);
    // Pass SS=-1 so the ESP32 SPI peripheral never owns CS. The MCP2515 library
    // bit-bangs CS around multi-byte commands; hardware CS would pulse between bytes.
    if (!SPI.begin(MCP2515_SCK_PIN, MCP2515_MISO_PIN, MCP2515_MOSI_PIN, -1)) {
        mcpSetInitError("SPI.begin failed");
        Serial.println("[can] SPI.begin failed");
        return false;
    }
    SPI.setHwCs(false);

    SPI.beginTransaction(SPISettings(kSpiHz, MSBFIRST, SPI_MODE0));
    digitalWrite(MCP2515_CS_PIN, LOW);
    SPI.transfer(kMcpInstrReset);
    digitalWrite(MCP2515_CS_PIN, HIGH);
    SPI.endTransaction();
    delay(10);
    mcpRefreshIds();
    if ((g_canstat & kMcpOpModeMask) != kMcpOpConfig) {
        snprintf(g_initError, sizeof(g_initError),
                 "SPI no response CANSTAT=0x%02X CANCTRL=0x%02X — seat the hat",
                 g_canstat, g_canctrl);
        Serial.printf("[can] MCP2515 not in config after RESET  CANSTAT=0x%02X CANCTRL=0x%02X "
                      "(expect CANSTAT opmode 0x80). Hat not seated, or CS/MISO/SCK.\n",
                      g_canstat, g_canctrl);
        return false;
    }

    if (g_mcp.reset() != MCP2515::ERROR_OK) {
        mcpRefreshIds();
        snprintf(g_initError, sizeof(g_initError),
                 "reset failed CANSTAT=0x%02X CANCTRL=0x%02X", g_canstat, g_canctrl);
        Serial.printf("[can] MCP2515 reset failed  CANSTAT=0x%02X CANCTRL=0x%02X\n",
                      g_canstat, g_canctrl);
        return false;
    }
    if (!mcpApplyConfig()) {
        return false;
    }

    if ((int) MCP2515_INT_PIN >= 0 && !g_intAttached) {
        pinMode(MCP2515_INT_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(MCP2515_INT_PIN), mcpIntIsr, FALLING);
        g_intAttached = true;
    }

    g_started = true;
    g_state = "running";
    Serial.printf("[can] MCP2515 started at 1 Mbps  CS=%d INT=%d SPI=%d/%d/%d  xtal=%d MHz  "
                  "CANSTAT=0x%02X\n",
                  (int) MCP2515_CS_PIN, (int) MCP2515_INT_PIN,
                  (int) MCP2515_SCK_PIN, (int) MCP2515_MISO_PIN, (int) MCP2515_MOSI_PIN,
                  MCP2515_CLOCK_MHZ, g_canstat);
    return true;
}

bool canHwHealthy() {
    if (!g_started) return false;
    if (!lockTake(20)) return false;
    mcpDrainRxLocked();
    uint8_t eflg = g_cachedEflg;
    lockGive();
    return (eflg & MCP2515::EFLG_TXBO) == 0;
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
        mcpDrainRxLocked();
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

    if (g_rxq && xQueueReceive(g_rxq, &out, 0) == pdTRUE) return true;

    uint32_t start = millis();
    for (;;) {
        if (lockTake(timeoutMs ? 5 : 0)) {
            mcpDrainRxLocked();
            lockGive();
        }
        if (g_rxq && xQueueReceive(g_rxq, &out, 0) == pdTRUE) return true;
        if (timeoutMs == 0) return false;
        uint32_t elapsed = millis() - start;
        if (elapsed >= timeoutMs) return false;
        uint32_t remain = timeoutMs - elapsed;
        uint32_t wait = remain < 5 ? remain : 5;
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait));
    }
}

void canHwPollHealth() {
    if (!g_started) {
        uint32_t now = millis();
        if (now - g_lastInitTryMs >= 2000) {
            g_lastInitTryMs = now;
            canHwInit();
        }
        return;
    }
    if (!lockTake(20)) return;
    mcpDrainRxLocked();
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
    can["rx_queued"] = g_rxq ? (int) uxQueueMessagesWaiting(g_rxq) : 0;
    can["canstat"] = g_canstat;
    can["canctrl"] = g_canctrl;
    if (g_initError[0]) can["init_error"] = g_initError;
}

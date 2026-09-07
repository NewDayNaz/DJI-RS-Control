#include "ptz_bridge.h"

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <cmath>
#include <cstring>

// ---------------------------------------------------------------------------
// Ports (the ones hardware controllers actually have in their camera profiles)
// ---------------------------------------------------------------------------
static constexpr uint16_t kViscaIpUdp = 52381;
static constexpr uint16_t kViscaRawUdp = 1259;
static constexpr uint16_t kViscaRawTcp = 5678;
static constexpr uint16_t kPelcoPort = 4000;
static constexpr uint16_t kPanasonicUdp = 49152;

static constexpr float kMaxRateDefault = 30.0f;
static constexpr float kMaxRateMin = 5.0f;
static constexpr float kMaxRateMax = 120.0f;
static constexpr int kZoomMax = 4096;
static constexpr int kPresetCount = 16;
static constexpr int kTcpClients = 2;
static constexpr size_t kTcpBuf = 64;

static PtzSink g_sink = {};
static float g_maxRate = kMaxRateDefault;
static bool g_invertPan = false;
static bool g_invertTilt = false;

struct Preset {
    bool set = false;
    float yaw = 0, roll = 0, pitch = 0;
};
static Preset g_presets[kPresetCount];

static uint32_t g_nViscaUdp = 0;
static uint32_t g_nViscaTcp = 0;
static uint32_t g_nPelco = 0;
static uint32_t g_nAw = 0;
static uint32_t g_nCgi = 0;
static char g_lastCmd[48] = "";
static char g_lastFrom[20] = "";
static uint32_t g_lastMs = 0;

static WiFiUDP g_udpViscaIp;
static WiFiUDP g_udpViscaRaw;
static WiFiUDP g_udpPelco;
static WiFiUDP g_udpAw;
static WiFiServer g_tcpVisca(kViscaRawTcp);
static WiFiServer g_tcpPelco(kPelcoPort);

struct TcpSlot {
    WiFiClient client;
    uint8_t buf[kTcpBuf];
    size_t len = 0;
};
static TcpSlot g_viscaCli[kTcpClients];
static TcpSlot g_pelcoCli[kTcpClients];

static void setLastCmd(const char *cmd) {
    strncpy(g_lastCmd, cmd, sizeof(g_lastCmd) - 1);
    g_lastCmd[sizeof(g_lastCmd) - 1] = '\0';
    g_lastMs = millis();
}

static void noteCmd(const char *cmd) { setLastCmd(cmd); }

static void noteCmd(const char *cmd, const IPAddress &from) {
    setLastCmd(cmd);
    snprintf(g_lastFrom, sizeof(g_lastFrom), "%u.%u.%u.%u",
             from[0], from[1], from[2], from[3]);
}

static void noteCmdHost(const char *cmd, const char *from) {
    strncpy(g_lastCmd, cmd, sizeof(g_lastCmd) - 1);
    g_lastCmd[sizeof(g_lastCmd) - 1] = '\0';
    strncpy(g_lastFrom, from, sizeof(g_lastFrom) - 1);
    g_lastFrom[sizeof(g_lastFrom) - 1] = '\0';
    g_lastMs = millis();
}

// ---------------------------------------------------------------------------
// Axis helpers
// ---------------------------------------------------------------------------

static float applyPan(float yawDps) { return g_invertPan ? -yawDps : yawDps; }
static float applyTilt(float pitchDps) { return g_invertTilt ? -pitchDps : pitchDps; }

static float viscaPanDps(uint8_t vv) {
    if (vv == 0) return 0.0f;
    float n = (float) vv / 24.0f;
    if (n > 1.0f) n = 1.0f;
    return n * g_maxRate;
}

static float viscaTiltDps(uint8_t ww) {
    if (ww == 0) return 0.0f;
    float n = (float) ww / 20.0f;
    if (n > 1.0f) n = 1.0f;
    return n * g_maxRate;
}

static float pelcoDps(uint8_t data) {
    if (data == 0xFF) return g_maxRate;
    float n = (float) (data > 0x3F ? 0x3F : data) / 63.0f;
    return n * g_maxRate;
}

static float awNorm(int v) {
    // Panasonic 01-99, 50 = stop.
    if (v < 1) v = 1;
    if (v > 99) v = 99;
    return (v - 50) / 49.0f;
}

static uint16_t viscaNibbles4(const uint8_t *p) {
    return (uint16_t) (((p[0] & 0x0F) << 12) | ((p[1] & 0x0F) << 8) |
                       ((p[2] & 0x0F) << 4) | (p[3] & 0x0F));
}

static void putNibbles4(uint8_t *p, uint16_t v) {
    p[0] = (v >> 12) & 0x0F;
    p[1] = (v >> 8) & 0x0F;
    p[2] = (v >> 4) & 0x0F;
    p[3] = v & 0x0F;
}

static int viscaToZoom(uint16_t v) {
    if (v > 0x4000) v = 0x4000;
    return (int) ((uint32_t) v * kZoomMax / 0x4000);
}

static uint16_t zoomToVisca(int z) {
    if (z < 0) z = 0;
    if (z > kZoomMax) z = kZoomMax;
    return (uint16_t) ((uint32_t) z * 0x4000 / kZoomMax);
}

static float moveTimeS(float yaw, float pitch) {
    float cy = 0, cr = 0, cp = 0;
    if (g_sink.getAttitude && g_sink.getAttitude(&cy, &cr, &cp)) {
        float dy = fabsf(yaw - cy);
        float dp = fabsf(pitch - cp);
        float dist = dy > dp ? dy : dp;
        float t = dist / fmaxf(8.0f, g_maxRate);
        return fmaxf(0.4f, fminf(8.0f, t));
    }
    return 1.0f;
}

static void driveSpeed(float yaw, float pitch) {
    yaw = applyPan(yaw);
    pitch = applyTilt(pitch);
    if (fabsf(yaw) < 0.05f && fabsf(pitch) < 0.05f) {
        if (g_sink.stop) g_sink.stop();
        return;
    }
    if (g_sink.speed) g_sink.speed(yaw, 0.0f, pitch);
}

static void goPosition(float yaw, float pitch) {
    yaw = applyPan(yaw);
    pitch = applyTilt(pitch);
    float roll = 0.0f, cy = 0, cp = 0;
    if (g_sink.getAttitude) g_sink.getAttitude(&cy, &roll, &cp);
    if (g_sink.position) g_sink.position(yaw, roll, pitch, moveTimeS(yaw, pitch));
}

static void recallPreset(int idx) {
    if (idx < 0 || idx >= kPresetCount) {
        if (g_sink.home) g_sink.home();
        return;
    }
    if (g_presets[idx].set && g_sink.position) {
        const Preset &p = g_presets[idx];
        g_sink.position(p.yaw, p.roll, p.pitch, moveTimeS(p.yaw, p.pitch));
    } else if (g_sink.home) {
        g_sink.home();
    }
}

static void savePreset(int idx) {
    if (idx < 0 || idx >= kPresetCount) return;
    float y = 0, r = 0, p = 0;
    if (g_sink.getAttitude && g_sink.getAttitude(&y, &r, &p)) {
        g_presets[idx].set = true;
        g_presets[idx].yaw = y;
        g_presets[idx].roll = r;
        g_presets[idx].pitch = p;
    }
}

static float viscaZoomRate(uint8_t p, int sign) {
    // p = 0..7 → 0.125..1.0; standard tele/wide uses mid speed.
    float mag = (p > 7) ? 1.0f : (p + 1) / 8.0f;
    return sign * mag;
}

// ---------------------------------------------------------------------------
// VISCA
// ---------------------------------------------------------------------------

struct ViscaOut {
    uint8_t data[24];
    uint8_t len = 0;
    bool sendAck = false;      // ACK + completion
    bool sendReply = false;    // inquiry / IF_Clear (no ACK)
    bool syntax = false;
};

static void viscaCopy(ViscaOut &o, const uint8_t *b, uint8_t n) {
    if (n > sizeof(o.data)) n = (uint8_t) sizeof(o.data);
    memcpy(o.data, b, n);
    o.len = n;
    o.sendReply = true;
}

static void handleVisca(const uint8_t *m, size_t n, ViscaOut &out) {
    out = {};
    if (n < 3 || (m[0] & 0xF0) != 0x80 || m[n - 1] != 0xFF) {
        out.syntax = true;
        return;
    }
    uint8_t type = m[1];

    // IF_Clear  8x 01 00 01 FF  →  90 50 FF
    if (n >= 5 && type == 0x01 && m[2] == 0x00 && m[3] == 0x01) {
        const uint8_t r[] = {0x90, 0x50, 0xFF};
        viscaCopy(out, r, sizeof(r));
        noteCmd("visca if-clear");
        return;
    }
    // Command cancel  8x 2y FF
    if (n == 3 && (type & 0xF0) == 0x20) {
        const uint8_t r[] = {0x90, (uint8_t) (0x60 | (type & 0x0F)), 0x04, 0xFF};
        viscaCopy(out, r, sizeof(r));
        if (g_sink.stop) g_sink.stop();
        noteCmd("visca cancel");
        return;
    }

    if (type == 0x01) {
        // ---- commands ----
        out.sendAck = true;
        if (n >= 6 && m[2] == 0x04 && m[3] == 0x00) {
            // Power  8x 01 04 00 0p FF
            if (m[4] == 0x02) {
                if (g_sink.wake) g_sink.wake();
                noteCmd("visca wake");
            } else if (m[4] == 0x03) {
                if (g_sink.sleep) g_sink.sleep();
                noteCmd("visca sleep");
            }
            return;
        }
        if (n >= 6 && m[2] == 0x04 && m[3] == 0x07) {
            uint8_t p = m[4];
            if (p == 0x00) {
                if (g_sink.zoomRate) g_sink.zoomRate(0);
                noteCmd("visca zoom-stop");
            } else if (p == 0x02) {
                if (g_sink.zoomRate) g_sink.zoomRate(0.5f);
                noteCmd("visca zoom-tele");
            } else if (p == 0x03) {
                if (g_sink.zoomRate) g_sink.zoomRate(-0.5f);
                noteCmd("visca zoom-wide");
            } else if ((p & 0xF0) == 0x20) {
                if (g_sink.zoomRate) g_sink.zoomRate(viscaZoomRate(p & 0x0F, +1));
                noteCmd("visca zoom-tele");
            } else if ((p & 0xF0) == 0x30) {
                if (g_sink.zoomRate) g_sink.zoomRate(viscaZoomRate(p & 0x0F, -1));
                noteCmd("visca zoom-wide");
            }
            return;
        }
        if (n >= 6 && m[2] == 0x04 && m[3] == 0x08) {
            // Focus rocker → same motor as zoom (the gimbal has one lens axis).
            uint8_t p = m[4];
            if (p == 0x00) {
                if (g_sink.zoomRate) g_sink.zoomRate(0);
                noteCmd("visca focus-stop");
            } else if (p == 0x02 || (p & 0xF0) == 0x20) {
                if (g_sink.zoomRate) g_sink.zoomRate(viscaZoomRate(p & 0x0F, +1));
                noteCmd("visca focus-far");
            } else if (p == 0x03 || (p & 0xF0) == 0x30) {
                if (g_sink.zoomRate) g_sink.zoomRate(viscaZoomRate(p & 0x0F, -1));
                noteCmd("visca focus-near");
            }
            return;
        }
        if (n >= 9 && m[2] == 0x04 && m[3] == 0x47) {
            uint16_t z = viscaNibbles4(m + 4);
            if (g_sink.zoomAbs) g_sink.zoomAbs(viscaToZoom(z));
            noteCmd("visca zoom-direct");
            return;
        }
        if (n >= 9 && m[2] == 0x04 && m[3] == 0x48) {
            uint16_t z = viscaNibbles4(m + 4);
            if (g_sink.zoomAbs) g_sink.zoomAbs(viscaToZoom(z));
            noteCmd("visca focus-direct");
            return;
        }
        if (n >= 6 && m[2] == 0x04 && m[3] == 0x3F) {
            uint8_t op = m[4];
            uint8_t id = (n >= 7) ? m[5] : 0;
            if (op == 0x01) savePreset(id);
            else if (op == 0x02) recallPreset(id);
            else if (op == 0x00 && id < kPresetCount) g_presets[id].set = false;
            noteCmd("visca memory");
            return;
        }
        if (n >= 9 && m[2] == 0x06 && m[3] == 0x01) {
            uint8_t vv = m[4], ww = m[5], pp = m[6], tt = m[7];
            float yaw = 0, pitch = 0;
            if (pp == 0x01) yaw = -viscaPanDps(vv);
            else if (pp == 0x02) yaw = viscaPanDps(vv);
            if (tt == 0x01) pitch = viscaTiltDps(ww);
            else if (tt == 0x02) pitch = -viscaTiltDps(ww);
            driveSpeed(yaw, pitch);
            noteCmd((pp == 0x03 && tt == 0x03) ? "visca stop" : "visca drive");
            return;
        }
        if (n >= 14 && m[2] == 0x06 && (m[3] == 0x02 || m[3] == 0x03)) {
            // Absolute / relative: 4 nibble pan + 4 nibble tilt (classic).
            int16_t pan = (int16_t) viscaNibbles4(m + 6);
            int16_t tilt = (int16_t) viscaNibbles4(m + 10);
            float yaw = pan * 0.1f;
            float pitch = tilt * 0.1f;
            if (m[3] == 0x03) {
                float cy = 0, cr = 0, cp = 0;
                if (g_sink.getAttitude) g_sink.getAttitude(&cy, &cr, &cp);
                yaw += cy;
                pitch += cp;
            }
            goPosition(yaw, pitch);
            noteCmd(m[3] == 0x02 ? "visca abs" : "visca rel");
            return;
        }
        if (n >= 5 && m[2] == 0x06 && m[3] == 0x04) {
            if (g_sink.home) g_sink.home();
            noteCmd("visca home");
            return;
        }
        if (n >= 5 && m[2] == 0x06 && m[3] == 0x05) {
            if (g_sink.home) g_sink.home();
            noteCmd("visca reset");
            return;
        }
        // Unknown command: still ACK+completion so the controller does not stall.
        noteCmd("visca nop");
        return;
    }

    if (type == 0x09) {
        // ---- inquiries ----
        if (n >= 5 && m[2] == 0x00 && m[3] == 0x02) {
            // CAM_VersionInq — a generic Sony-shaped reply so RM-IP boxes accept us.
            const uint8_t r[] = {0x90, 0x50, 0x00, 0x20, 0x04, 0x24, 0x02, 0x00, 0xFF};
            viscaCopy(out, r, sizeof(r));
            noteCmd("visca version");
            return;
        }
        if (n >= 5 && m[2] == 0x04 && m[3] == 0x00) {
            const uint8_t r[] = {0x90, 0x50, 0x02, 0xFF};
            viscaCopy(out, r, sizeof(r));
            return;
        }
        if (n >= 5 && m[2] == 0x04 && m[3] == 0x47) {
            int z = 0;
            if (g_sink.getZoom) g_sink.getZoom(&z);
            uint8_t nb[4];
            putNibbles4(nb, zoomToVisca(z));
            uint8_t r[] = {0x90, 0x50, nb[0], nb[1], nb[2], nb[3], 0xFF};
            viscaCopy(out, r, sizeof(r));
            return;
        }
        if (n >= 5 && m[2] == 0x04 && m[3] == 0x48) {
            int z = 0;
            if (g_sink.getZoom) g_sink.getZoom(&z);
            uint8_t nb[4];
            putNibbles4(nb, zoomToVisca(z));
            uint8_t r[] = {0x90, 0x50, nb[0], nb[1], nb[2], nb[3], 0xFF};
            viscaCopy(out, r, sizeof(r));
            return;
        }
        if (n >= 5 && m[2] == 0x06 && m[3] == 0x12) {
            float y = 0, r = 0, p = 0;
            if (g_sink.getAttitude) g_sink.getAttitude(&y, &r, &p);
            int16_t pan = (int16_t) lroundf(applyPan(y) * 10.0f);
            int16_t tilt = (int16_t) lroundf(applyTilt(p) * 10.0f);
            uint8_t yb[4], tb[4];
            putNibbles4(yb, (uint16_t) pan);
            putNibbles4(tb, (uint16_t) tilt);
            uint8_t resp[] = {0x90, 0x50,
                              yb[0], yb[1], yb[2], yb[3],
                              tb[0], tb[1], tb[2], tb[3], 0xFF};
            viscaCopy(out, resp, sizeof(resp));
            return;
        }
        // Unknown inquiry: empty success so pollers do not retry forever.
        const uint8_t empty[] = {0x90, 0x50, 0xFF};
        viscaCopy(out, empty, sizeof(empty));
        return;
    }

    out.syntax = true;
}

static size_t wrapViscaIp(uint8_t *out, size_t cap, uint16_t type, uint32_t seq,
                          const uint8_t *pay, size_t payLen) {
    if (cap < 8 + payLen) return 0;
    out[0] = (uint8_t) (type >> 8);
    out[1] = (uint8_t) type;
    out[2] = (uint8_t) (payLen >> 8);
    out[3] = (uint8_t) payLen;
    out[4] = (uint8_t) (seq >> 24);
    out[5] = (uint8_t) (seq >> 16);
    out[6] = (uint8_t) (seq >> 8);
    out[7] = (uint8_t) seq;
    memcpy(out + 8, pay, payLen);
    return 8 + payLen;
}

static void viscaReplyRaw(WiFiUDP &udp, const IPAddress &ip, uint16_t port,
                          const ViscaOut &r) {
    if (r.syntax) {
        static const uint8_t syn[] = {0x90, 0x60, 0x02, 0xFF};
        udp.beginPacket(ip, port);
        udp.write(syn, sizeof(syn));
        udp.endPacket();
        return;
    }
    if (r.sendAck) {
        static const uint8_t ack[] = {0x90, 0x41, 0xFF};
        static const uint8_t cmp[] = {0x90, 0x51, 0xFF};
        udp.beginPacket(ip, port);
        udp.write(ack, sizeof(ack));
        udp.endPacket();
        udp.beginPacket(ip, port);
        udp.write(cmp, sizeof(cmp));
        udp.endPacket();
    }
    if (r.sendReply && r.len) {
        udp.beginPacket(ip, port);
        udp.write(r.data, r.len);
        udp.endPacket();
    }
}

static void viscaReplyIp(WiFiUDP &udp, const IPAddress &ip, uint16_t port,
                         uint32_t seq, const ViscaOut &r) {
    uint8_t pkt[40];
    if (r.syntax) {
        static const uint8_t syn[] = {0x90, 0x60, 0x02, 0xFF};
        size_t n = wrapViscaIp(pkt, sizeof(pkt), 0x0111, seq, syn, sizeof(syn));
        udp.beginPacket(ip, port);
        udp.write(pkt, n);
        udp.endPacket();
        return;
    }
    if (r.sendAck) {
        static const uint8_t ack[] = {0x90, 0x41, 0xFF};
        static const uint8_t cmp[] = {0x90, 0x51, 0xFF};
        size_t n = wrapViscaIp(pkt, sizeof(pkt), 0x0111, seq, ack, sizeof(ack));
        udp.beginPacket(ip, port);
        udp.write(pkt, n);
        udp.endPacket();
        n = wrapViscaIp(pkt, sizeof(pkt), 0x0111, seq, cmp, sizeof(cmp));
        udp.beginPacket(ip, port);
        udp.write(pkt, n);
        udp.endPacket();
    }
    if (r.sendReply && r.len) {
        size_t n = wrapViscaIp(pkt, sizeof(pkt), 0x0111, seq, r.data, r.len);
        udp.beginPacket(ip, port);
        udp.write(pkt, n);
        udp.endPacket();
    }
}

static void viscaReplyTcp(WiFiClient &c, const ViscaOut &r) {
    if (!c) return;
    if (r.syntax) {
        static const uint8_t syn[] = {0x90, 0x60, 0x02, 0xFF};
        c.write(syn, sizeof(syn));
        return;
    }
    if (r.sendAck) {
        static const uint8_t ack[] = {0x90, 0x41, 0xFF};
        static const uint8_t cmp[] = {0x90, 0x51, 0xFF};
        c.write(ack, sizeof(ack));
        c.write(cmp, sizeof(cmp));
    }
    if (r.sendReply && r.len) c.write(r.data, r.len);
}

static void dispatchVisca(const uint8_t *m, size_t n, ViscaOut &out) {
    handleVisca(m, n, out);
}

static void onViscaDatagram(WiFiUDP &udp, bool framedIp, uint32_t *counter) {
    int n = udp.parsePacket();
    if (n <= 0) return;
    uint8_t buf[48];
    int got = udp.read(buf, sizeof(buf));
    if (got <= 0) return;
    IPAddress ip = udp.remoteIP();
    uint16_t port = udp.remotePort();
    (*counter)++;

    if (framedIp || (got >= 9 && buf[0] == 0x01 && (buf[1] == 0x00 || buf[1] == 0x10))) {
        uint16_t ptype = (uint16_t) ((buf[0] << 8) | buf[1]);
        uint16_t plen = (uint16_t) ((buf[2] << 8) | buf[3]);
        uint32_t seq = ((uint32_t) buf[4] << 24) | ((uint32_t) buf[5] << 16) |
                       ((uint32_t) buf[6] << 8) | buf[7];
        if (ptype == 0x0200) {
            // Sequence reset / control.
            uint8_t payload = (got > 8) ? buf[8] : 0x01;
            uint8_t pkt[12];
            uint8_t body[1] = {payload ? payload : (uint8_t) 0x01};
            size_t wn = wrapViscaIp(pkt, sizeof(pkt), 0x0201, seq, body, 1);
            udp.beginPacket(ip, port);
            udp.write(pkt, wn);
            udp.endPacket();
            noteCmd("visca seq-reset", ip);
            return;
        }
        if (got < 8 + 3 || plen < 3 || (size_t) got < 8u + plen) return;
        ViscaOut r;
        dispatchVisca(buf + 8, plen, r);
        snprintf(g_lastFrom, sizeof(g_lastFrom), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
        viscaReplyIp(udp, ip, port, seq, r);
        return;
    }
    // Raw VISCA (and also VISCA-IP port with a controller that forgot the header).
    if (got >= 3 && (buf[0] & 0xF0) == 0x80) {
        ViscaOut r;
        dispatchVisca(buf, (size_t) got, r);
        snprintf(g_lastFrom, sizeof(g_lastFrom), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
        viscaReplyRaw(udp, ip, port, r);
    }
}

// ---------------------------------------------------------------------------
// Pelco-D / Pelco-P
// ---------------------------------------------------------------------------

static bool pelcoDValid(const uint8_t *m) {
    uint8_t sum = (uint8_t) (m[1] + m[2] + m[3] + m[4] + m[5]);
    return m[0] == 0xFF && sum == m[6];
}

static bool pelcoPValid(const uint8_t *m) {
    if (m[0] != 0xA0 || m[6] != 0xAF) return false;
    uint8_t x = 0;
    for (int i = 1; i <= 6; i++) x ^= m[i];
    return x == m[7];
}

static void handlePelcoBits(uint8_t cmd1, uint8_t cmd2, uint8_t d1, uint8_t d2) {
    // Extended commands (cmd1 == 0): set / clear / go-to preset.
    if (cmd1 == 0x00 && (cmd2 == 0x03 || cmd2 == 0x07 || cmd2 == 0x05)) {
        int id = d2;
        if (cmd2 == 0x07 && id == 0x21) {
            if (g_sink.home) g_sink.home();
            noteCmd("pelco flip");
            return;
        }
        if (cmd2 == 0x03) savePreset(id);
        else if (cmd2 == 0x07) recallPreset(id);
        else if (cmd2 == 0x05 && id < kPresetCount) g_presets[id].set = false;
        noteCmd("pelco preset");
        return;
    }
    if (cmd1 == 0x00 && cmd2 == 0x0F) {
        if (g_sink.home) g_sink.home();
        noteCmd("pelco reset");
        return;
    }

    // Command 2: bit0 right, bit1 left, bit2 up, bit3 down, bit4 tele, bit5 wide.
    bool right = cmd2 & 0x01;
    bool left = cmd2 & 0x02;
    bool up = cmd2 & 0x04;
    bool down = cmd2 & 0x08;
    bool tele = cmd2 & 0x10;
    bool wide = cmd2 & 0x20;
    bool focusFar = cmd1 & 0x40;
    bool focusNear = cmd1 & 0x20;

    float yaw = 0, pitch = 0;
    float panSp = pelcoDps(d1);
    float tiltSp = pelcoDps(d2);
    if (left) yaw = -panSp;
    else if (right) yaw = panSp;
    if (up) pitch = tiltSp;
    else if (down) pitch = -tiltSp;
    driveSpeed(yaw, pitch);

    // Each Pelco frame is a full bitfield snapshot, including the lens rocker.
    if (tele || focusFar) {
        if (g_sink.zoomRate) g_sink.zoomRate(0.6f);
    } else if (wide || focusNear) {
        if (g_sink.zoomRate) g_sink.zoomRate(-0.6f);
    } else if (g_sink.zoomRate) {
        g_sink.zoomRate(0);
    }

    noteCmd((left || right || up || down || tele || wide || focusFar || focusNear)
                ? "pelco drive" : "pelco stop");
}

static void handlePelcoFrame(const uint8_t *m, size_t n, const IPAddress &from) {
    if (n >= 7 && m[0] == 0xFF && pelcoDValid(m)) {
        g_nPelco++;
        handlePelcoBits(m[2], m[3], m[4], m[5]);
        snprintf(g_lastFrom, sizeof(g_lastFrom), "%u.%u.%u.%u", from[0], from[1], from[2], from[3]);
        return;
    }
    if (n >= 8 && m[0] == 0xA0 && pelcoPValid(m)) {
        g_nPelco++;
        handlePelcoBits(m[2], m[3], m[4], m[5]);
        snprintf(g_lastFrom, sizeof(g_lastFrom), "%u.%u.%u.%u", from[0], from[1], from[2], from[3]);
    }
}

static void onPelcoUdp() {
    int n = g_udpPelco.parsePacket();
    if (n <= 0) return;
    uint8_t buf[32];
    int got = g_udpPelco.read(buf, sizeof(buf));
    if (got > 0) handlePelcoFrame(buf, (size_t) got, g_udpPelco.remoteIP());
}

// ---------------------------------------------------------------------------
// Panasonic AW (ASCII)
// ---------------------------------------------------------------------------

static int awDigits(const char *s, int n) {
    int v = 0;
    for (int i = 0; i < n; i++) {
        char c = s[i];
        if (c < '0' || c > '9') return -1;
        v = v * 10 + (c - '0');
    }
    return v;
}

static int awHex(const char *s, int n) {
    int v = 0;
    for (int i = 0; i < n; i++) {
        char c = s[i];
        int d;
        if (c >= '0' && c <= '9') d = c - '0';
        else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
        else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
        else return -1;
        v = (v << 4) | d;
    }
    return v;
}

// Returns a static ACK/inquiry body (valid until the next call).
static const char *handleAw(char *cmd) {
    // Strip CR/LF and a leading '#'.
    size_t n = strlen(cmd);
    while (n && (cmd[n - 1] == '\r' || cmd[n - 1] == '\n' || cmd[n - 1] == ' ')) {
        cmd[--n] = '\0';
    }
    if (cmd[0] == '#') {
        memmove(cmd, cmd + 1, n);
        n = strlen(cmd);
    }
    if (n >= 7 && strncmp(cmd, "PTS", 3) == 0) {
        int pan = awDigits(cmd + 3, 2);
        int tilt = awDigits(cmd + 5, 2);
        if (pan >= 0 && tilt >= 0) {
            driveSpeed(awNorm(pan) * g_maxRate, awNorm(tilt) * g_maxRate);
            noteCmd("aw pts");
        }
        return "sST";
    }
    if (n >= 3 && cmd[0] == 'Z') {
        int z = awDigits(cmd + 1, 2);
        if (z >= 0 && g_sink.zoomRate) {
            float r = awNorm(z);
            g_sink.zoomRate(fabsf(r) < 0.02f ? 0.0f : r);
            noteCmd("aw zoom");
        }
        return "sST";
    }
    if (n >= 3 && cmd[0] == 'F') {
        int z = awDigits(cmd + 1, 2);
        if (z >= 0 && g_sink.zoomRate) {
            float r = awNorm(z);
            g_sink.zoomRate(fabsf(r) < 0.02f ? 0.0f : r);
            noteCmd("aw focus");
        }
        return "sST";
    }
    if (n >= 11 && strncmp(cmd, "APC", 3) == 0) {
        int pan = awHex(cmd + 3, 4);
        int tilt = awHex(cmd + 7, 4);
        if (pan >= 0 && tilt >= 0) {
            float yaw = ((int) pan - 0x8000) * 0.1f;
            float pitch = ((int) tilt - 0x8000) * 0.1f;
            goPosition(yaw, pitch);
            noteCmd("aw apc");
        }
        return "sST";
    }
    if (n == 3 && strncmp(cmd, "APC", 3) == 0) {
        float y = 0, r = 0, p = 0;
        if (g_sink.getAttitude) g_sink.getAttitude(&y, &r, &p);
        int pan = 0x8000 + (int) lroundf(applyPan(y) * 10.0f);
        int tilt = 0x8000 + (int) lroundf(applyTilt(p) * 10.0f);
        if (pan < 0) pan = 0;
        if (pan > 0xFFFF) pan = 0xFFFF;
        if (tilt < 0) tilt = 0;
        if (tilt > 0xFFFF) tilt = 0xFFFF;
        static char body[16];
        snprintf(body, sizeof(body), "aPC%04X%04X", pan, tilt);
        return body;
    }
    if (n >= 3 && cmd[0] == 'R' && n <= 4) {
        int id = awDigits(cmd + 1, (int) n - 1);
        if (id >= 0) recallPreset(id);
        noteCmd("aw recall");
        return "sST";
    }
    if (n >= 3 && cmd[0] == 'M' && n <= 4) {
        int id = awDigits(cmd + 1, (int) n - 1);
        if (id >= 0) savePreset(id);
        noteCmd("aw memory");
        return "sST";
    }
    if (n >= 2 && cmd[0] == 'O') {
        if (cmd[1] == '1' && g_sink.wake) g_sink.wake();
        else if (cmd[1] == '0' && g_sink.sleep) g_sink.sleep();
        noteCmd("aw power");
        return "sST";
    }
    return "eR2"; // syntax
}

static void onAwUdp() {
    int n = g_udpAw.parsePacket();
    if (n <= 0) return;
    char buf[48];
    int got = g_udpAw.read((uint8_t *) buf, sizeof(buf) - 1);
    if (got <= 0) return;
    buf[got] = '\0';
    g_nAw++;
    IPAddress ip = g_udpAw.remoteIP();
    uint16_t port = g_udpAw.remotePort();
    const char *reply = handleAw(buf);
    snprintf(g_lastFrom, sizeof(g_lastFrom), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
    if (reply) {
        g_udpAw.beginPacket(ip, port);
        g_udpAw.write((const uint8_t *) reply, strlen(reply));
        g_udpAw.endPacket();
    }
}

// ---------------------------------------------------------------------------
// TCP accept / byte machines
// ---------------------------------------------------------------------------

static void takeClient(WiFiServer &srv, TcpSlot *slots, int nslot) {
    WiFiClient incoming = srv.accept();
    if (!incoming) return;
    incoming.setNoDelay(true);
    incoming.setTimeout(1);
    for (int i = 0; i < nslot; i++) {
        if (!slots[i].client || !slots[i].client.connected()) {
            slots[i].client.stop();
            slots[i].client = incoming;
            slots[i].len = 0;
            Serial.printf("[ptz] tcp client %s\n", incoming.remoteIP().toString().c_str());
            return;
        }
    }
    incoming.stop();
}

static void pumpViscaTcp(TcpSlot &s) {
    if (!s.client) return;
    if (!s.client.connected()) {
        s.client.stop();
        s.len = 0;
        return;
    }
    while (s.client.available()) {
        int b = s.client.read();
        if (b < 0) break;
        if (s.len < kTcpBuf) s.buf[s.len++] = (uint8_t) b;
        else s.len = 0;
        if ((uint8_t) b == 0xFF && s.len >= 3) {
            // Could be raw VISCA. Also skip a Sony IP header if present.
            const uint8_t *msg = s.buf;
            size_t n = s.len;
            if (n >= 11 && s.buf[0] == 0x01 && (s.buf[1] == 0x00 || s.buf[1] == 0x10)) {
                uint16_t plen = (uint16_t) ((s.buf[2] << 8) | s.buf[3]);
                if (n >= 8u + plen) {
                    msg = s.buf + 8;
                    n = plen;
                }
            }
            ViscaOut r;
            dispatchVisca(msg, n, r);
            snprintf(g_lastFrom, sizeof(g_lastFrom), "%u.%u.%u.%u",
                     s.client.remoteIP()[0], s.client.remoteIP()[1],
                     s.client.remoteIP()[2], s.client.remoteIP()[3]);
            viscaReplyTcp(s.client, r);
            g_nViscaTcp++;
            s.len = 0;
        }
    }
}

static void pumpPelcoTcp(TcpSlot &s) {
    if (!s.client) return;
    if (!s.client.connected()) {
        s.client.stop();
        s.len = 0;
        return;
    }
    while (s.client.available()) {
        int b = s.client.read();
        if (b < 0) break;
        if (s.len == 0 && (uint8_t) b != 0xFF && (uint8_t) b != 0xA0) continue;
        if (s.len < kTcpBuf) s.buf[s.len++] = (uint8_t) b;
        else s.len = 0;
        if (s.buf[0] == 0xFF && s.len >= 7) {
            handlePelcoFrame(s.buf, 7, s.client.remoteIP());
            memmove(s.buf, s.buf + 7, s.len - 7);
            s.len -= 7;
        } else if (s.buf[0] == 0xA0 && s.len >= 8) {
            handlePelcoFrame(s.buf, 8, s.client.remoteIP());
            memmove(s.buf, s.buf + 8, s.len - 8);
            s.len -= 8;
        }
    }
}

static void ptzTask(void *) {
    for (;;) {
        onViscaDatagram(g_udpViscaIp, true, &g_nViscaUdp);
        onViscaDatagram(g_udpViscaRaw, false, &g_nViscaUdp);
        onPelcoUdp();
        onAwUdp();

        takeClient(g_tcpVisca, g_viscaCli, kTcpClients);
        takeClient(g_tcpPelco, g_pelcoCli, kTcpClients);
        for (int i = 0; i < kTcpClients; i++) {
            pumpViscaTcp(g_viscaCli[i]);
            pumpPelcoTcp(g_pelcoCli[i]);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ---------------------------------------------------------------------------
// HTTP CGI (PTZOptics / Sony / Panasonic) — runs on the async_tcp thread.
// ---------------------------------------------------------------------------

static String cgiPeer(AsyncWebServerRequest *req) {
    return req->client() ? req->client()->remoteIP().toString() : String("?");
}

static void cgiOk(AsyncWebServerRequest *req, const char *body = "OK") {
    req->send(200, "text/plain", body);
}

static float cgiSpeed(const String &s, float fallbackDiv) {
    int v = s.toInt();
    if (v <= 0) return g_maxRate * 0.5f;
    float n = (float) v / fallbackDiv;
    if (n > 1.0f) n = 1.0f;
    return n * g_maxRate;
}

static void handlePtzOpticsCgi(AsyncWebServerRequest *req) {
    // /cgi-bin/ptzctrl.cgi?ptzcmd&up&12   (unnamed query tokens)
    String tokens[6];
    int nt = 0;
    for (size_t i = 0; i < req->params() && nt < 6; i++) {
        tokens[nt++] = req->argName(i);
    }
    if (nt == 0) {
        cgiOk(req);
        return;
    }
    int start = 0;
    if (tokens[0] == "ptzcmd") start = 1;
    String act = (start < nt) ? tokens[start] : "";
    act.toLowerCase();
    String spd = (start + 1 < nt) ? tokens[start + 1] : "12";
    float pan = cgiSpeed(spd, 24.0f);
    float tilt = pan;
    g_nCgi++;
    noteCmdHost("cgi ptzoptics", cgiPeer(req).c_str());

    if (act == "up") driveSpeed(0, tilt);
    else if (act == "down") driveSpeed(0, -tilt);
    else if (act == "left") driveSpeed(-pan, 0);
    else if (act == "right") driveSpeed(pan, 0);
    else if (act == "leftup" || act == "upleft") driveSpeed(-pan, tilt);
    else if (act == "rightup" || act == "upright") driveSpeed(pan, tilt);
    else if (act == "leftdown" || act == "downleft") driveSpeed(-pan, -tilt);
    else if (act == "rightdown" || act == "downright") driveSpeed(pan, -tilt);
    else if (act == "ptzstop" || act == "stop") {
        if (g_sink.stop) g_sink.stop();
        if (g_sink.zoomRate) g_sink.zoomRate(0);
    } else if (act == "home") {
        if (g_sink.home) g_sink.home();
    } else if (act == "zoomin") {
        if (g_sink.zoomRate) g_sink.zoomRate(cgiSpeed(spd, 8.0f) / g_maxRate);
    } else if (act == "zoomout") {
        if (g_sink.zoomRate) g_sink.zoomRate(-cgiSpeed(spd, 8.0f) / g_maxRate);
    } else if (act == "zoomstop") {
        if (g_sink.zoomRate) g_sink.zoomRate(0);
    }
    cgiOk(req);
}

static void handleSonyCgi(AsyncWebServerRequest *req) {
    String move = req->hasParam("Move") ? req->getParam("Move")->value()
                                        : (req->hasParam("move") ? req->getParam("move")->value() : "");
    g_nCgi++;
    noteCmdHost("cgi sony", cgiPeer(req).c_str());
    if (move.length()) {
        // Move=up,24,24  or Move=stop
        int c1 = move.indexOf(',');
        String dir = (c1 < 0) ? move : move.substring(0, c1);
        dir.toLowerCase();
        float pan = g_maxRate, tilt = g_maxRate;
        if (c1 >= 0) {
            int c2 = move.indexOf(',', c1 + 1);
            String sp = (c2 < 0) ? move.substring(c1 + 1) : move.substring(c1 + 1, c2);
            pan = cgiSpeed(sp, 24.0f);
            tilt = (c2 < 0) ? pan : cgiSpeed(move.substring(c2 + 1), 24.0f);
        }
        if (dir == "stop") {
            if (g_sink.stop) g_sink.stop();
        } else if (dir == "up") driveSpeed(0, tilt);
        else if (dir == "down") driveSpeed(0, -tilt);
        else if (dir == "left") driveSpeed(-pan, 0);
        else if (dir == "right") driveSpeed(pan, 0);
        else if (dir == "upleft" || dir == "leftup") driveSpeed(-pan, tilt);
        else if (dir == "upright" || dir == "rightup") driveSpeed(pan, tilt);
        else if (dir == "downleft" || dir == "leftdown") driveSpeed(-pan, -tilt);
        else if (dir == "downright" || dir == "rightdown") driveSpeed(pan, -tilt);
        else if (dir == "home") {
            if (g_sink.home) g_sink.home();
        }
    }
    if (req->hasParam("Home") || req->hasParam("home")) {
        if (g_sink.home) g_sink.home();
    }
    cgiOk(req);
}

static void handleAwHttp(AsyncWebServerRequest *req) {
    String cmd = req->hasParam("cmd") ? req->getParam("cmd")->value() : "";
    g_nCgi++;
    noteCmdHost("cgi aw", cgiPeer(req).c_str());
    char buf[48];
    strncpy(buf, cmd.c_str(), sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    const char *reply = handleAw(buf);
    cgiOk(req, reply ? reply : "sST");
}

void ptzBridgeRegisterHttp(AsyncWebServer &server) {
    server.on("/cgi-bin/ptzctrl.cgi", HTTP_GET, handlePtzOpticsCgi);
    server.on("/cgi-bin/ptzctrl.cgi", HTTP_POST, handlePtzOpticsCgi);
    server.on("/command/ptzf.cgi", HTTP_GET, handleSonyCgi);
    server.on("/command/ptzf.cgi", HTTP_POST, handleSonyCgi);
    server.on("/cgi-bin/aw_ptz", HTTP_GET, handleAwHttp);
    server.on("/cgi-bin/aw_ptz", HTTP_POST, handleAwHttp);
    server.on("/cgi-bin/aw_cam", HTTP_GET, handleAwHttp);
    server.on("/cgi-bin/aw_cam", HTTP_POST, handleAwHttp);

    auto *profile = new AsyncCallbackJsonWebHandler("/api/ptz/profile",
        [](AsyncWebServerRequest *req, JsonVariant &json) {
            JsonObject body = json.as<JsonObject>();
            if (!body["max_rate"].isNull()) ptzBridgeSetMaxRate(body["max_rate"] | kMaxRateDefault);
            bool pan = g_invertPan;
            bool tilt = g_invertTilt;
            if (!body["invert_pan"].isNull()) pan = body["invert_pan"].as<bool>();
            if (!body["invert_tilt"].isNull()) tilt = body["invert_tilt"].as<bool>();
            ptzBridgeSetInvert(pan, tilt);
            JsonDocument doc;
            doc["ok"] = "ptz-profile";
            doc["max_rate"] = g_maxRate;
            doc["invert_pan"] = g_invertPan;
            doc["invert_tilt"] = g_invertTilt;
            String out;
            serializeJson(doc, out);
            req->send(200, "application/json", out);
        });
    server.addHandler(profile);
}

void ptzBridgeFillStatus(JsonObject obj) {
    obj["max_rate"] = g_maxRate;
    obj["invert_pan"] = g_invertPan;
    obj["invert_tilt"] = g_invertTilt;
    obj["visca_udp"] = g_nViscaUdp;
    obj["visca_tcp"] = g_nViscaTcp;
    obj["pelco"] = g_nPelco;
    obj["panasonic"] = g_nAw;
    obj["cgi"] = g_nCgi;
    obj["last"] = g_lastCmd[0] ? g_lastCmd : nullptr;
    obj["last_from"] = g_lastFrom[0] ? g_lastFrom : nullptr;
    if (g_lastMs) obj["last_age_s"] = (millis() - g_lastMs) / 1000.0f;
    JsonObject ports = obj["ports"].to<JsonObject>();
    ports["visca_ip_udp"] = kViscaIpUdp;
    ports["visca_raw_udp"] = kViscaRawUdp;
    ports["visca_raw_tcp"] = kViscaRawTcp;
    ports["pelco"] = kPelcoPort;
    ports["panasonic_udp"] = kPanasonicUdp;
}

void ptzBridgeSetMaxRate(float dps) {
    g_maxRate = fmaxf(kMaxRateMin, fminf(kMaxRateMax, dps));
}

float ptzBridgeMaxRate() { return g_maxRate; }

void ptzBridgeSetInvert(bool pan, bool tilt) {
    g_invertPan = pan;
    g_invertTilt = tilt;
}

void ptzBridgeBegin(const PtzSink &sink) {
    g_sink = sink;
    bool ok = true;
    ok = g_udpViscaIp.begin(kViscaIpUdp) && ok;
    ok = g_udpViscaRaw.begin(kViscaRawUdp) && ok;
    ok = g_udpPelco.begin(kPelcoPort) && ok;
    ok = g_udpAw.begin(kPanasonicUdp) && ok;
    g_tcpVisca.begin();
    g_tcpPelco.begin();
    g_tcpVisca.setNoDelay(true);
    g_tcpPelco.setNoDelay(true);
    xTaskCreate(ptzTask, "ptz", 6144, nullptr, 4, nullptr);
    Serial.printf("[ptz] VISCA UDP %u/%u TCP %u  Pelco %u  AW UDP %u%s\n",
                  kViscaIpUdp, kViscaRawUdp, kViscaRawTcp, kPelcoPort, kPanasonicUdp,
                  ok ? "" : "  (a UDP bind failed)");
}

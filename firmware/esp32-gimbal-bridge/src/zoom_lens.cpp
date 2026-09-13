#include "zoom_lens.h"

#include <Preferences.h>
#include <Arduino.h>
#include <cmath>
#include <cstring>

struct ZoomLensPoint {
    uint16_t pos;
    float mm;
};

struct ZoomLensCurve {
    float vmax = 900.0f;
    float accel = 1800.0f;
    uint8_t band = 0; // 0 optical, 1 digital
    uint8_t nIn = 0;
    uint8_t nOut = 0;
    ZoomLensPoint inPts[kZoomLensMaxPoints];
    ZoomLensPoint outPts[kZoomLensMaxPoints];
};

struct ZoomLensProfile {
    bool enabled = false;
    bool coupling = false;
    char name[24] = "";
    float wideMm = 0.0f;
    float teleMm = 0.0f;
    float vmax = 900.0f;
    float accel = 4500.0f;
    float decel = 4500.0f;
    float vcut = 120.0f;
    float vstatic = 0.0f;
    float settleS = 1.5f;
    float repeatMm = 0.0f;
    uint16_t engageIn = 0;
    uint16_t engageOut = 0;
    uint8_t nCurves = 0;
    uint8_t canonical = 0;
    ZoomLensCurve curves[kZoomLensMaxCurves];
};

static ZoomLensProfile g_lens;
static SemaphoreHandle_t g_lensLock = nullptr;
static Preferences g_prefs;

static constexpr uint16_t kFocusPosMin = 1;
static constexpr uint16_t kFocusPosMax = 4095;

static uint16_t clampFocusPos(uint16_t pos) {
    if (pos < kFocusPosMin) return kFocusPosMin;
    if (pos > kFocusPosMax) return kFocusPosMax;
    return pos;
}
static constexpr int kPosDup = 32;
static constexpr float kVmaxDup = 80.0f;
static constexpr size_t kJsonMax = 3072;

static void lock() {
    if (g_lensLock) xSemaphoreTake(g_lensLock, portMAX_DELAY);
}
static void unlock() {
    if (g_lensLock) xSemaphoreGive(g_lensLock);
}

static int curvePointCount(const ZoomLensCurve &c) {
    return (int) c.nIn + (int) c.nOut;
}

static int sampleCountLocked() {
    int n = 0;
    for (int i = 0; i < g_lens.nCurves; i++) n += curvePointCount(g_lens.curves[i]);
    return n;
}

static void sortPoints(ZoomLensPoint *pts, int n) {
    for (int i = 1; i < n; i++) {
        ZoomLensPoint key = pts[i];
        int j = i - 1;
        while (j >= 0 && pts[j].pos > key.pos) {
            pts[j + 1] = pts[j];
            j--;
        }
        pts[j + 1] = key;
    }
}

static void insertPoint(ZoomLensPoint *pts, uint8_t *n, uint16_t pos, float mm) {
    for (int i = 0; i < *n; i++) {
        if (abs((int) pts[i].pos - (int) pos) <= kPosDup) {
            pts[i].pos = pos;
            pts[i].mm = mm;
            sortPoints(pts, *n);
            return;
        }
    }
    if (*n < kZoomLensMaxPoints) {
        pts[*n].pos = pos;
        pts[*n].mm = mm;
        (*n)++;
        sortPoints(pts, *n);
        return;
    }
    int best = 0;
    int bestD = abs((int) pts[0].pos - (int) pos);
    for (int i = 1; i < *n; i++) {
        int d = abs((int) pts[i].pos - (int) pos);
        if (d < bestD) {
            bestD = d;
            best = i;
        }
    }
    pts[best].pos = pos;
    pts[best].mm = mm;
    sortPoints(pts, *n);
}

static bool interpMm(const ZoomLensPoint *pts, int n, int pos, float *out) {
    if (n <= 0) return false;
    if (n == 1) {
        *out = pts[0].mm;
        return true;
    }
    if (pos <= pts[0].pos) {
        *out = pts[0].mm;
        return true;
    }
    if (pos >= pts[n - 1].pos) {
        *out = pts[n - 1].mm;
        return true;
    }
    for (int i = 0; i < n - 1; i++) {
        if (pos >= pts[i].pos && pos <= pts[i + 1].pos) {
            float span = (float) (pts[i + 1].pos - pts[i].pos);
            float t = span > 0.0f ? (pos - pts[i].pos) / span : 0.0f;
            *out = pts[i].mm + t * (pts[i + 1].mm - pts[i].mm);
            return true;
        }
    }
    return false;
}

static bool interpPos(const ZoomLensPoint *pts, int n, float mm, int *out) {
    if (n <= 0) return false;
    if (n == 1) {
        *out = pts[0].pos;
        return true;
    }
    for (int i = 0; i < n - 1; i++) {
        float m0 = pts[i].mm, m1 = pts[i + 1].mm;
        float lo = fminf(m0, m1), hi = fmaxf(m0, m1);
        if (mm >= lo && mm <= hi) {
            if (fabsf(m1 - m0) < 1e-3f) {
                *out = pts[i].pos;
                return true;
            }
            float t = (mm - m0) / (m1 - m0);
            *out = (int) lroundf((float) pts[i].pos + t * (float) (pts[i + 1].pos - pts[i].pos));
            return true;
        }
    }
    int best = 0;
    float bestD = fabsf(pts[0].mm - mm);
    for (int i = 1; i < n; i++) {
        float d = fabsf(pts[i].mm - mm);
        if (d < bestD) {
            bestD = d;
            best = i;
        }
    }
    *out = pts[best].pos;
    return true;
}

static void copyTable(JsonArrayConst arr, ZoomLensPoint *pts, uint8_t *n) {
    *n = 0;
    for (JsonVariantConst v : arr) {
        if (*n >= kZoomLensMaxPoints) break;
        uint16_t pos = 0;
        float mm = 0;
        if (v.is<JsonArrayConst>()) {
            JsonArrayConst a = v.as<JsonArrayConst>();
            pos = a[0] | 0;
            mm = a[1] | 0.0f;
        } else {
            pos = v["pos"] | 0;
            mm = v["mm"] | 0.0f;
        }
        if (mm <= 0.0f || mm > 2000.0f) continue;
        pos = clampFocusPos(pos);
        insertPoint(pts, n, pos, mm);
    }
}

static void emitTableSimple(JsonObject obj, const char *key, const ZoomLensPoint *pts, int n) {
    JsonArray arr = obj[key].to<JsonArray>();
    for (int i = 0; i < n; i++) {
        JsonObject p = arr.add<JsonObject>();
        p["pos"] = pts[i].pos;
        p["mm"] = pts[i].mm;
    }
}

static int pickCurveLocked(float hint, bool needSamples, int band = -1) {
    int best = -1;
    float bestD = 1.0e9f;
    for (int i = 0; i < g_lens.nCurves; i++) {
        if (needSamples && curvePointCount(g_lens.curves[i]) < 2) continue;
        if (band >= 0 && g_lens.curves[i].band != (uint8_t) band) continue;
        float d = fabsf(g_lens.curves[i].vmax - hint);
        if (d < bestD) {
            bestD = d;
            best = i;
        }
    }
    if (best >= 0) return best;
    if (needSamples) return -1;
    if (g_lens.nCurves > 0) {
        int c = g_lens.canonical;
        if (c >= g_lens.nCurves) c = 0;
        return c;
    }
    return -1;
}

static int findCurveLocked(float vmax, int band) {
    for (int i = 0; i < g_lens.nCurves; i++) {
        if (g_lens.curves[i].band != (uint8_t) band) continue;
        if (fabsf(g_lens.curves[i].vmax - vmax) <= kVmaxDup) return i;
    }
    return -1;
}

static int ensureCurveLocked(float vmax, float accel, int band) {
    vmax = fmaxf(50.0f, fminf(8000.0f, vmax));
    accel = fmaxf(50.0f, fminf(40000.0f, accel));
    if (band < 0) band = 0;
    if (band > 1) band = 1;
    int i = findCurveLocked(vmax, band);
    if (i >= 0) {
        g_lens.curves[i].vmax = vmax;
        g_lens.curves[i].accel = accel;
        g_lens.curves[i].band = (uint8_t) band;
        return i;
    }
    if (g_lens.nCurves < kZoomLensMaxCurves) {
        i = g_lens.nCurves++;
        g_lens.curves[i] = ZoomLensCurve{};
        g_lens.curves[i].vmax = vmax;
        g_lens.curves[i].accel = accel;
        g_lens.curves[i].band = (uint8_t) band;
        return i;
    }
    i = pickCurveLocked(vmax, false, band);
    if (i < 0) i = 0;
    g_lens.curves[i].vmax = vmax;
    g_lens.curves[i].accel = accel;
    g_lens.curves[i].band = (uint8_t) band;
    return i;
}

static bool posFromCurve(const ZoomLensCurve &c, float mm, int dir, int *pos) {
    bool ok = false;
    int a = -1, b = -1;
    if (dir >= 0 && c.nIn >= 1) ok = interpPos(c.inPts, c.nIn, mm, &a);
    if (dir <= 0 && c.nOut >= 1) ok = interpPos(c.outPts, c.nOut, mm, &b) || ok;
    if (!ok) return false;
    if (dir > 0 && a >= 0) {
        *pos = constrain(a, (int) kFocusPosMin, (int) kFocusPosMax);
        return true;
    }
    if (dir < 0 && b >= 0) {
        *pos = constrain(b, (int) kFocusPosMin, (int) kFocusPosMax);
        return true;
    }
    if (a >= 0 && b >= 0) {
        *pos = constrain((a + b) / 2, (int) kFocusPosMin, (int) kFocusPosMax);
        return true;
    }
    if (a >= 0) {
        *pos = constrain(a, (int) kFocusPosMin, (int) kFocusPosMax);
        return true;
    }
    if (b >= 0) {
        *pos = constrain(b, (int) kFocusPosMin, (int) kFocusPosMax);
        return true;
    }
    return false;
}

static bool mmFromCurve(const ZoomLensCurve &c, int pos, int dir, float *mm) {
    float a = NAN, b = NAN;
    bool ha = false, hb = false;
    if (dir >= 0 && c.nIn >= 1) ha = interpMm(c.inPts, c.nIn, pos, &a);
    if (dir <= 0 && c.nOut >= 1) hb = interpMm(c.outPts, c.nOut, pos, &b);
    if (dir > 0 && ha) {
        *mm = a;
        return true;
    }
    if (dir < 0 && hb) {
        *mm = b;
        return true;
    }
    if (ha && hb) {
        *mm = 0.5f * (a + b);
        return true;
    }
    if (ha) {
        *mm = a;
        return true;
    }
    if (hb) {
        *mm = b;
        return true;
    }
    return false;
}

static bool loadFromNvs() {
    if (!g_prefs.begin("zoomlens", true)) return false;
    size_t n = g_prefs.getBytesLength("json");
    if (n == 0 || n >= kJsonMax) {
        g_prefs.end();
        return false;
    }
    char buf[kJsonMax];
    n = g_prefs.getBytes("json", buf, sizeof(buf) - 1);
    g_prefs.end();
    if (n == 0) return false;
    buf[n] = '\0';
    JsonDocument doc;
    if (deserializeJson(doc, buf, n)) return false;
    return zoomLensApplyJson(doc.as<JsonVariantConst>());
}

void zoomLensSave() {
    JsonDocument doc;
    zoomLensFillJson(doc.to<JsonObject>());
    char buf[kJsonMax];
    size_t n = serializeJson(doc, buf, sizeof(buf));
    if (n == 0 || n >= sizeof(buf)) return;
    if (!g_prefs.begin("zoomlens", false)) return;
    g_prefs.putBytes("json", buf, n);
    g_prefs.end();
}

void zoomLensBegin() {
    if (!g_lensLock) g_lensLock = xSemaphoreCreateMutex();
    loadFromNvs();
}

bool zoomLensEnabled() {
    lock();
    bool on = g_lens.enabled && sampleCountLocked() >= 2;
    unlock();
    return on;
}

bool zoomLensWantEnabled() {
    lock();
    bool on = g_lens.enabled;
    unlock();
    return on;
}

bool zoomLensCouplingReady() {
    lock();
    bool ok = g_lens.coupling && (g_lens.engageIn > 0 || g_lens.engageOut > 0);
    unlock();
    return ok;
}

int zoomLensSampleCount() {
    lock();
    int n = sampleCountLocked();
    unlock();
    return n;
}

bool zoomLensGetRamp(float *vmax, float *accel, float *vcut, float *decel) {
    lock();
    bool ok = g_lens.vmax >= 50.0f && g_lens.accel >= 50.0f &&
              (g_lens.coupling || g_lens.enabled || sampleCountLocked() >= 2);
    if (ok) {
        int c = g_lens.canonical;
        if (c < g_lens.nCurves) {
            if (vmax) *vmax = g_lens.curves[c].vmax;
            if (accel) *accel = g_lens.curves[c].accel;
        } else {
            if (vmax) *vmax = g_lens.vmax;
            if (accel) *accel = g_lens.accel;
        }
        if (vcut) *vcut = g_lens.vcut;
        if (decel) *decel = g_lens.decel >= 50.0f ? g_lens.decel : g_lens.accel;
    }
    unlock();
    return ok;
}

bool zoomLensGetRange(float *wideMm, float *teleMm) {
    lock();
    bool ok = g_lens.wideMm > 0.0f && g_lens.teleMm > g_lens.wideMm;
    if (ok) {
        if (wideMm) *wideMm = g_lens.wideMm;
        if (teleMm) *teleMm = g_lens.teleMm;
    }
    unlock();
    return ok;
}

bool zoomLensGetEngage(int dir, int *counts) {
    lock();
    bool ok = g_lens.coupling;
    int v = dir >= 0 ? (int) g_lens.engageOut : (int) g_lens.engageIn;
    if (v <= 0) v = dir >= 0 ? (int) g_lens.engageIn : (int) g_lens.engageOut;
    unlock();
    if (!ok || v <= 0) return false;
    if (counts) *counts = v;
    return true;
}

bool zoomLensRampForVmax(float hint, float *vmax, float *accel) {
    lock();
    int i = pickCurveLocked(hint, true);
    if (i < 0) i = pickCurveLocked(hint, false);
    bool ok = i >= 0;
    if (ok) {
        if (vmax) *vmax = g_lens.curves[i].vmax;
        if (accel) *accel = g_lens.curves[i].accel;
    }
    unlock();
    return ok;
}

bool zoomLensPosFromMm(float mm, int dir, int *pos) {
    lock();
    float hint = g_lens.vmax;
    unlock();
    return zoomLensPosFromMmAt(mm, dir, hint, pos, nullptr, nullptr);
}

bool zoomLensPosFromMmAt(float mm, int dir, float vmaxHint, int *pos,
                         float *useVmax, float *useAccel) {
    if (!pos) return false;
    lock();
    int band = (g_lens.teleMm > 0.0f && mm > g_lens.teleMm + 0.35f) ? 1 : 0;
    int i = pickCurveLocked(vmaxHint, true, band);
    if (i < 0) i = pickCurveLocked(vmaxHint, true, -1);
    if (i < 0 && g_lens.canonical < g_lens.nCurves &&
        curvePointCount(g_lens.curves[g_lens.canonical]) >= 2) {
        i = g_lens.canonical;
    }
    bool ok = false;
    if (i >= 0) {
        ok = posFromCurve(g_lens.curves[i], mm, dir, pos);
        if (ok) {
            if (useVmax) *useVmax = g_lens.curves[i].vmax;
            if (useAccel) *useAccel = g_lens.curves[i].accel;
        }
    }
    unlock();
    return ok;
}

bool zoomLensMmFromPos(int pos, int dir, float *mm) {
    if (!mm) return false;
    lock();
    int i = g_lens.canonical;
    if (i >= g_lens.nCurves || curvePointCount(g_lens.curves[i]) < 1) {
        i = pickCurveLocked(g_lens.vmax, true);
    }
    bool ok = i >= 0 && mmFromCurve(g_lens.curves[i], pos, dir, mm);
    unlock();
    return ok;
}

bool zoomLensMmAtOptical(float t, float *mm) {
    if (!mm) return false;
    t = fmaxf(0.0f, fminf(1.0f, t));
    lock();
    bool ok = g_lens.wideMm > 0.0f && g_lens.teleMm > g_lens.wideMm;
    if (ok) *mm = g_lens.wideMm + t * (g_lens.teleMm - g_lens.wideMm);
    unlock();
    return ok;
}

void zoomLensFillJson(JsonObject obj) {
    lock();
    obj["enabled"] = g_lens.enabled;
    obj["coupling"] = g_lens.coupling;
    obj["name"] = g_lens.name;
    obj["wide_mm"] = g_lens.wideMm;
    obj["tele_mm"] = g_lens.teleMm;
    obj["vmax"] = g_lens.vmax;
    obj["accel"] = g_lens.accel;
    obj["decel"] = g_lens.decel;
    obj["vcut"] = g_lens.vcut;
    obj["vstatic"] = g_lens.vstatic;
    obj["repeat_mm"] = g_lens.repeatMm;
    obj["settle_s"] = g_lens.settleS;
    obj["engage_in"] = g_lens.engageIn;
    obj["engage_out"] = g_lens.engageOut;
    obj["canonical"] = g_lens.canonical;
    int count = 0;
    JsonArray curves = obj["curves"].to<JsonArray>();
    for (int i = 0; i < g_lens.nCurves; i++) {
        JsonObject c = curves.add<JsonObject>();
        c["vmax"] = g_lens.curves[i].vmax;
        c["accel"] = g_lens.curves[i].accel;
        c["band"] = g_lens.curves[i].band;
        emitTableSimple(c, "in", g_lens.curves[i].inPts, g_lens.curves[i].nIn);
        emitTableSimple(c, "out", g_lens.curves[i].outPts, g_lens.curves[i].nOut);
        count += curvePointCount(g_lens.curves[i]);
    }
    obj["count"] = count;
    int can = g_lens.canonical < g_lens.nCurves ? g_lens.canonical : 0;
    if (g_lens.nCurves > 0) {
        emitTableSimple(obj, "in", g_lens.curves[can].inPts, g_lens.curves[can].nIn);
        emitTableSimple(obj, "out", g_lens.curves[can].outPts, g_lens.curves[can].nOut);
    } else {
        obj["in"].to<JsonArray>();
        obj["out"].to<JsonArray>();
    }
    unlock();
}

static void importLegacyTables(JsonObjectConst o, ZoomLensProfile &next) {
    if (o["in"].isNull() && o["out"].isNull()) return;
    if (next.nCurves <= 0) {
        next.nCurves = 1;
        next.canonical = 0;
        next.curves[0] = ZoomLensCurve{};
        next.curves[0].vmax = next.vmax;
        next.curves[0].accel = next.accel;
    }
    int i = next.canonical < next.nCurves ? next.canonical : 0;
    if (!o["in"].isNull()) copyTable(o["in"].as<JsonArrayConst>(), next.curves[i].inPts, &next.curves[i].nIn);
    if (!o["out"].isNull()) copyTable(o["out"].as<JsonArrayConst>(), next.curves[i].outPts, &next.curves[i].nOut);
}

bool zoomLensApplyJson(JsonVariantConst v) {
    if (v.isNull() || !v.is<JsonObjectConst>()) return false;
    JsonObjectConst o = v.as<JsonObjectConst>();
    lock();
    ZoomLensProfile next = g_lens;
    unlock();
    if (!o["name"].isNull()) {
        const char *name = o["name"] | "";
        strncpy(next.name, name, sizeof(next.name) - 1);
        next.name[sizeof(next.name) - 1] = '\0';
    }
    if (!o["wide_mm"].isNull()) next.wideMm = o["wide_mm"] | 0.0f;
    if (!o["tele_mm"].isNull()) next.teleMm = o["tele_mm"] | 0.0f;
    if (!o["vmax"].isNull()) next.vmax = o["vmax"] | next.vmax;
    if (!o["accel"].isNull()) next.accel = o["accel"] | next.accel;
    if (!o["decel"].isNull()) next.decel = o["decel"] | next.decel;
    if (!o["vcut"].isNull()) next.vcut = o["vcut"] | next.vcut;
    if (!o["vstatic"].isNull()) next.vstatic = o["vstatic"] | next.vstatic;
    if (!o["repeat_mm"].isNull()) next.repeatMm = o["repeat_mm"] | next.repeatMm;
    if (!o["settle_s"].isNull()) next.settleS = o["settle_s"] | next.settleS;
    if (!o["enabled"].isNull()) next.enabled = o["enabled"] | false;
    if (!o["coupling"].isNull()) next.coupling = o["coupling"] | false;
    if (!o["engage_in"].isNull()) next.engageIn = (uint16_t) (o["engage_in"] | 0);
    if (!o["engage_out"].isNull()) next.engageOut = (uint16_t) (o["engage_out"] | 0);
    if (!o["canonical"].isNull()) next.canonical = (uint8_t) (o["canonical"] | 0);
    next.vmax = fmaxf(50.0f, fminf(8000.0f, next.vmax));
    next.accel = fmaxf(50.0f, fminf(40000.0f, next.accel));
    next.decel = fmaxf(50.0f, fminf(40000.0f, next.decel >= 50.0f ? next.decel : next.accel));
    next.vcut = fmaxf(0.0f, fminf(next.vmax * 0.9f, next.vcut));
    next.vstatic = fmaxf(0.0f, fminf(8000.0f, next.vstatic));
    next.repeatMm = fmaxf(0.0f, fminf(50.0f, next.repeatMm));
    next.settleS = fmaxf(0.4f, fminf(6.0f, next.settleS));
    if (next.wideMm < 0.0f) next.wideMm = 0.0f;
    if (next.teleMm < 0.0f) next.teleMm = 0.0f;
    if (next.engageIn > 4096) next.engageIn = 4096;
    if (next.engageOut > 4096) next.engageOut = 4096;

    if (!o["curves"].isNull() && o["curves"].is<JsonArrayConst>()) {
        next.nCurves = 0;
        for (JsonVariantConst cv : o["curves"].as<JsonArrayConst>()) {
            if (next.nCurves >= kZoomLensMaxCurves) break;
            if (!cv.is<JsonObjectConst>()) continue;
            JsonObjectConst c = cv.as<JsonObjectConst>();
            ZoomLensCurve &dst = next.curves[next.nCurves];
            dst = ZoomLensCurve{};
            dst.vmax = fmaxf(50.0f, fminf(8000.0f, c["vmax"] | next.vmax));
            dst.accel = fmaxf(50.0f, fminf(40000.0f, c["accel"] | next.accel));
            dst.band = (uint8_t) (c["band"] | 0);
            if (!c["in"].isNull()) copyTable(c["in"].as<JsonArrayConst>(), dst.inPts, &dst.nIn);
            if (!c["out"].isNull()) copyTable(c["out"].as<JsonArrayConst>(), dst.outPts, &dst.nOut);
            next.nCurves++;
        }
    } else {
        importLegacyTables(o, next);
    }
    if (next.canonical >= next.nCurves) next.canonical = 0;
    if (next.nCurves > 0) {
        next.vmax = next.curves[next.canonical].vmax;
        next.accel = next.curves[next.canonical].accel;
    }
    lock();
    g_lens = next;
    unlock();
    return true;
}

bool zoomLensAddSample(uint16_t pos, float mm, int dir, float vmax, float accel,
                       int band) {
    if (mm <= 0.0f || mm > 2000.0f) return false;
    pos = clampFocusPos(pos);
    if (band < 0) band = 0;
    if (band > 1) band = 1;
    lock();
    if (band == 0 && g_lens.teleMm > 0.0f && mm > g_lens.teleMm + 0.35f) band = 1;
    int i = ensureCurveLocked(vmax, accel, band);
    if (dir >= 0) insertPoint(g_lens.curves[i].inPts, &g_lens.curves[i].nIn, pos, mm);
    if (dir <= 0) insertPoint(g_lens.curves[i].outPts, &g_lens.curves[i].nOut, pos, mm);
    unlock();
    return true;
}

void zoomLensClearSamples() {
    lock();
    for (int i = 0; i < g_lens.nCurves; i++) {
        g_lens.curves[i].nIn = 0;
        g_lens.curves[i].nOut = 0;
    }
    g_lens.enabled = false;
    unlock();
}

void zoomLensSetEnabled(bool on) {
    lock();
    g_lens.enabled = on;
    unlock();
}

void zoomLensSetMeta(const char *name, float wideMm, float teleMm,
                     float vmax, float accel) {
    lock();
    if (name) {
        strncpy(g_lens.name, name, sizeof(g_lens.name) - 1);
        g_lens.name[sizeof(g_lens.name) - 1] = '\0';
    }
    if (wideMm > 0.0f) g_lens.wideMm = wideMm;
    if (teleMm > 0.0f) g_lens.teleMm = teleMm;
    if (vmax >= 50.0f) g_lens.vmax = fmaxf(50.0f, fminf(8000.0f, vmax));
    if (accel >= 50.0f) g_lens.accel = fmaxf(50.0f, fminf(40000.0f, accel));
    if (g_lens.nCurves > 0 && g_lens.canonical < g_lens.nCurves) {
        if (vmax >= 50.0f) g_lens.curves[g_lens.canonical].vmax = g_lens.vmax;
        if (accel >= 50.0f) g_lens.curves[g_lens.canonical].accel = g_lens.accel;
    }
    unlock();
}

void zoomLensSetCoupling(bool ok, uint16_t engageIn, uint16_t engageOut,
                         float settleS, float vcut) {
    lock();
    g_lens.coupling = ok;
    g_lens.engageIn = engageIn > 4096 ? 4096 : engageIn;
    g_lens.engageOut = engageOut > 4096 ? 4096 : engageOut;
    g_lens.settleS = fmaxf(0.4f, fminf(6.0f, settleS));
    if (vcut >= 0.0f) g_lens.vcut = fmaxf(0.0f, fminf(8000.0f, vcut));
    unlock();
}

bool zoomLensEnsureCurve(float vmax, float accel, bool makeCanonical, int band) {
    lock();
    bool first = g_lens.nCurves == 0;
    int i = ensureCurveLocked(vmax, accel, band);
    bool ok = i >= 0;
    if (ok && (first || makeCanonical)) {
        g_lens.canonical = (uint8_t) i;
        g_lens.vmax = g_lens.curves[i].vmax;
        g_lens.accel = g_lens.curves[i].accel;
    }
    unlock();
    return ok;
}

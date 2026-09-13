// Per-lens zoom map: focus-motor counts <-> camera-displayed focal length.
//
// Optical landings depend on motion history, not just motor position. Stage 1
// stores a canonical motion profile (T_running, T_max, accel, decel, settle).
// Stage 2 records pos<->mm using that profile only, approached from optical
// wide. Digital zoom is a separate curve (band=1) past tele_mm so it cannot
// contaminate the optical table. Go-to then replays the canonical profile.
#pragma once

#include <ArduinoJson.h>
#include <cstdint>

static constexpr int kZoomLensMaxPoints = 12;
static constexpr int kZoomLensMaxCurves = 3;
static constexpr uint8_t kZoomBandOptical = 0;
static constexpr uint8_t kZoomBandDigital = 1;

void zoomLensBegin();
void zoomLensSave();

bool zoomLensEnabled();      // stored flag AND enough samples to apply
bool zoomLensWantEnabled();  // stored checkbox flag only
bool zoomLensCouplingReady();
int zoomLensSampleCount();
bool zoomLensGetRamp(float *vmax, float *accel, float *vcut = nullptr,
                     float *decel = nullptr);
bool zoomLensGetRange(float *wideMm, float *teleMm);
bool zoomLensGetEngage(int dir, int *counts);
bool zoomLensRampForVmax(float hint, float *vmax, float *accel);

// dir: +1 motor counts increasing, -1 decreasing, 0 = average / either table.
bool zoomLensPosFromMm(float mm, int dir, int *pos);
bool zoomLensPosFromMmAt(float mm, int dir, float vmaxHint, int *pos,
                         float *useVmax, float *useAccel);
bool zoomLensMmFromPos(int pos, int dir, float *mm);
bool zoomLensMmAtOptical(float t, float *mm);  // t=0 wide .. 1 tele

void zoomLensFillJson(JsonObject obj);
bool zoomLensApplyJson(JsonVariantConst v);     // replace profile (PUT)
bool zoomLensAddSample(uint16_t pos, float mm, int dir, float vmax, float accel,
                       int band = 0);
void zoomLensClearSamples();
void zoomLensSetEnabled(bool on);
void zoomLensSetMeta(const char *name, float wideMm, float teleMm,
                     float vmax, float accel);
void zoomLensSetCoupling(bool ok, uint16_t engageIn, uint16_t engageOut,
                         float settleS, float vcut = 0.0f);
bool zoomLensEnsureCurve(float vmax, float accel, bool makeCanonical,
                         int band = 0);

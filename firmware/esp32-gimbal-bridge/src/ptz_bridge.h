// IP PTZ protocol front-end for hardware joystick/keyboard controllers.
//
// Speaks the camera-side of the protocols those boxes already know, and maps
// pan/tilt/zoom onto the DJI R SDK session in main.cpp:
//
//   VISCA over IP     UDP 52381   Sony 8-byte header + VISCA payload
//   VISCA raw         UDP 1259    PTZOptics / generic (no IP header)
//   VISCA raw         TCP 5678    PTZOptics / generic
//   Pelco-D / Pelco-P UDP+TCP 4000
//   Panasonic AW      UDP 49152   #PTS / #Z / #APC / #R / #O
//   HTTP CGI          :80         PTZOptics ptzctrl, Sony ptzf, Panasonic aw_ptz
//
// ONVIF is not implemented: SOAP + WS-Discovery is too heavy for the C3, and
// dedicated PTZ hardware almost never speaks it (that's a VMS path).
#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

class AsyncWebServer;

struct PtzSink {
    // Continuous rates in deg/s. Hold until stop() or a later speed().
    void (*speed)(float yawDps, float rollDps, float pitchDps);
    void (*stop)();
    void (*position)(float yawDeg, float rollDeg, float pitchDeg, float timeS);
    void (*zoomAbs)(int position);       // 0-4096 focus-motor counts
    void (*zoomRate)(float signedRate);  // -1 wide .. +1 tele, 0 = freeze
    void (*home)();
    void (*sleep)();
    void (*wake)();
    void (*recStart)();
    void (*recStop)();
    bool (*getAttitude)(float *yawDeg, float *rollDeg, float *pitchDeg);
    bool (*getZoom)(int *position);
};

void ptzBridgeBegin(const PtzSink &sink);
void ptzBridgeRegisterHttp(AsyncWebServer &server);
void ptzBridgeFillStatus(JsonObject obj);
void ptzBridgeSetMaxRate(float dps);
float ptzBridgeMaxRate();
void ptzBridgeSetInvert(bool pan, bool tilt);

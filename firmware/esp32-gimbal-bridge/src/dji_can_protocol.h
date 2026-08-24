// DJI R SDK gimbal packet framing (CmdSet 0x0E) — ported from dji_gimbal_cli.py.
//
// This header is transport-agnostic: it only builds/parses the SDK packet bytes.
// The caller (main.cpp) is responsible for fragmenting into 8-byte CAN frames
// on the way out, and reassembling frames back into packets on the way in
// (see Reassembler below, which mirrors the Python `reassemble()` state machine).
#pragma once

#include <Arduino.h>
#include <vector>

namespace dji {

constexpr uint8_t SOF = 0xAA;
constexpr uint8_t CMD_SET_GIMBAL = 0x0E;
constexpr uint8_t CMD_TYPE_REPLY_REQUIRED = 0x03;

constexpr uint8_t RET_SUCCESS = 0x00;
constexpr uint8_t RET_PARSE_ERROR = 0x01;
constexpr uint8_t RET_EXEC_FAIL = 0x02;
constexpr uint8_t RET_UNDEFINED = 0xFF;

// CmdSet 0x0E command IDs (see docs/DJI_R_SDK_Protocol.md).
constexpr uint8_t CMD_ID_POSITION = 0x00;
constexpr uint8_t CMD_ID_SPEED = 0x01;
constexpr uint8_t CMD_ID_ANGLE = 0x02;
constexpr uint8_t CMD_ID_LIMIT_ANGLE = 0x04;
constexpr uint8_t CMD_ID_STIFFNESS = 0x06;
constexpr uint8_t CMD_ID_PUSH_ENABLE = 0x07;
constexpr uint8_t CMD_ID_PUSH_PARAMS = 0x08; // unsolicited
constexpr uint8_t CMD_ID_MODULE_VERSION = 0x09;
constexpr uint8_t CMD_ID_USER_PARAMS = 0x0B;
constexpr uint8_t CMD_ID_RECENTER_SELFIE = 0x0E;
constexpr uint8_t CMD_ID_CALIB_STATUS = 0x10; // unsolicited
constexpr uint8_t CMD_ID_ACTIVETRACK = 0x11;
constexpr uint8_t CMD_ID_FOCUS = 0x12;

// ---- CRC (poly 0x8005 / init 0x3AA3 for CRC-16; poly 0x04C11DB7 / init 0x00003AA3
//      for CRC-32; both reflected in/out) — identical tables to dji_gimbal_cli.py.
uint16_t crc16(const uint8_t *data, size_t len);
uint32_t crc32(const uint8_t *data, size_t len);

// ---- Sequence counter (mirrors Python's wraparound at 0xFFFD -> 0x0002) ----
uint16_t nextSeq();

// ---- Packet builders. Each returns a complete SDK packet (SOF..CRC32). ----
std::vector<uint8_t> buildSdkPacket(uint8_t cmdSet, uint8_t cmdId,
                                     const uint8_t *cmdData, size_t cmdDataLen,
                                     uint8_t cmdType = CMD_TYPE_REPLY_REQUIRED);

std::vector<uint8_t> buildObtainGimbalAngle(uint8_t dataTypeByte = 0x01); // 0x01=attitude, 0x02=joint
std::vector<uint8_t> buildControlPosition(float yawDeg, float rollDeg, float pitchDeg,
                                           bool absolute = true, float timeS = 0.2f);
std::vector<uint8_t> buildControlSpeed(float yawDegPerS, float rollDegPerS, float pitchDegPerS);
std::vector<uint8_t> buildRecenterSelfie(uint8_t mode); // 0x01=recenter, 0x02=selfie
std::vector<uint8_t> buildActiveTrackToggle();
std::vector<uint8_t> buildSetParameterPush(bool enable);
std::vector<uint8_t> buildFocusSet(uint16_t position); // 0-4096
std::vector<uint8_t> buildFocusGet();

// ---- Reply / push parsing ----
struct SdkReply {
    uint8_t cmdSet = 0;
    uint8_t cmdId = 0;
    uint8_t retCode = 0;
    const uint8_t *data = nullptr; // points into the packet buffer passed to validateSdkReply
    size_t dataLen = 0;
};

// Validates SOF, length, reply bit (byte3 & 0x20), CRC-16, CRC-32. `packet` must
// stay alive as long as `out.data` is used.
bool validateSdkReply(const uint8_t *packet, size_t len, SdkReply &out);

struct GimbalAngles {
    float yawDeg;
    float rollDeg;
    float pitchDeg;
};

// Works for both the CMD_ID_ANGLE reply and the CMD_ID_PUSH_PARAMS unsolicited push
// (same payload layout: data_type(1), yaw(2), roll(2), pitch(2), all int16 in 0.1deg).
bool parseGimbalAngles(const SdkReply &reply, GimbalAngles &out);

// Focus reply: position is the last 4 bytes of `data` as little-endian uint32
// (see docs/DJI_R_SDK_Protocol.md §19 — empirically derived, not from official docs).
bool parseFocusPosition(const SdkReply &reply, uint32_t &position);

// ---- Frame reassembler: feed raw CAN frame payloads (<=8 bytes each) here.
// Mirrors the `reassemble()` state machine in dji_gimbal_cli.py exactly, including
// the early CRC-16 check once 12 bytes are buffered (SOF..CRC16) before continuing
// to accumulate the rest of the packet. ----
class Reassembler {
public:
    // Returns true and fills `out` with a complete, CRC32-valid packet once one
    // finishes; otherwise returns false (still accumulating, or dropped a bad frame).
    bool feed(const uint8_t *frameData, size_t frameLen, std::vector<uint8_t> &out);

private:
    std::vector<uint8_t> buf_;
    int step_ = 0;
    int packLen_ = 0;
};

} // namespace dji

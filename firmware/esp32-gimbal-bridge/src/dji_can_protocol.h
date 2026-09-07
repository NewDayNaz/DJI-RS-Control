// DJI R SDK gimbal (CmdSet 0x0E) and camera (CmdSet 0x0D) packet framing — ported
// from dji_gimbal_cli.py.
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
constexpr uint8_t CMD_SET_CAMERA = 0x0D;
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
constexpr uint8_t CMD_ID_SLEEP_WAKE = 0x0C;
constexpr uint8_t CMD_ID_RECENTER_SELFIE = 0x0E;
constexpr uint8_t CMD_ID_AUTOTUNE = 0x0F;
constexpr uint8_t CMD_ID_CALIB_STATUS = 0x10; // unsolicited
constexpr uint8_t CMD_ID_ACTIVETRACK = 0x11;
constexpr uint8_t CMD_ID_FOCUS = 0x12;

// CmdSet 0x0D command IDs.
constexpr uint8_t CMD_ID_CAMERA_CTRL = 0x00; // record / focus-center
constexpr uint8_t CMD_ID_CAMERA_CMD = 0x01;  // camera query (data 01, reply byte 0x02)

// Speed-control takeover byte: bit 7 set. A working CAN client uses 0x80;
// older notes (and pre-golden firmware) used 0x88.
constexpr uint8_t SPEED_CTRL_TAKEOVER = 0x80;

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
std::vector<uint8_t> buildObtainModuleVersion(uint32_t deviceId = 0x00000001); // 0x01 = DJI R SDK
std::vector<uint8_t> buildObtainGimbalLimitAngle(uint8_t query = 0x01);
std::vector<uint8_t> buildObtainMotorStiffness();
std::vector<uint8_t> buildObtainGimbalUserParams();
std::vector<uint8_t> buildObtainGimbalUserParamsPoll(); // cmd_type 0x02, data 00 22 23
// ctrl bits: bit0=absolute(1)/incremental(0); bits 1,2,3 = yaw/roll/pitch invalid (1=invalid).
std::vector<uint8_t> buildControlPosition(float yawDeg, float rollDeg, float pitchDeg,
                                           bool absolute = true, float timeS = 0.2f,
                                           bool yawValid = true, bool rollValid = true,
                                           bool pitchValid = true);
std::vector<uint8_t> buildControlSpeed(float yawDegPerS, float rollDegPerS, float pitchDegPerS,
                                        uint8_t ctrl = SPEED_CTRL_TAKEOVER);
std::vector<uint8_t> buildRecenterSelfie(uint8_t mode); // 0x01=recenter, 0x02=selfie
std::vector<uint8_t> buildActiveTrackToggle();
std::vector<uint8_t> buildSetParameterPush(bool enable);
std::vector<uint8_t> buildSleep(); // 0x0E/0x0C 23 01 01
std::vector<uint8_t> buildWake();  // 0x0E/0x0C 23 01 00
std::vector<uint8_t> buildCalibrate(); // AutoTune gimbal motors (0x0E/0x0F 00 01 01)
std::vector<uint8_t> buildMotorCalibrate(); // focus-motor autocal (0x0E/0x12 02 00 01)
std::vector<uint8_t> buildRecordStart();       // 0x0D/0x00 03 00
std::vector<uint8_t> buildRecordStop();        // 0x0D/0x00 04 00
std::vector<uint8_t> buildFocusCenterStart();  // 0x0D/0x00 05 00
std::vector<uint8_t> buildFocusCenterStop();   // 0x0D/0x00 0B 00
std::vector<uint8_t> buildCameraCmd();         // 0x0D/0x01 01
std::vector<uint8_t> buildFocusSet(uint16_t position, uint8_t cmdSubId = 0x01,
                                    uint8_t ctlType = 0x00, uint8_t dataLength = 0x02); // 0-4096
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

// Parses the CMD_ID_ANGLE reply (payload: data_type(1), yaw(2), roll(2), pitch(2),
// all int16 in 0.1deg). Requires retCode == RET_SUCCESS.
bool parseGimbalAngles(const SdkReply &reply, GimbalAngles &out);

// Parses the CMD_ID_PUSH_PARAMS unsolicited push. Byte 14 (SdkReply.retCode) is a
// flags field on pushes, not a return code: if bit0 is set and >=6 data bytes follow,
// the angles start at data[0]; otherwise fall back to the angle-reply layout
// (data_type byte + 3 x int16). Mirrors _push_angles() in dji_can_session.py.
bool parsePushAngles(const SdkReply &reply, GimbalAngles &out);

// Focus reply: position is the last 4 bytes of `data` as little-endian uint32
// (see docs/DJI_R_SDK_Protocol.md §19 — empirically derived, not from official docs).
bool parseFocusPosition(const SdkReply &reply, uint32_t &position);

// Module version reply (0x0E/0x09): device_id(4 LE) + version(4 LE).
// versionUint 0xAABBCCDD renders as "AA.BB.CC.DD" hex digits, e.g. 0x01020304 -> 1.2.3.4.
bool parseModuleVersion(const SdkReply &reply, uint32_t &deviceId, uint32_t &versionUint);
void formatModuleVersion(uint32_t versionUint, char *out, size_t outLen);

struct GimbalLimits {
    float yawMin, yawMax, rollMin, rollMax, pitchMin, pitchMax;
};

// Limit angle reply (0x0E/0x04): 6 x int16 in 0.1deg, either bare (12 bytes) or with a
// 1-byte query prefix (13 bytes). Returns false for other shapes (e.g. 6-byte compact).
bool parseLimitAngles(const SdkReply &reply, GimbalLimits &out);

// Human-readable DJI return code ("success", "command parse error", ...).
const char *returnCodeStr(uint8_t code);

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

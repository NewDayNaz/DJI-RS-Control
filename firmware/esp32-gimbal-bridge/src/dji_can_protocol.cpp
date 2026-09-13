#include "dji_can_protocol.h"

#include <cstring>

namespace dji {

// ---- CRC-16 table: poly 0x8005, init 0x3AA3, reflected in/out ----
// Identical to CRC16_TABLE in dji_gimbal_cli.py.
static const uint16_t CRC16_TABLE[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
};

// ---- CRC-32 table: poly 0x04C11DB7, init 0x00003AA3, reflected in/out ----
// Identical to CRC32_TABLE in dji_gimbal_cli.py.
static const uint32_t CRC32_TABLE[256] = {
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
    0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988, 0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
    0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
    0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172, 0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
    0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
    0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
    0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
    0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E, 0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
    0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
    0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0, 0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
    0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
    0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A, 0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
    0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
    0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC, 0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
    0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
    0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
    0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236, 0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
    0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
    0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38, 0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
    0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
    0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2, 0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
    0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
    0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
    0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94, 0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D,
};

static uint16_t crc16Update(uint16_t crc, const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uint8_t idx = (uint8_t) (crc ^ data[i]);
        crc = (uint16_t) (CRC16_TABLE[idx] ^ (crc >> 8));
    }
    return crc;
}

static uint32_t crc32Update(uint32_t crc, const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uint8_t idx = (uint8_t) (crc ^ data[i]);
        crc = CRC32_TABLE[idx] ^ (crc >> 8);
    }
    return crc;
}

uint16_t crc16(const uint8_t *data, size_t len) {
    return crc16Update(0x3AA3, data, len);
}

uint32_t crc32(const uint8_t *data, size_t len) {
    return crc32Update(0x00003AA3, data, len);
}

// ---- Sequence counter ----
static uint16_t g_seq = 0x2210;

uint16_t nextSeq() {
    if (g_seq >= 0xFFFD) {
        g_seq = 0x0002;
    }
    g_seq += 1;
    return g_seq;
}

// ---- Packet builders ----
std::vector<uint8_t> buildSdkPacket(uint8_t cmdSet, uint8_t cmdId,
                                     const uint8_t *cmdData, size_t cmdDataLen,
                                     uint8_t cmdType) {
    const size_t prefixLen = 10;
    const size_t crc16Len = 2;
    const size_t crc32Len = 4;
    const size_t dataSegLen = 2 + cmdDataLen; // cmd_set + cmd_id + cmdData
    const size_t cmdLength = prefixLen + crc16Len + dataSegLen + crc32Len;

    uint16_t seq = nextSeq();

    std::vector<uint8_t> pkt;
    pkt.reserve(cmdLength);
    pkt.push_back(SOF);
    pkt.push_back((uint8_t) (cmdLength & 0xFF));
    pkt.push_back((uint8_t) ((cmdLength >> 8) & 0xFF));
    pkt.push_back(cmdType);
    pkt.push_back(0x00); // ENC
    pkt.push_back(0x00);
    pkt.push_back(0x00);
    pkt.push_back(0x00); // RES x3
    pkt.push_back((uint8_t) (seq & 0xFF));
    pkt.push_back((uint8_t) ((seq >> 8) & 0xFF));

    uint16_t c16 = crc16(pkt.data(), pkt.size());
    pkt.push_back((uint8_t) (c16 & 0xFF));
    pkt.push_back((uint8_t) ((c16 >> 8) & 0xFF));

    pkt.push_back(cmdSet);
    pkt.push_back(cmdId);
    for (size_t i = 0; i < cmdDataLen; i++) {
        pkt.push_back(cmdData[i]);
    }

    uint32_t c32 = crc32(pkt.data(), pkt.size());
    pkt.push_back((uint8_t) (c32 & 0xFF));
    pkt.push_back((uint8_t) ((c32 >> 8) & 0xFF));
    pkt.push_back((uint8_t) ((c32 >> 16) & 0xFF));
    pkt.push_back((uint8_t) ((c32 >> 24) & 0xFF));

    return pkt;
}

std::vector<uint8_t> buildObtainGimbalAngle(uint8_t dataTypeByte) {
    uint8_t d[1] = {dataTypeByte};
    return buildSdkPacket(CMD_SET_GIMBAL, CMD_ID_ANGLE, d, 1);
}

std::vector<uint8_t> buildObtainModuleVersion(uint32_t deviceId) {
    uint8_t d[4];
    memcpy(d, &deviceId, 4);
    return buildSdkPacket(CMD_SET_GIMBAL, CMD_ID_MODULE_VERSION, d, sizeof(d));
}

std::vector<uint8_t> buildObtainGimbalLimitAngle(uint8_t query) {
    // A working CAN client sends DATA 01; empty DATA is also accepted by some devices.
    uint8_t d[1] = {query};
    return buildSdkPacket(CMD_SET_GIMBAL, CMD_ID_LIMIT_ANGLE, d, 1);
}

std::vector<uint8_t> buildObtainMotorStiffness() {
    return buildSdkPacket(CMD_SET_GIMBAL, CMD_ID_STIFFNESS, nullptr, 0);
}

std::vector<uint8_t> buildObtainGimbalUserParams() {
    return buildSdkPacket(CMD_SET_GIMBAL, CMD_ID_USER_PARAMS, nullptr, 0);
}

std::vector<uint8_t> buildObtainGimbalUserParamsPoll() {
    // Periodic user-params poll used by a working CAN client: CMD_TYPE 0x02 (not
    // reply-required), DATA 00 22 23.
    uint8_t d[3] = {0x00, 0x22, 0x23};
    return buildSdkPacket(CMD_SET_GIMBAL, CMD_ID_USER_PARAMS, d, sizeof(d), 0x02);
}

std::vector<uint8_t> buildControlPosition(float yawDeg, float rollDeg, float pitchDeg,
                                           bool absolute, float timeS,
                                           bool yawValid, bool rollValid, bool pitchValid) {
    int16_t yaw = (int16_t) lroundf(yawDeg * 10.0f);
    int16_t roll = (int16_t) lroundf(rollDeg * 10.0f);
    int16_t pitch = (int16_t) lroundf(pitchDeg * 10.0f);
    uint8_t ctrl = absolute ? 0x01 : 0x00;
    if (!yawValid) ctrl |= 0x02;
    if (!rollValid) ctrl |= 0x04;
    if (!pitchValid) ctrl |= 0x08;
    uint8_t timeByte = (uint8_t) constrain((int) lroundf(timeS * 10.0f), 0, 255);

    uint8_t d[8];
    memcpy(&d[0], &yaw, 2);
    memcpy(&d[2], &roll, 2);
    memcpy(&d[4], &pitch, 2);
    d[6] = ctrl;
    d[7] = timeByte;
    return buildSdkPacket(CMD_SET_GIMBAL, CMD_ID_POSITION, d, sizeof(d));
}

std::vector<uint8_t> buildControlSpeed(float yawDegPerS, float rollDegPerS, float pitchDegPerS,
                                        uint8_t ctrl) {
    int16_t yaw = (int16_t) constrain((int) lroundf(yawDegPerS * 10.0f), -3600, 3600);
    int16_t roll = (int16_t) constrain((int) lroundf(rollDegPerS * 10.0f), -3600, 3600);
    int16_t pitch = (int16_t) constrain((int) lroundf(pitchDegPerS * 10.0f), -3600, 3600);

    uint8_t d[7];
    memcpy(&d[0], &yaw, 2);
    memcpy(&d[2], &roll, 2);
    memcpy(&d[4], &pitch, 2);
    d[6] = ctrl;
    return buildSdkPacket(CMD_SET_GIMBAL, CMD_ID_SPEED, d, sizeof(d));
}

std::vector<uint8_t> buildRecenterSelfie(uint8_t mode) {
    uint8_t d[2] = {0xFE, mode};
    return buildSdkPacket(CMD_SET_GIMBAL, CMD_ID_RECENTER_SELFIE, d, sizeof(d));
}

std::vector<uint8_t> buildActiveTrackToggle() {
    uint8_t d[1] = {0x03};
    return buildSdkPacket(CMD_SET_GIMBAL, CMD_ID_ACTIVETRACK, d, sizeof(d));
}

std::vector<uint8_t> buildSetParameterPush(bool enable) {
    uint8_t d[1] = {(uint8_t) (enable ? 0x01 : 0x00)};
    return buildSdkPacket(CMD_SET_GIMBAL, CMD_ID_PUSH_ENABLE, d, sizeof(d));
}

std::vector<uint8_t> buildSleep() {
    uint8_t d[3] = {0x23, 0x01, 0x01};
    return buildSdkPacket(CMD_SET_GIMBAL, CMD_ID_SLEEP_WAKE, d, sizeof(d));
}

std::vector<uint8_t> buildWake() {
    uint8_t d[3] = {0x23, 0x01, 0x00};
    return buildSdkPacket(CMD_SET_GIMBAL, CMD_ID_SLEEP_WAKE, d, sizeof(d));
}

std::vector<uint8_t> buildCalibrate() {
    uint8_t d[3] = {0x00, 0x01, 0x01};
    return buildSdkPacket(CMD_SET_GIMBAL, CMD_ID_AUTOTUNE, d, sizeof(d));
}

std::vector<uint8_t> buildMotorCalibrate() {
    // Focus-motor autocalibration: finds lens endpoints; required before zoom/focus-set.
    uint8_t d[3] = {0x02, 0x00, 0x01};
    return buildSdkPacket(CMD_SET_GIMBAL, CMD_ID_FOCUS, d, sizeof(d));
}

std::vector<uint8_t> buildRecordStart() {
    uint8_t d[2] = {0x03, 0x00};
    return buildSdkPacket(CMD_SET_CAMERA, CMD_ID_CAMERA_CTRL, d, sizeof(d));
}

std::vector<uint8_t> buildRecordStop() {
    uint8_t d[2] = {0x04, 0x00};
    return buildSdkPacket(CMD_SET_CAMERA, CMD_ID_CAMERA_CTRL, d, sizeof(d));
}

std::vector<uint8_t> buildFocusCenterStart() {
    uint8_t d[2] = {0x05, 0x00};
    return buildSdkPacket(CMD_SET_CAMERA, CMD_ID_CAMERA_CTRL, d, sizeof(d));
}

std::vector<uint8_t> buildFocusCenterStop() {
    uint8_t d[2] = {0x0B, 0x00};
    return buildSdkPacket(CMD_SET_CAMERA, CMD_ID_CAMERA_CTRL, d, sizeof(d));
}

std::vector<uint8_t> buildCameraCmd() {
    uint8_t d[1] = {0x01};
    return buildSdkPacket(CMD_SET_CAMERA, CMD_ID_CAMERA_CMD, d, sizeof(d));
}

std::vector<uint8_t> buildFocusSet(uint16_t position, uint8_t cmdSubId,
                                    uint8_t ctlType, uint8_t dataLength) {
    // The motor-calib span is documented as 0–4096, but the gimbal ignores the
    // exact endpoints: 0 is treated as empty, and 4096 does not fit in 12 bits
    // (max 4095). Command 1 and 4095 instead; that still reaches the buffered
    // digital-tele / past-wide stops.
    if (position < 1) position = 1;
    if (position > 4095) position = 4095;
    uint8_t d[5];
    d[0] = cmdSubId;
    d[1] = ctlType;
    d[2] = dataLength;
    memcpy(&d[3], &position, 2);
    return buildSdkPacket(CMD_SET_GIMBAL, CMD_ID_FOCUS, d, sizeof(d));
}

std::vector<uint8_t> buildFocusGet() {
    uint8_t d[2] = {0x15, 0x00};
    return buildSdkPacket(CMD_SET_GIMBAL, CMD_ID_FOCUS, d, sizeof(d));
}

// ---- Reply parsing ----
bool validateSdkReply(const uint8_t *packet, size_t len, SdkReply &out) {
    if (len < 16) return false;
    if (packet[0] != SOF) return false;
    if ((packet[3] & 0x20) == 0) return false; // not a reply/push
    size_t packLen = packet[1] | ((packet[2] & 0x03) << 8);
    if (len != packLen) return false;
    if (crc16(packet, 10) != (uint16_t) (packet[10] | (packet[11] << 8))) return false;
    uint32_t storedCrc32;
    memcpy(&storedCrc32, packet + len - 4, 4);
    if (crc32(packet, len - 4) != storedCrc32) return false;

    out.cmdSet = packet[12];
    out.cmdId = packet[13];
    out.retCode = packet[14];
    out.data = packet + 15;
    out.dataLen = len - 4 - 15;
    return true;
}

static bool anglesFromInt16s(const uint8_t *blob, size_t len, size_t offset, GimbalAngles &out) {
    if (offset + 6 > len) return false;
    int16_t yaw, roll, pitch;
    memcpy(&yaw, blob + offset, 2);
    memcpy(&roll, blob + offset + 2, 2);
    memcpy(&pitch, blob + offset + 4, 2);
    out.yawDeg = yaw * 0.1f;
    out.rollDeg = roll * 0.1f;
    out.pitchDeg = pitch * 0.1f;
    return true;
}

bool parseGimbalAngles(const SdkReply &reply, GimbalAngles &out) {
    if (reply.retCode != RET_SUCCESS) return false;
    // data = data_type(1), yaw(2), roll(2), pitch(2)
    return anglesFromInt16s(reply.data, reply.dataLen, 1, out);
}

bool parsePushAngles(const SdkReply &reply, GimbalAngles &out) {
    // Byte 14 is flags on unsolicited push (bit 0 = angles present), then 3 x int16.
    // Some 26-byte pushes use the angle-reply layout (data_type + 3 x int16) instead.
    if ((reply.retCode & 0x01) && reply.dataLen >= 6) {
        if (anglesFromInt16s(reply.data, reply.dataLen, 0, out)) return true;
    }
    if (reply.dataLen >= 7) {
        return anglesFromInt16s(reply.data, reply.dataLen, 1, out);
    }
    return false;
}

bool parseFocusPosition(const SdkReply &reply, uint32_t &position) {
    if (reply.retCode != RET_SUCCESS) return false;
    if (reply.dataLen < 4) return false;
    memcpy(&position, reply.data + reply.dataLen - 4, 4);
    return true;
}

bool parseModuleVersion(const SdkReply &reply, uint32_t &deviceId, uint32_t &versionUint) {
    if (reply.retCode != RET_SUCCESS) return false;
    if (reply.dataLen < 8) return false;
    memcpy(&deviceId, reply.data, 4);
    memcpy(&versionUint, reply.data + 4, 4);
    return true;
}

void formatModuleVersion(uint32_t versionUint, char *out, size_t outLen) {
    snprintf(out, outLen, "%lX.%02lX.%02lX.%02lX",
             (unsigned long) ((versionUint >> 24) & 0xFF),
             (unsigned long) ((versionUint >> 16) & 0xFF),
             (unsigned long) ((versionUint >> 8) & 0xFF),
             (unsigned long) (versionUint & 0xFF));
}

bool parseLimitAngles(const SdkReply &reply, GimbalLimits &out) {
    if (reply.retCode != RET_SUCCESS) return false;
    // Bare: 6 x int16 (12 bytes). Prefixed: query byte + 6 x int16 (13 bytes) — prefer
    // the prefixed layout at 13 so the query byte is not read as yaw_min.
    size_t offset;
    if (reply.dataLen == 13) {
        offset = 1;
    } else if (reply.dataLen >= 12) {
        offset = 0;
    } else {
        return false;
    }
    int16_t v[6];
    for (int i = 0; i < 6; i++) {
        memcpy(&v[i], reply.data + offset + i * 2, 2);
    }
    out.yawMin = v[0] * 0.1f;
    out.yawMax = v[1] * 0.1f;
    out.rollMin = v[2] * 0.1f;
    out.rollMax = v[3] * 0.1f;
    out.pitchMin = v[4] * 0.1f;
    out.pitchMax = v[5] * 0.1f;
    return true;
}

const char *returnCodeStr(uint8_t code) {
    switch (code) {
        case RET_SUCCESS: return "success";
        case RET_PARSE_ERROR: return "command parse error";
        case RET_EXEC_FAIL: return "command execution failed";
        case RET_UNDEFINED: return "undefined error";
        default: return "unknown";
    }
}

// ---- Reassembler (mirrors reassemble() in dji_gimbal_cli.py) ----
bool Reassembler::feed(const uint8_t *frameData, size_t frameLen, std::vector<uint8_t> &out) {
    for (size_t i = 0; i < frameLen; i++) {
        uint8_t b = frameData[i];
        switch (step_) {
            case 0:
                if (b == SOF) {
                    buf_.clear();
                    buf_.push_back(b);
                    step_ = 1;
                }
                break;
            case 1:
                packLen_ = b;
                buf_.push_back(b);
                step_ = 2;
                break;
            case 2:
                packLen_ |= (b & 0x03) << 8;
                buf_.push_back(b);
                step_ = 3;
                break;
            case 3:
                buf_.push_back(b);
                if (buf_.size() == 12) {
                    uint16_t stored = (uint16_t) (buf_[10] | (buf_[11] << 8));
                    if (crc16(buf_.data(), 10) == stored) {
                        step_ = 4;
                    } else {
                        step_ = 0;
                        buf_.clear();
                    }
                }
                break;
            case 4:
                buf_.push_back(b);
                if ((int) buf_.size() == packLen_) {
                    step_ = 0;
                    uint32_t storedCrc32;
                    memcpy(&storedCrc32, buf_.data() + buf_.size() - 4, 4);
                    bool ok = crc32(buf_.data(), buf_.size() - 4) == storedCrc32;
                    if (ok) {
                        out = buf_;
                    }
                    buf_.clear();
                    if (ok) return true;
                }
                break;
        }
    }
    return false;
}

} // namespace dji

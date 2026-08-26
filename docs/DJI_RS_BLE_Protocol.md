# DJI RS BLE Protocol (DUML / V1 Transport)

This document describes how the official Ronin Android app talks to RS-series gimbals over Bluetooth Low Energy, and how that path maps onto the accessory-port CAN protocol in `docs/DJI_R_SDK_Protocol.md`.

BLE is **not** a tunnel for the CAN `0xAA` R SDK packets. The phone sends DJI V1 / DUML frames (`SOF 0x55`) on GATT. The gimbal firmware accepts both transports and implements overlapping features with different wrappers:

| | Accessory CAN | Phone BLE |
|---|---|---|
| Physical | RSA / 4-pin CAN at 1 Mbps | BLE 4.x/5, service `FFF0` |
| Packet SOF | `0xAA` | `0x55` |
| Gimbal command set | `0x0E` | `0x04` |
| Camera command set | `0x0D` | DUML camera / third-party camera (not captured yet on RS 5) |
| Header CRC | CRC-16 init `0x3AA3` | CRC8 init `0x77` over bytes 0–2 |
| Body CRC | CRC-32 init `0x3AA3` | CRC16 init `0x3692` |
| Host implementation | `dji_gimbal_cli.py` | `dji_v1.py` + `dji_ble_cli.py` |

Encodings marked **verified** were rebuilt from official-app HCI captures and match `dji_v1.pack()` byte-for-byte. Native type names come from Ghidra on `mobile/lib/arm64-v8a/libdjibase.so`. Command-id names in section 8 that are not verified on RS 5 are labelled **public DUML** (o-gs `dji-dumlv1-gimbal.lua`) or **SDK key** (string table in `libdjibase.so`).

---

## 0. Analysis environment

### 0.1 What is already set up

| Tool | Location / state |
|---|---|
| Ghidra 12.1.3 | `C:\Tools\ghidra_12.1.3_PUBLIC` |
| Ghidra MCP | Project `DJI_Ronin_App` at `C:\Users\JonBons\DJI_Ronin_App`, TCP `http://127.0.0.1:8089` |
| Imported programs | `/arm64/libdjibase.so`, `/arm64/libSDKRelativeJNI.so`, `/arm64/libdji_innertools.so` (open **one** at a time; djibase + innertools together OOMs) |
| jadx 1.5.6 | `C:\Tools\jadx\jadx-gui-1.5.6.exe` and `lib\jadx-gui-1.5.6-all.jar` |
| Extracted APK bits | `mobile/lib/arm64-v8a/*.so`, `mobile/assets/` |
| APK | `DJI-v1.8.4-6192-official-sec.apk` (AppGuard-packed) |

CLI for jadx (the GUI jar includes the CLI):

```bash
"/c/Tools/jadx/jre/bin/java.exe" -cp "/c/Tools/jadx/lib/jadx-gui-1.5.6-all.jar" \
  jadx.cli.JadxCLI -h
```

### 0.2 AppGuard packing (why jadx cannot see midware)

`classes.dex` is a 30 MB AppGuard stub. The DEX header reports **5 class_defs** (`Lcom/AppGuard/AppGuard/...`) and 268 strings. The real Java (`dji.midware.ble2.*`, `CmdIdGimbal`, `DataGimbalControl`, …) is encrypted and unpacked at runtime by `libAppGuard.so` / `libappsec.so`.

String scans of the encrypted blob still show the original class names, which is enough to know the Java stack exists:

- `dji.midware.ble2.BleConstants` / `BleManager` / `BluetoothService` / `base.BleDevice`
- `dji.midware.data.config.P3.CmdIdGimbal$CmdIdType`
- `dji.midware.data.model.P3.DataGimbalControl`, `DataGimbalSpeedControl`, `DataGimbalNewResetAndSetMode`, `DataGimbalFocusControl`, `DataThirdPartyCameraSetAction`

Dumping the decrypted DEX needs a runtime dump from a device that has already unpacked the app. Do not expect `jadx` on the official APK file to produce those classes.

BLE framing and the object-model types live in native code, which **is** in the extracted `.so` files.

### 0.3 Native libraries reviewed

All under `mobile/lib/arm64-v8a/`. Open only `libdji_innertools.so` in Ghidra when working the encoder (do not keep `libdjibase.so` loaded).

**`libdjibase.so`.** Object-model types (`GimbalAngleRotation`, `GimbalSpeedRotation`, `GimbalResetCommandMsg`), MSDK key names, CRC tables, `dji::sdk::V1PackHeader` JSON fields.

**`libSDKRelativeJNI.so`.** Not BLE. `dji/midware/natives/SDKRelativeJNI` registers geo/fly-forbid, Airmap, license, upgrade-URL, and SDK-activation getters. Skip for protocol work.

**`libdji_innertools.so`.** This is the V1 codec the app actually packs with.

| Symbol (Ghidra addr, image base `00100000`) | Role |
|---|---|
| `dji::inner::DjiProtocolEncoder::Encode` `@ 0040c2c4` | Packs `dji_cmd_req` to wire. Compact V1 matches `dji_v1.pack()`. |
| `DjiProtocolDecoder::DecodeCommand` `@ 0040ab78` | Unpacks SOF `0x55`, version 1 compact / version 2 extended. |
| `GetFullCmdId` `@ 0040c628` | `(cmdType << 30) \| (cmdSet << 16) \| cmdId` |
| `NeedAck` `@ 0040c5f0` | Ack when cmd type is 0 or 1 (with extra rules for type==1). |
| `KeyInfoMgr::getKeyInfoList` `@ 00418018` | Catalog of key **names** (`SDKKeyInfo`), not cmd_set/cmd_id. |
| `CmdWatchMgr`, `CSDKCmdCenterMgr::SendV1`, `ActionSendPack` | Debug tap / inject of V1 packets. |

String blob around file offset `0x003caf9d` labels the keys: `ResetGimbal`, `RotateByAngle` (“Rotate gimbal with angle value”), `RotateBySpeed` (“Rotate gimbal with speed”), `TurnOnGimbal` / `TurnOffGimbal` / `DormantGimbal`, `JoystickControlSpeed`, `SleepNegotiate`, `StartAdvancedGimbalCalibrate`, `BLEMacAddress`.

The full key-name region in `libdji_innertools.so` (`0x3caf00`–`0x3cc950`, ~129 names) is the complete RS 5 gimbal surface. Beyond the doc set it adds `GimbalSceneMode`, `GimbalRollMode`, `FineTune{Pitch,Roll,Yaw}InDegrees`, `{Pitch,Yaw}SmoothingFactor(+Range)`, `{Pitch,Yaw}ControllerMaxSpeed`, `GimbalVerticalShotEnabled`, `AxisLockSwitch`, `StartGimbal{TimeLapse,PanoMission,CustomPathMission}`, `StartBalanceDetection`, `StiffnessSelfTuningResult`, `StartControlParametersAutoTuning` / `StopControlParametersAutoTuning`. All are **key names only**.

**Why cmd_set/cmd_id is not recoverable from these binaries.** The encoder does not carry a compiled-in key→command table. `SDKKeyInfo(name, ..., canGet, canSet, canAction, canListen, ...)` stores only a name and capability flags; `KeyInfoMgr::SyncKeyInfoList` populates the list at runtime; `KeyValueManager::UpdateValue(InnerKeyType, DjiValue)` and `ServerCmdMgr::ActionSendPack` pass values through without a per-key constant. The log format `cmd_set:{0},cmd_id:{1},sender_type:{2},sender_index:{3},receiver_type:{4},receiver_index:{5}` (`0x3e4432`) is filled at runtime, confirming the IDs arrive with the synced key list (from the gimbal or the packed Java), not from a static table. This is the hard ceiling on static analysis: **RS 5 cmd_ids can only come from a wire capture or safe probing** (§6.2), not from the `.so` files.

`libgimbal-lib.so` is ActiveTrack math (`TkGimbalCtrl`, object-box tracking). Skip Flutter/FFmpeg/AppGuard/Crashlytics.

### 0.4 Python tools in this repo

```bash
python dji_v1.py                 # rebuilds three captured golden frames
python dji_ble_cli.py scan
python dji_ble_cli.py listen
python dji_ble_cli.py joystick --pitch 0.4 --hold 1.5
python dji_ble_cli.py recenter
python dji_ble_cli.py selfie
python dji_ble_cli.py focus 90
python analyze_btsnoop.py        # HCI log catalog
python analyze_btsnoop.py --diff
```

---

## 1. Architecture

```text
Ronin app (Flutter UI + packed Java midware)
        │
        ▼
libdjibase.so   dji::sdk value types  (GimbalAngleRotation, GimbalSpeedRotation, …)
        │       V1PackHeader { cmdSet, cmdId, senderType, receiverType, sequenceNum, … }
        ▼
DUML V1 encoder  SOF 0x55, CRC8, CRC16     calc_crc8 / calc_crc16 in libdjibase.so
        │
        ▼
BLE GATT         service FFF0
                 write FFF5  (ATT Write Command 0x52)
                 notify FFF4 (enable CCCD 0x01 0x00)
        │
        ▼
RS gimbal BLE radio  ──same feature MCU──  RSA CAN (0xAA R SDK)
```

The app's object model (`RotateByAngle`, `RotateBySpeed`, `ResetGimbal`, `TurnOffGimbal`, …) is the same feature set the CAN SDK exposes. The wire format is DUML, not R SDK.

`libgimbal-lib.so` is ActiveTrack geometry. It does not pack BLE frames.

---

## 2. GATT

| Attribute | Value |
|-----------|--------|
| Service | `0000FFF0-0000-1000-8000-00805F9B34FB` |
| Notify (device → host) | UUID `0xFFF4` |
| Notify CCCD | write `0x01 0x00` to enable |
| Write (host → device) | UUID `0xFFF5`, ATT Write Command (`0x52`, no response) |
| Extra write handle seen in HCI | ATT handle `0x001F` (`FFF3`), purpose unknown |
| Advertised name | e.g. `DJI RS5 -070M9U` |

Handles in one capture (they can move; match by UUID):

| ATT handle | Role |
|-----------:|------|
| `0x0021` | FFF4 value (notifications) |
| `0x0022` | FFF4 CCCD |
| `0x0024` | FFF5 write |

Request a larger ATT MTU. Handshake frames are 13–38 bytes. Joystick and concatenated notifications are larger. Host writes may concatenate several V1 packets in one ATT value. Notifications do the same. Reassemble with `dji_v1.split_frames()`.

Pairing: put the gimbal in Bluetooth pairing mode, scan for names starting `DJI RS` / `Ronin`.

---

## 3. V1 / DUML frame

```text
Byte 0          SOF              0x55
Bytes 1–2       length + version little-endian bitfield
                length  = 10 bits (total size including SOF and CRC16)
                version = 6 bits, always 1 in these captures
                → byte1 = length & 0xFF
                → byte2 = (length >> 8) | (version << 2)   = 0x04 for length < 256
Byte 3          CRC8             DJI CRC8 of bytes 0–2, init 0x77
Byte 4          sender
Byte 5          receiver
Bytes 6–7       sequence         uint16 LE
Byte 8          attr             0x40 exec, 0x00 push/write, 0x80 ack
Byte 9          cmd_set
Byte 10         cmd_id
Bytes 11..N-3   payload
Bytes N-2..N-1  CRC16            DJI CRC16 of bytes 0..N-3, init 0x3692
```

Minimum packet is **13 bytes** (empty payload).

Byte 3 is **not** a message type. It is a function of length (and version), which is why early notes treated `0x33` / `0xFC` / `0xA2` as opcodes.

### 3.1 CRC (from `libdjibase.so`)

Exports: `calc_crc8` `@ 0084f270`, `calc_crc16`, `calc_crc16_ex`, `verify_crc16_cheksum`.

- CRC8: table-driven, init `0x77`. First table bytes `00 5E BC E2 61 3F DD 83 …`
- CRC16: reflected poly `0x1021` (table `0000 1189 2312 …`), init `0x3692`

`dji_v1.py` uses those tables. Golden frames for GetVersion, recenter, and joystick rebuild exactly.

### 3.2 High-level header in native code

`dji::sdk::V1PackHeader` JSON fields (Ghidra, `json_io` at `00db4a08`):

`version`, `senderIndex`, `senderType`, `receiverType`, `receiverIndex`, `senderMid`, `receiverMid`, `isNeedRes`, `isResponse`, `sequenceNum`, `cmdType`, `cmdSet`, `cmdId`, `retryTimes`, `retryInterval`

On the wire, sender/receiver collapse to one byte each (bytes 4–5). `isNeedRes` / `isResponse` fold into `attr`. `cmdType` is not a separate on-wire byte in these RS 5 captures.

`DjiProtocolEncoder::Encode` in `libdji_innertools.so` (`@ 0040c2c4`) writes the compact V1 layout when its compact flag is set. Total size is `payload_len + 0x0D`. Bytes 1–2 are `(length & 0x3FF) | 0x400` (version 1). CRC8 of the first three bytes goes in byte 3. CRC16 of the packet minus the last two bytes goes at the end. That is the same packing as `dji_v1.pack()`.

The encoder also has a **version-2 / extended** path (header 0x1D extra bytes, optional `0xCC55` prefix). `DecodeCommand` branches on `length_word >> 10`: `1` = compact (BLE), `2` = extended. RS 5 BLE captures are version 1 only. The `0xCC55` wrapper is an inner-tools TCP/USB envelope, not GATT.

**Attr byte (byte 8), from the decoder.** Low nibble is encryption type. Bits 5–6 are cmd type. Bit 7 is `isNeedRes`. RS 5 BLE uses `0x40` (`cmd_type=2`, encrypt=0) and `0x80` (ack). If the low nibble is `0x3` or `0xF`, the decoder XOR-decrypts from byte 9 using the sequence number and a derived key. Captured Ronin frames are plaintext.

### 3.3 Module IDs (bytes 4–5)

Each address byte is classic DUML: `(index & 7) << 5 | (type & 0x1F)`. Confirmed by `Encode` (sender type in `req[5]`, index in `req[6]`, receiver type in `req[7]`) and `DecodeCommand` (`*(uint16*)(pkt+4) & 0x1F` / `>> 5 & 7` / byte5 `& 0x1F` / `>> 13`).

Host commands use sender `0x02` (type APP=2, index 0).

| Wire | Type | Index | Role on RS 5 BLE |
|------|-----:|------:|------------------|
| `0x02` | 2 APP | 0 | Host / app |
| `0x04` | 4 GIMBAL | 0 | Joystick / recenter target |
| `0xBF` | 31 | 5 | Gimbal motors / focus |
| `0x27` | 7 WIFI | 1 | Push / control |
| `0xE5` | 5 CENTER | 7 | Accessory / capability / battery |
| `0xE4` | 4 GIMBAL | 7 | Video-side module |
| `0x24` | 4 | 1 | GetVersion during connect |
| `0x26` | 6 RC | 1 | GetVersion during connect |
| `0x44` | 4 | 2 | GetVersion during connect |
| `0x0B` | 11 | 0 | GetVersion during connect |
| `0x32` | 18 | 1 | GetVersion during connect |

Joystick and recenter still go to wire `0x04` (gimbal type 0, index 0). Focus goes to `0xBF` (type 31, index 5). You can keep treating the byte as an opaque module ID when building packets. The split only matters when matching DeviceType enums.

---

## 4. Handshake

From `bt_snoop_app_connect_landing_view.log`, emitted by `dji_v1.handshake_packets()`:

1. Enable FFF4 notifications.
2. GetVersion (`cmd_set 0x00`, `cmd_id 0x01`) to `BF`, `04`, `04`.
3. Battery query `0x0D/0x01` to `E5`, 9 zero bytes.
4. Enable push `0x07/0x0E` to `27`.
5. GetVersion `27`.
6. Capability pages 0, 1, 2 (`0x00/0x4F` to `E5`).
7. Empty acks `0x04/0x38` and `0x04/0x64` to `04` (`attr=0x80`).
8. Config `0x04/0x12` hello + block to `E5` (`attr=0x00`).
9. Probe `0x00/0x32` payload `11` to `E5`.
10. GetVersion remaining modules; param read `0x04/0x10` id `0x12`; subscribe `0x07/0x07`.

Motion commands that the official app sends after landing (joystick, recenter) work after this sequence. Some features may need extra config pages. If a command is ignored, capture that action from the official app and diff against this handshake.

---

## 5. Commands verified on RS 5 BLE

All host commands: sender `0x02`, `attr=0x40` unless noted.

### 5.1 General `cmd_set=0x00`

| cmd_id | Dest | Payload | Meaning |
|--------|------|---------|---------|
| `0x01` | each module | empty | GetVersion / register. Ack (`attr=0x80`) carries ASCII like `DJI RS5 -070M9U` |
| `0x32` | `0xE5` | `11` | Connect probe |
| `0x4F` | `0xE5` | `01 00 PP 00 00 FF FF FF FF` | Capability page `PP` |

### 5.2 Gimbal `cmd_set=0x04`

**Joystick** (`cmd_id=0x01`, dest `0x04`, 9-byte payload). Native type `GimbalJoystickControlSpeedMsg` / SDK key `JoystickControlSpeed`. Public DUML name: Gimbal Control.

```text
uint16 LE pitch, uint16 LE roll, uint16 LE yaw, then 00 00 02
```

Neutral is **1024** on every axis. Typical stick throw ±400–600. Send ~10 Hz while held, then a 1024/1024/1024 release. This is the on-screen stick, not CAN speed control.

**Recenter / selfie** (`cmd_id=0x4C`, dest `0x04`, payload `FE` + mode). Native `GimbalResetCommandMsg` / SDK key `ResetGimbal`. Public DUML name: Gimbal Reset And Set Mode.

| Mode | Action |
|------|--------|
| `0x01` | Recenter once |
| `0x02` | Selfie / flip once |

Same `FE <mode>` payload as CAN `0x0E/0x0E`.

**Focus motor** (`cmd_id=0x2F`, dest `0xBF`, `attr=0x00`, payload `01 00 02` + uint16 LE position). Range ~20–4096. The app ramps at ~20 Hz. CAN equivalent is `0x0E/0x12`.

**Param read** (`cmd_id=0x10`, dest `0x04`, 1-byte param id). Seen: `0x02–0x05`, `0x0E`, `0x0F`, `0x11`, `0x12`. Public DUML: Gimbal User Params Get.

**Config block** (`cmd_id=0x12`, dest `0xE5`, `attr=0x00`). 21- and 25-byte blobs in `handshake_packets()`. Required before some motion.

**Attitude push** (`cmd_id=0x05`, src `0x04`, ~50-byte payload, ~1–2 Hz). First three int16 LE values are yaw/roll/pitch in 0.1°. Native `GimbalCurrentAttitudeMsg`. Public DUML: Gimbal Params Get. This is the BLE stand-in for CAN `0x0E/0x02` angle queries / `0x0E/0x08` push.

Gimbal src `0xBF` also streams `0x04/0x2F` at high rate (several payload sizes). That is focus-motor telemetry, not 3-axis attitude.

### 5.3 Push enable `cmd_set=0x07` dest `0x27`

| cmd_id | Meaning |
|--------|---------|
| `0x0E` | Enable push |
| `0x07` | Subscribe to push events |

Empty payload. These are the old 13-byte “0x33” frames whose 3-byte “payload” was actually `attr, cmd_set, cmd_id`.

### 5.4 Battery `cmd_set=0x0D`

`cmd_id=0x01` dest `0xE5`, 9 zero bytes. Queried once at connect.

---

## 6. CAN-equivalent control over BLE

Do not wrap `0xAA` CAN packets inside FFF5 writes. Map each CAN feature to a DUML command instead.

| CAN (`docs/DJI_R_SDK_Protocol.md`) | BLE DUML | Evidence | Status |
|---|---|---|---|
| Position `0x0E/0x00` 3×int16 0.1° + ctrl + time | `0x04/0x0A` Ext Ctrl Degree, or `0x04/0x14` Abs Angle. SDK type `GimbalAngleRotation` / key `RotateByAngle` | Native JSON fields `mode, pitch, roll, yaw, pitchIgnored, rollIgnored, yawIgnored, duration, jointReferenceUsed, timeout` match the CAN ctrl/time semantics. On-wire cmd_id **not captured** on RS 5. | Hypothesis. Capture “go to angle” from the app. |
| Speed `0x0E/0x01` 3×int16 0.1°/s + ctrl | `0x04/0x0C` Ext Ctrl Accel. SDK type `GimbalSpeedRotation` / key `RotateBySpeed` | Native ctor is three doubles + `CtrlInfo`. Public DUML payload is 3×int16 + flags. | Hypothesis. Capture a non-joystick pan (preset speed or “follow speed”). |
| Angles `0x0E/0x02` | Push `0x04/0x05` (and optionally poll `0x04/0x02`) | Verified push parse in `dji_v1.parse_attitude()` | Use the push. |
| Limits `0x0E/0x04` | User params / `GimbalAttitudeRange` / `GimbalLimitationState` | SDK keys only | Capture. |
| Stiffness `0x0E/0x06` | Keys `YawMotorControlStiffness`, `PitchMotorControlStiffness`, `RollMotorControlStiffness`. DUML `0x04/0x0F` set / `0x04/0x10` get | Native strings + param-read already in handshake | Capture a stiffness slider. |
| Parameter push enable `0x0E/0x07` | BLE `0x07/0x0E` + `0x07/0x07` | Verified handshake | Done. |
| Angle push `0x0E/0x08` | `0x04/0x05` | Verified | Done. |
| Module version `0x0E/0x09` | `0x00/0x01` GetVersion | Verified | Done. |
| User params `0x0E/0x0B` | `0x04/0x10` get, `0x04/0x0F` / `0x04/0x12` set. SDK `GimbalFollowSettings`, `GimbalJoystickSettings`, `GimbalMode` | Handshake already reads param `0x12` | Partial. |
| Sleep / wake `0x0E/0x0C` | Keys `TurnOffGimbal`, `TurnOnGimbal`, `DormantGimbal`, `SleepNegotiate`. Public DUML `0x04/0x0D` suspend/resume (Phantom magic `0x7EF2` / `0x2AB5`) | Native types exist. RS 5 payload **unknown**. | Capture power-off / sleep from the app. |
| Recenter / selfie `0x0E/0x0E` `FE 01` / `FE 02` | `0x04/0x4C` `FE 01` / `FE 02` | Verified | Done. `dji_ble_cli.py recenter` / `selfie` |
| AutoTune `0x0E/0x0F` | `0x04/0x08` Calibration, key `StartAdvancedGimbalCalibrate`, type `GimbalControlParametersAutoTuningState` | Native keys | Capture AutoTune. |
| AutoTune status push `0x0E/0x10` | `0x04/0x30` Auto Calibration Status | Public DUML name | Listen after AutoTune. |
| ActiveTrack `0x0E/0x11` | Keys `ActiveTrackMode`, `ActiveTrackMovementMode`. `libgimbal-lib.so` does the vision math | Native keys | Capture track start/stop. |
| Focus `0x0E/0x12` | `0x04/0x2F` dest `BF` | Verified | Done. `dji_ble_cli.py focus` |
| Camera record `0x0D/0x00` `03 00` / `04 00` | Packed Java `DataThirdPartyCameraSetAction`. SDK fields `cameraAction`, `cameraActionType`. DUML camera is usually cmd_set `0x02`, but Ronin may proxy through the gimbal | Class name in packed DEX strings | Capture record start/stop with a camera on the control cable. |
| Follow / lock / FPV mode | `GimbalMode`, `GimbalRollMode`, `surfaceFollowModeEnable`. Recenter cmd `0x4C` can also set mode on older products | Capture file name `bt_snoop_app_connect_follow_mode_smooth.log` is referenced but not in the repo | Re-capture follow-mode change. |

### 6.1 What “nearly CAN-level” means in practice

Already reachable from `dji_ble_cli.py` after handshake:

- Stick-style rates (joystick `0x04/0x01`)
- Recenter and selfie
- Focus/zoom motor position
- Attitude stream
- Battery probe / version / capability pages

Still needed for parity with `dji_gimbal_cli.py`:

1. Absolute / timed position (`RotateByAngle`)
2. True speed takeover (`RotateBySpeed`, ctrl bit 7 on CAN)
3. Sleep / wake
4. Camera record and focus-center
5. AutoTune and focus-motor autocal
6. Stiffness and follow-mode writes
7. Axis limits

Items 1–2 are the important ones. Joystick is PWM-style around 1024. CAN speed is degrees/second with an explicit takeover bit. They feel different on RS 4/5 (the CAN notes already say gimbal-side endpoints are ignored unless the host is in a joystick-style mode).

### 6.2 Proposed on-wire payloads to try (unverified on RS 5)

Public DUML (o-gs lua, Phantom) and Osmo Pocket 3 BLE agree on **sizes**. Axis **order** does not: CAN is yaw/roll/pitch; native `GimbalAngleRotation` and Osmo `0x0A` are pitch/roll/yaw. Confirm with an official-app capture before treating this as spec.

These are encoded as experimental builders in `dji_v1.py` and exposed in `dji_ble_cli.py`, each gated behind `--experimental`. They run after the normal handshake and then listen for an ACK/error:

```bash
python dji_ble_cli.py rotate-angle --pitch 5 --experimental
python dji_ble_cli.py rotate-angle --pitch 5 --absolute --experimental   # 0x14 form
python dji_ble_cli.py rotate-speed --yaw 5 --hold 0.5 --experimental
python dji_ble_cli.py mode follow --experimental
python dji_ble_cli.py stiffness pitch 60 --experimental
python dji_ble_cli.py sleep --experimental          # and --wake to resume
python dji_ble_cli.py autotune --experimental       # watch cmd 0x04/0x30
python dji_ble_cli.py record --experimental         # very speculative
```

A good result is an ACK on the same cmd with an empty/zero payload or motion. A parse-error ACK or silence means RS 5 uses a different cmd_id or payload for that action — record the reply and adjust.

**Position / RotateByAngle** — try `cmd_set=0x04 cmd_id=0x0A` dest `0x04`, `attr=0x40` (10 bytes). Alternate: `0x04/0x14` (8 bytes, no duration word).

```text
int16 LE axis0_x10     Osmo: pitch. CAN: yaw. Native object: pitch.
int16 LE axis1_x10     roll
int16 LE axis2_x10     Osmo: yaw. CAN: pitch. Native object: yaw.
int16 LE duration_x100 (0x0A only)
uint8  flags           Osmo 0x14: bit0=pitch bit1=roll bit2=yaw. Native: pitch/roll/yawIgnored.
uint8  time            0.1 s (maps to native timeout / CAN time)
```

**Speed / RotateBySpeed** — try `cmd_set=0x04 cmd_id=0x0C` dest `0x04` (7 bytes):

```text
int16 LE axis0_rate_x10
int16 LE axis1_rate_x10
int16 LE axis2_rate_x10
uint8  flags            Osmo bit0=enable. CAN uses 0x80 takeover.
```

Native `GimbalSpeedRotation` is three doubles + 1-byte `CtrlInfo` (DjiValue size 25). Same conversion as position: doubles → int16 ×10, plus that flag byte.

If the gimbal ACKs with a parse error or ignores the packet, RS 5 likely uses a Ronin-specific cmd_id (recenter moved to `0x4C`, focus to `0x2F`). RS 5 joystick is already 9 bytes (`3×uint16` + `00 00 02`); Osmo joystick is 6 bytes. Extra trailer bytes are a Ronin thing.

**Sleep** — public DUML `0x04/0x0D` uint16 LE `0x7EF2` suspend / `0x2AB5` resume only as a last resort. Native keys `DeviceRequestEnterLowPowerMode` / `ExitLowPowerMode` / `SleepNegotiate` suggest a multi-step exchange. Prefer copying `TurnOffGimbal` from the official app.

**Follow / lock / FPV** — lua and Osmo both put mode on `0x04/0x4C` as two bytes. Osmo: mode `0=lock 1=follow 2=FPV`. RS 5 recenter/selfie is the same cmd_id with `FE 01` / `FE 02`. A follow-mode change is likely another `0x4C` payload, not a new cmd_id.

**AutoTune** — lua `0x04/0x08` is a 1-byte calib command on Phantom. Native keys: `StartControlParametersAutoTuning` / `StartAdvancedGimbalCalibrate`. Status push is lua `0x04/0x30` (progress + status).

---

## 7. Native SDK keys in `libdjibase.so`

These are the app-facing names. The KeyManager maps them onto V1 `cmdSet`/`cmdId`. Useful when searching Ghidra (`search_strings` / `search_functions`) or when reading a future decrypted DEX.

**Motion.** `RotateByAngle`, `RotateBySpeed`, `ResetGimbal`, `MotionControl`, `JoystickControlSpeed`, `RequestJoystickControlAuth`, `GimbalFollowSpeed`, `AlignedGimbalAttitude`, `GimbalCurrentAttitude`, `GimbalJointAttitude`

**Power.** `TurnOnGimbal`, `TurnOffGimbal`, `DormantGimbal`, `SleepNegotiate`

**Mode / feel.** `GimbalMode`, `GimbalRollMode`, `GimbalSceneMode`, `GimbalFollowSettings`, `GimbalJoystickSettings`, `JoystickControlMode`, `surfaceFollowModeEnable`, `YawMotorControlStiffness`, `PitchMotorControlStiffness`, `RollMotorControlStiffness`

**Calibration.** `StartAdvancedGimbalCalibrate`, `ManualCalibrateGimbal`, `GimbalCalibrationState`, `GimbalControlParametersAutoTuningState`, `GimbalBalanceDetectionState`

**Camera / track.** `cameraAction`, `ActiveTrackMode`, `ActiveTrackMovementMode`

**Value types with serializers (`libdjibase.so`, DjiValue packed layout — not DUML).**

| Type | Packed size | Layout |
|---|---:|---|
| `GimbalAngleRotation` | 44 | uint32 mode; 3×double pitch,roll,yaw; 3×bool ignored; double duration; bool jointReferenceUsed; int32 timeout |
| `GimbalSpeedRotation` | 25 | 3×double + 1-byte `CtrlInfo` |
| `GimbalResetCommandMsg` | 4 | uint32 `GimbalResetCommand` enum |
| `GimbalModeMsg` | 4 | uint32 `GimbalMode` enum |
| `GimbalMotionControlReq` | 32 | 3×double attitude + uint32 mode + uint32 |
| `GimbalJoystickControlSpeedMsg` | 12 | 3×uint32 |
| `GimbalLimitationState` | 3 | 3×bool |
| `GimbalAttitudeRange` | 72 | 9×double (min/max per axis) |
| `ManualCalibrateGimbalParam` | 56 | 7×double |
| `Attitude` | 24 | 3×double |

Ctor for angle is `GimbalAngleRotation(mode, pitch, roll, yaw, pitchIgnored, rollIgnored, yawIgnored, duration, jointReferenceUsed, timeout)`.

Also in the native catalog (innertools string blob + djibase `KeyManager` registrar): `StartControlParametersAutoTuning` / `StopControlParametersAutoTuning`, `DeviceRequestEnterLowPowerMode` / `ExitLowPowerMode` / `RejectEnterLowPowerMode`, `RecenterProgress`, `StartGimbalTimeLapse` / `StopGimbalTimeLapse`.

The 0x19 / 0x2C / 0x20 sizes are **DjiValue** serializations, not DUML payloads. The transport layer converts doubles/enums into the compact on-wire integers.

`KeyManager` registration in `libdjibase.so` is `(name, GetValueSharedPtr<T>)` — value-type factory, not `cmd_set`/`cmd_id`. The DUML IDs are applied later (packed Java midware, or a table we have not found). HCI capture of one official-app action is still the reliable way to get the on-wire ID.

---

## 8. Public DUML gimbal cmd_id table (`cmd_set 0x04`)

From o-gs `dji-dumlv1-gimbal.lua`. Phantom-era names. RS 5 reuses some IDs and adds others (`0x2F` focus is not in this list).

| cmd_id | Public name | Payload (public / Osmo) | RS 5 notes |
|-------:|-------------|-------------------------|------------|
| `0x01` | Gimbal Control | Osmo: 3×uint16 around 1024 | **Verified** 9 bytes: that plus `00 00 02` |
| `0x02` | Get Position | empty | Not seen. Attitude arrives as `0x05` push |
| `0x05` | Params Get / push position | ≥12 B, 0.1° | **Verified** `dji_v1.parse_attitude()` |
| `0x08` | Calibration | 1-byte cmd (Phantom) | AutoTune candidate |
| `0x0A` | Ext Ctrl Degree | 10 B, 0.1° + duration + flags | Position candidate |
| `0x0C` | Ext Ctrl Accel | 7 B, 0.1°/s + flags | Speed candidate |
| `0x0D` | Suspend / Resume | uint16 `0x7EF2` / `0x2AB5` | Sleep candidate |
| `0x0F` | User Params Set | stiffness/accel words | Stiffness candidate |
| `0x10` | User Params Get | | **Verified** 1-byte id |
| `0x12` | (not in lua) | | **Verified** config blob to `E5` |
| `0x14` | Abs Angle Control | 8 B, 0.1° + flags + duration | Alternate position candidate |
| `0x15` | Movement | 20 B incremental | Unseen |
| `0x2F` | (not in lua) | | **Verified** focus motor |
| `0x30` | Auto Calibration Status | progress + status | Listen after AutoTune |
| `0x31` / `0x32` | Robin Params Set / Get | | Ronin-specific |
| `0x38` | Timelapse Status | | Handshake empty ack |
| `0x39` | Lock | | Unseen |
| `0x4C` | Reset And Set Mode | 2 B. Osmo: 0/1/2 = lock/follow/FPV | **Verified** `FE 01` / `FE 02` recenter/selfie |
| `0x58` | Handheld Stick Control Set | | Unseen |
| `0x64` | (not in lua) | | Handshake empty ack |

---

## 9. How to finish the remaining mappings

One official-app action per log. On a GrapheneOS Pixel there is no root `adb pull` of `/data/misc/bluetooth/logs`. Enable HCI snoop, you do the action, then pull a bugreport.

```bash
# one-time: USB debugging + "Enable Bluetooth HCI snoop log" in Developer options
adb devices
python ble_phone_snoop.py status
python ble_phone_snoop.py enable
python ble_phone_snoop.py stop-app
# open Ronin, connect, do ONLY the labeled action
python ble_phone_snoop.py pull --label go_to_angle    # 1–3 minutes (bugreport)
python ble_phone_snoop.py diff --new captures/bt_snoop_app_connect_go_to_angle.log
```

Cursor can drive the same loop via the `dji-ble-snoop` MCP (`dji_ble_snoop_mcp.py`, needs `pip install mcp`). The MCP cannot tap the Ronin UI.

| Capture | CAN feature it unlocks |
|---|---|
| Move to a known angle (or a saved position) | `RotateByAngle` cmd_id + payload |
| Hold a speed that is not the on-screen joystick | `RotateBySpeed` |
| Sleep / power off / wake | `TurnOffGimbal` |
| Record start / stop with camera cable | camera DUML |
| AutoTune | `0x04/0x08` or Ronin-specific |
| Follow → lock → FPV | `GimbalMode` |
| Stiffness slider | user-param write |
| Focus-motor autocal | extra `0x04/0x2F` opcode |

```bash
python ble_phone_snoop.py catalog captures/bt_snoop_app_connect_<action>.log
python ble_phone_snoop.py diff --new captures/bt_snoop_app_connect_<action>.log
python analyze_btsnoop.py --file captures/bt_snoop_app_connect_<action>.log
```

`analyze_btsnoop.py` still labels byte 3 as `msg_id` (a length fingerprint). `ble_phone_snoop.py catalog` / `diff` decode real `cmd_set`/`cmd_id` with `dji_v1.unpack()`.

Diff the new log against `bt_snoop_app_connect_landing_view.log` (handshake only). The extra `cmd_set/cmd_id` after the handshake **is** the action.

Then add a builder in `dji_v1.py` and a subcommand in `dji_ble_cli.py`, same pattern as `cmd_recenter`.

---

## 10. Python API (current)

```bash
python dji_ble_cli.py scan
python dji_ble_cli.py listen --seconds 10
python dji_ble_cli.py joystick --pitch 0.4 --yaw -0.2 --hold 1.5
python dji_ble_cli.py recenter
python dji_ble_cli.py selfie
python dji_ble_cli.py focus 90
```

Handshake is automatic. Requires `pip install bleak`.

---

## 11. Uncertainties

- Whether RS 5 position/speed use Phantom cmd_ids `0x0A`/`0x0C` or newer Ronin IDs like `0x2F`/`0x4C`.
- Exact sleep payload. `SleepNegotiate` suggests a multi-step exchange, not a single magic word.
- Camera record path: gimbal-proxied DUML vs camera cmd_set `0x02` vs something that looks like CAN `0x0D` packed inside V1 (unlikely, but not disproven).
- FFF3 write handle `0x001F`.
- Whether ATT Write Request (`0x12`) is ever required. Captures use Write Command (`0x52`).
- Encrypted Java `CmdIdGimbal$CmdIdType` numeric values. A runtime DEX dump would list them in one enum.

---

## 12. HCI captures referenced

Isolated official-app actions (connect + one control). Files may live outside the git tree:

| File | Action |
|------|--------|
| `bt_snoop_app_connect_landing_view.log` | Connect only |
| `bt_snoop_app_connect_follow_mode_smooth.log` | Follow smooth |
| `bt_snoop_app_connect_recenter.log` | Recenter |
| `bt_snoop_app_connect_pitch_up.log` / `_down` | Pitch stick |
| `bt_snoop_app_connect_yaw_left.log` | Yaw stick |
| `bt_snoop_app_connect_roll_right.log` | Roll stick |
| `bt_snoop_app_connect_focus_zoom_to_90.log` | Focus ~90% |
| `bt_snoop_app_connect_focus_zoom_to_7_then_0.log` | Focus 7% then 0% |

# Quick Start Guide - ESP32 Gimbal Bridge

Get up and running with the ESP32 gimbal bridge firmware in minutes.

## Prerequisites

- **Hardware**: Seeed XIAO ESP32C3 + XIAO CAN Bus Expansion Board
- **USB Cable**: Must support data transfer (not just charging)
- **Computer**: Windows, macOS, or Linux

## Setup (First Time Only)

### Option 1: Automated Setup (Recommended)

**Windows (PowerShell - run as Administrator):**
```powershell
cd firmware/esp32-gimbal-bridge
.\setup-dev-environment.ps1
```

**Linux/macOS (Terminal):**
```bash
cd firmware/esp32-gimbal-bridge
chmod +x setup-dev-environment.sh
./setup-dev-environment.sh
```

This installs:
- Python 3.7+
- PlatformIO Core
- All required libraries
- USB drivers (Linux)

### Option 2: Manual Setup

1. Install Python 3.7+ from [python.org](https://www.python.org/downloads/)
   - Windows: Check "Add Python to PATH" during installation
2. Install PlatformIO:
   ```bash
   pip install platformio
   ```
3. Install project dependencies:
   ```bash
   cd firmware/esp32-gimbal-bridge
   pio pkg install
   ```

## Build and Flash

### Quick Flash (Everything)

**Windows (PowerShell):**
```powershell
.\flash-board.ps1
```

**Linux/macOS:**
```bash
chmod +x flash-board.sh
./flash-board.sh
```

This automatically:
1. Finds your board
2. Builds the firmware
3. Flashes firmware
4. Flashes web UI
5. Shows next steps

### Flash with Serial Monitor

To see the boot log immediately after flashing:

**Windows:**
```powershell
.\flash-board.ps1 -Monitor
```

**Linux/macOS:**
```bash
./flash-board.sh -m
```

### Update Web UI Only

If you only changed `data/index.html`:

**Windows:**
```powershell
.\flash-board.ps1 -WebOnly
```

**Linux/macOS:**
```bash
./flash-board.sh -w
```

## Troubleshooting

### Board Not Found

1. **Check USB cable** - must support data (many cables are charge-only)
2. **Install drivers**:
   - Windows: Usually automatic via Windows Update
   - Linux: Run `./setup-dev-environment.sh` for udev rules
   - macOS: Usually automatic

3. **Manual bootloader mode**:
   - Hold **BOOT** button
   - Tap **RESET** button
   - Release **BOOT** when upload starts

### Upload Hangs on Windows

If the upload progress shows Unicode characters incorrectly:
- The script handles this automatically
- Or manually set before running `pio`:
  ```powershell
  $env:PYTHONUTF8 = "1"
  $env:PYTHONIOENCODING = "utf-8"
  chcp 65001
  ```

### Linux Permission Denied

If you get `Permission denied` on `/dev/ttyACM0`:

1. Run setup script (adds you to `dialout` group):
   ```bash
   ./setup-dev-environment.sh
   ```

2. **Log out and log back in** for group changes to take effect

3. Or temporarily use:
   ```bash
   sudo chmod 666 /dev/ttyACM0
   ```

### Windows: Git Bash Issues

**Don't use Git Bash for flashing!** Use PowerShell or CMD instead.

Git Bash has issues with:
- PlatformIO's serial port handling
- Environment variables
- UTF-8 encoding

Always use:
- **PowerShell**: `.\flash-board.ps1`
- **CMD**: Same as PowerShell commands

## Manual Commands (If Scripts Don't Work)

Navigate to the firmware directory:
```bash
cd firmware/esp32-gimbal-bridge
```

**Build only:**
```bash
pio run -e seeed_xiao_esp32c3
```

**Flash firmware:**
```bash
pio run -e seeed_xiao_esp32c3 -t upload
```

**Flash filesystem (web UI):**
```bash
pio run -e seeed_xiao_esp32c3 -t uploadfs
```

**Serial monitor:**
```bash
pio device monitor -b 115200
```

**Clean build:**
```bash
pio run -e seeed_xiao_esp32c3 -t clean
```

## First Boot

After flashing successfully:

1. **WiFi Setup**:
   - Board creates AP: `DJI-Gimbal-Setup`
   - Connect with your phone/laptop
   - Captive portal opens (or go to `192.168.4.1`)
   - Enter your WiFi credentials

2. **Find IP Address**:
   - Check router's DHCP client list
   - Or connect serial monitor to see IP in boot log

3. **Access Web Interface**:
   - Open browser: `http://<device-ip>/`
   - You should see the joystick control interface

4. **Connect Hardware**:
   - Wire CAN bus to gimbal as per [README.md](firmware/esp32-gimbal-bridge/README.md)
   - Set gimbal switch to **CAN** (not S-BUS)
   - Power from Focus Wheel or external 5V

## Script Options

### flash-board.sh / flash-board.ps1

```
Options:
  -b, --build-only      Build without flashing
  -f, --firmware-only   Flash firmware only (skip filesystem)
  -w, --web-only        Flash web UI only (skip firmware)
  -m, --monitor         Open serial monitor after flashing
  -p, --port PORT       Specify serial port manually
  -h, --help            Show help message
```

**Examples:**

```bash
# Build without flashing
./flash-board.sh --build-only

# Flash to specific port
./flash-board.sh --port /dev/ttyACM0

# Flash firmware only (skip web UI)
./flash-board.sh --firmware-only

# Flash and monitor
./flash-board.sh --monitor
```

## Next Steps

- See [firmware/esp32-gimbal-bridge/README.md](firmware/esp32-gimbal-bridge/README.md) for:
  - Complete API reference
  - Hardware wiring details
  - PTZ controller setup
  - Zoom calibration

- See [docs/DJI_R_SDK_Protocol.md](docs/DJI_R_SDK_Protocol.md) for:
  - Protocol documentation
  - CAN bus details
  - Command reference

## Getting Help

If you encounter issues:

1. **Check serial output**:
   ```bash
   pio device monitor -b 115200
   ```

2. **Verify board is detected**:
   ```bash
   pio device list
   ```

3. **Check CAN diagnostics**:
   - Open web UI: `http://<device-ip>/`
   - Click "CAN Status" to see bus health

4. **Common issues**:
   - No telemetry: Enable "Parameter Push" in web UI
   - No CAN ACK: Check CANH/CANL wiring and termination (P1 pad)
   - Bus-off: Gimbal switch must be on CAN, not S-BUS

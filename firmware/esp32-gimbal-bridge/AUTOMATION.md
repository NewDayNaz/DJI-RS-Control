# Development Automation Tools

This directory contains scripts to automate the setup and flashing process for the ESP32 gimbal bridge firmware.

## Files

### Setup Scripts

- **`setup-dev-environment.sh`** - Linux/macOS environment setup
- **`setup-dev-environment.ps1`** - Windows PowerShell environment setup

These scripts install:
- Python 3.7+
- PlatformIO Core
- Project dependencies
- USB drivers (Linux)

### Flash Scripts

- **`flash-board.sh`** - Linux/macOS build and flash automation
- **`flash-board.ps1`** - Windows PowerShell build and flash automation

These scripts:
- Auto-detect the ESP32-C3 board
- Build firmware
- Flash firmware and filesystem
- Handle platform-specific quirks

## Quick Start

See [QUICK_START.md](../../QUICK_START.md) in the project root for detailed instructions.

### First Time Setup

**Windows:**
```powershell
.\setup-dev-environment.ps1
```

**Linux/macOS:**
```bash
./setup-dev-environment.sh
```

### Flash Board

**Windows:**
```powershell
.\flash-board.ps1
```

**Linux/macOS:**
```bash
./flash-board.sh
```

## Script Features

### Auto-Detection
- Automatically finds Python installation
- Detects PlatformIO installation
- Finds connected ESP32-C3 board
- Handles multiple OS variants

### Error Handling
- Clear error messages
- Helpful troubleshooting hints
- Graceful fallbacks
- Boot button reminder on flash failure

### Platform-Specific Handling

**Windows:**
- UTF-8 encoding setup for esptool
- COM port auto-detection
- WMI device identification
- Warning against Git Bash usage

**Linux:**
- udev rules installation
- dialout group management
- Multiple package manager support
- USB device permission setup

**macOS:**
- Native USB device detection
- Homebrew compatibility

## Options

Both `flash-board.sh` and `flash-board.ps1` support:

```
-b, --build-only      Build without flashing
-f, --firmware-only   Flash firmware only (skip web UI)
-w, --web-only        Flash web UI only (skip firmware)
-m, --monitor         Open serial monitor after flash
-p, --port PORT       Specify port manually
-h, --help            Show help
```

## Examples

### Development Workflow

```bash
# One-time setup
./setup-dev-environment.sh

# Daily development: flash and monitor
./flash-board.sh -m

# Quick web UI updates
./flash-board.sh -w

# Test build without flashing
./flash-board.sh -b
```

### Troubleshooting

```bash
# Specify port manually
./flash-board.sh -p /dev/ttyACM0

# View available devices
pio device list

# Manual bootloader mode
# Hold BOOT, tap RESET, release BOOT, then:
./flash-board.sh
```

## Requirements

- Python 3.7 or higher
- PlatformIO Core 6.0+
- USB cable with data support
- Seeed XIAO ESP32C3 board

## Manual Alternative

If the scripts don't work for your setup, you can run commands manually:

```bash
cd firmware/esp32-gimbal-bridge

# Install dependencies
pio pkg install

# Build
pio run -e seeed_xiao_esp32c3

# Flash firmware
pio run -e seeed_xiao_esp32c3 -t upload

# Flash filesystem
pio run -e seeed_xiao_esp32c3 -t uploadfs

# Monitor
pio device monitor -b 115200
```

## Contributing

When adding new features that require additional dependencies:

1. Update `platformio.ini` with new libraries
2. Test setup scripts on clean systems
3. Update QUICK_START.md with any new requirements
4. Document platform-specific issues

## Notes

### Windows Considerations

- **Always use PowerShell or CMD**, never Git Bash for flashing
- Git Bash has serial port and encoding issues with PlatformIO
- The PowerShell script automatically handles UTF-8 encoding
- Administrator privileges may be needed for driver installation

### Linux Considerations

- User must be in `dialout` group (setup script handles this)
- **Log out and back in** after first setup for permissions
- udev rules installation requires sudo access
- Multiple distros supported (Debian, Fedora, Arch, etc.)

### macOS Considerations

- Native USB should work out of the box
- No special drivers needed for ESP32-C3
- Homebrew Python is supported
- May need to approve USB access in System Preferences

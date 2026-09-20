#!/usr/bin/env bash
# Automated build and flash script for ESP32 gimbal bridge
# Finds the board automatically and flashes firmware + filesystem

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_step() { echo -e "${BLUE}[STEP]${NC} $1"; }

# Check if PlatformIO is available
check_pio() {
    if ! command -v pio &> /dev/null; then
        log_error "PlatformIO not found in PATH"
        log_info "Please run ./setup-dev-environment.sh first"
        exit 1
    fi
}

# Find the ESP32-C3 board
find_board() {
    log_info "Searching for XIAO ESP32C3 board..."
    
    # Get list of serial ports
    pio device list > /tmp/pio_devices.txt 2>&1 || true
    
    # Look for common ESP32-C3 identifiers
    local port=""
    
    # Linux: /dev/ttyACM* or /dev/ttyUSB*
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        # Check for XIAO ESP32C3 specifically (native USB)
        for p in /dev/ttyACM* /dev/ttyUSB*; do
            if [ -e "$p" ]; then
                # Try to read device info
                if udevadm info "$p" 2>/dev/null | grep -iq "esp32\|cp210\|ch340\|1a86\|303a"; then
                    port="$p"
                    break
                fi
                # Fallback: just use the first available port
                if [ -z "$port" ]; then
                    port="$p"
                fi
            fi
        done
    
    # macOS: /dev/cu.usbserial-* or /dev/cu.usbmodem*
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        for p in /dev/cu.usbmodem* /dev/cu.usbserial* /dev/cu.wchusbserial*; do
            if [ -e "$p" ]; then
                port="$p"
                break
            fi
        done
    
    # Windows Git Bash/WSL
    elif [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "cygwin" ]]; then
        log_warn "Running under Git Bash/MSys detected"
        log_warn "For Windows, please use PowerShell and flash-board.ps1 instead"
        log_warn "Git Bash has issues with PlatformIO's serial handling"
        exit 1
    fi
    
    if [ -z "$port" ]; then
        log_error "No serial device found!"
        log_info "Please check:"
        log_info "  1. Board is connected via USB"
        log_info "  2. USB cable supports data (not just power)"
        log_info "  3. USB drivers are installed"
        log_info ""
        log_info "Available devices:"
        pio device list
        exit 1
    fi
    
    log_info "Found board at: $port"
    echo "$port"
}

# Build the project
build_firmware() {
    log_step "Building firmware..."
    cd "$SCRIPT_DIR"
    pio run -e seeed_xiao_esp32c3
    log_info "Build successful!"
}

# Flash firmware to board
flash_firmware() {
    local port=$1
    log_step "Flashing firmware to $port..."
    cd "$SCRIPT_DIR"
    
    # Try upload with auto-detected port
    if [ -n "$port" ]; then
        pio run -e seeed_xiao_esp32c3 -t upload --upload-port "$port"
    else
        # Let PlatformIO auto-detect
        pio run -e seeed_xiao_esp32c3 -t upload
    fi
    
    log_info "Firmware flashed successfully!"
}

# Flash filesystem (web UI)
flash_filesystem() {
    local port=$1
    log_step "Flashing web UI (LittleFS filesystem)..."
    cd "$SCRIPT_DIR"
    
    if [ -n "$port" ]; then
        pio run -e seeed_xiao_esp32c3 -t uploadfs --upload-port "$port"
    else
        pio run -e seeed_xiao_esp32c3 -t uploadfs
    fi
    
    log_info "Filesystem flashed successfully!"
}

# Monitor serial output
monitor_serial() {
    local port=$1
    log_step "Opening serial monitor (Ctrl+C to exit)..."
    log_info "Waiting for board to boot..."
    sleep 2
    
    cd "$SCRIPT_DIR"
    if [ -n "$port" ]; then
        pio device monitor -p "$port" -b 115200
    else
        pio device monitor -b 115200
    fi
}

show_help() {
    echo "ESP32 Gimbal Bridge Flash Tool"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -b, --build-only      Build without flashing"
    echo "  -f, --firmware-only   Flash firmware only (skip filesystem)"
    echo "  -w, --web-only        Flash web UI only (skip firmware)"
    echo "  -m, --monitor         Open serial monitor after flashing"
    echo "  -p, --port PORT       Specify serial port manually"
    echo "  -h, --help            Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                    # Build and flash everything"
    echo "  $0 -m                 # Flash and open serial monitor"
    echo "  $0 -w                 # Update web UI only"
    echo "  $0 -p /dev/ttyACM0    # Use specific port"
    echo ""
}

main() {
    local build_only=false
    local firmware_only=false
    local web_only=false
    local monitor=false
    local port=""
    
    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -b|--build-only)
                build_only=true
                shift
                ;;
            -f|--firmware-only)
                firmware_only=true
                shift
                ;;
            -w|--web-only)
                web_only=true
                shift
                ;;
            -m|--monitor)
                monitor=true
                shift
                ;;
            -p|--port)
                port="$2"
                shift 2
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                log_error "Unknown option: $1"
                show_help
                exit 1
                ;;
        esac
    done
    
    log_info "ESP32 Gimbal Bridge Flash Tool"
    log_info "==============================="
    
    check_pio
    
    # Auto-detect board if port not specified
    if [ -z "$port" ] && [ "$build_only" = false ]; then
        port=$(find_board)
    fi
    
    # Build
    if [ "$web_only" = false ]; then
        build_firmware
    fi
    
    if [ "$build_only" = true ]; then
        log_info "Build-only mode: skipping flash"
        exit 0
    fi
    
    # Flash firmware
    if [ "$web_only" = false ]; then
        flash_firmware "$port" || {
            log_error "Firmware flash failed!"
            log_warn "Try holding BOOT button, tap RESET, then release BOOT when upload starts"
            exit 1
        }
    fi
    
    # Flash filesystem
    if [ "$firmware_only" = false ]; then
        flash_filesystem "$port" || {
            log_error "Filesystem flash failed!"
            exit 1
        }
    fi
    
    log_info ""
    log_info "==============================="
    log_info "Flash complete!"
    log_info "==============================="
    log_info ""
    log_info "The board will now boot and create a WiFi access point:"
    log_info "  SSID: DJI-Gimbal-Setup"
    log_info "  Connect to configure your WiFi network"
    log_info ""
    
    if [ "$monitor" = true ]; then
        monitor_serial "$port"
    else
        log_info "To view serial output, run:"
        log_info "  pio device monitor -b 115200"
    fi
}

main "$@"

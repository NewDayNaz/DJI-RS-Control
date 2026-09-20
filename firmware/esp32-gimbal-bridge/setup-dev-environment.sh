#!/usr/bin/env bash
# Setup script for ESP32 gimbal bridge development environment
# Supports: Linux, macOS, Windows (Git Bash/WSL)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

detect_os() {
    case "$(uname -s)" in
        Linux*)     echo "linux";;
        Darwin*)    echo "macos";;
        CYGWIN*|MINGW*|MSYS*) echo "windows";;
        *)          echo "unknown";;
    esac
}

check_python() {
    if command -v python3 &> /dev/null; then
        PYTHON_CMD="python3"
    elif command -v python &> /dev/null; then
        PYTHON_CMD="python"
    else
        return 1
    fi
    
    PYTHON_VERSION=$($PYTHON_CMD --version 2>&1 | grep -oP '\d+\.\d+' | head -1)
    PYTHON_MAJOR=$(echo $PYTHON_VERSION | cut -d. -f1)
    PYTHON_MINOR=$(echo $PYTHON_VERSION | cut -d. -f2)
    
    if [ "$PYTHON_MAJOR" -ge 3 ] && [ "$PYTHON_MINOR" -ge 7 ]; then
        echo "$PYTHON_CMD"
        return 0
    fi
    return 1
}

install_python() {
    local os=$1
    log_info "Python 3.7+ is required but not found."
    
    case $os in
        linux)
            log_info "Installing Python 3 via package manager..."
            if command -v apt-get &> /dev/null; then
                sudo apt-get update
                sudo apt-get install -y python3 python3-pip python3-venv
            elif command -v dnf &> /dev/null; then
                sudo dnf install -y python3 python3-pip
            elif command -v yum &> /dev/null; then
                sudo yum install -y python3 python3-pip
            elif command -v pacman &> /dev/null; then
                sudo pacman -S --noconfirm python python-pip
            else
                log_error "Could not detect package manager. Please install Python 3.7+ manually."
                exit 1
            fi
            ;;
        macos)
            log_info "Please install Python from https://www.python.org/downloads/"
            log_info "Or use Homebrew: brew install python3"
            exit 1
            ;;
        windows)
            log_info "Please install Python from https://www.python.org/downloads/"
            log_info "Make sure to check 'Add Python to PATH' during installation!"
            exit 1
            ;;
    esac
}

install_platformio() {
    log_info "Installing PlatformIO Core..."
    
    if [ -z "$PYTHON_CMD" ]; then
        log_error "Python command not set"
        exit 1
    fi
    
    # Try pip3 first, then pip
    if command -v pip3 &> /dev/null; then
        pip3 install --user platformio
    elif command -v pip &> /dev/null; then
        pip install --user platformio
    else
        $PYTHON_CMD -m pip install --user platformio
    fi
    
    log_info "PlatformIO installed successfully!"
}

setup_udev_rules_linux() {
    log_info "Setting up USB device permissions (udev rules)..."
    
    # ESP32 and CP210x USB-to-serial rules
    RULES_FILE="/etc/udev/rules.d/99-platformio-udev.rules"
    
    if [ ! -f "$RULES_FILE" ]; then
        log_info "Creating udev rules file..."
        curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/master/platformio/assets/system/99-platformio-udev.rules | sudo tee "$RULES_FILE" > /dev/null
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        log_info "Udev rules installed. You may need to unplug/replug your device."
        
        # Add user to dialout group
        if groups | grep -q dialout; then
            log_info "User already in 'dialout' group"
        else
            log_warn "Adding user to 'dialout' group..."
            sudo usermod -a -G dialout $USER
            log_warn "You must LOG OUT and LOG BACK IN for group changes to take effect!"
        fi
    else
        log_info "Udev rules already exist"
    fi
}

check_platformio() {
    if command -v pio &> /dev/null; then
        return 0
    fi
    
    # Check if it's in user's local bin
    if [ -f "$HOME/.local/bin/pio" ]; then
        export PATH="$HOME/.local/bin:$PATH"
        return 0
    fi
    
    # Check Python user base bin
    if [ -n "$PYTHON_CMD" ]; then
        USER_BASE=$($PYTHON_CMD -m site --user-base)
        if [ -f "$USER_BASE/bin/pio" ]; then
            export PATH="$USER_BASE/bin:$PATH"
            return 0
        fi
    fi
    
    return 1
}

install_dependencies() {
    log_info "Installing project dependencies..."
    cd "$SCRIPT_DIR"
    
    if check_platformio; then
        pio pkg install
        log_info "PlatformIO libraries installed"
    else
        log_error "PlatformIO not found in PATH after installation"
        log_info "Please add PlatformIO to your PATH manually or restart your terminal"
        exit 1
    fi
}

show_path_instructions() {
    local os=$1
    log_warn ""
    log_warn "=========================================="
    log_warn "IMPORTANT: PlatformIO PATH Setup"
    log_warn "=========================================="
    
    case $os in
        linux|macos)
            log_info "Add this to your ~/.bashrc or ~/.zshrc:"
            log_info "  export PATH=\"\$HOME/.local/bin:\$PATH\""
            log_info ""
            log_info "Then run: source ~/.bashrc  (or restart terminal)"
            ;;
        windows)
            log_info "Add to your PATH in System Environment Variables:"
            log_info "  %USERPROFILE%\\.platformio\\penv\\Scripts"
            log_info ""
            log_info "Or restart your terminal/Git Bash"
            ;;
    esac
    
    log_warn "=========================================="
    log_warn ""
}

main() {
    log_info "ESP32 Gimbal Bridge Development Environment Setup"
    log_info "=================================================="
    
    OS=$(detect_os)
    log_info "Detected OS: $OS"
    
    # Check Python
    log_info "Checking for Python 3.7+..."
    if ! PYTHON_CMD=$(check_python); then
        install_python "$OS"
        if ! PYTHON_CMD=$(check_python); then
            log_error "Failed to install/find Python 3.7+"
            exit 1
        fi
    fi
    log_info "Found Python: $PYTHON_CMD ($($PYTHON_CMD --version))"
    
    # Check PlatformIO
    log_info "Checking for PlatformIO..."
    if ! check_platformio; then
        install_platformio
        
        if ! check_platformio; then
            show_path_instructions "$OS"
        else
            log_info "PlatformIO installed and ready"
        fi
    else
        log_info "PlatformIO already installed ($(pio --version))"
    fi
    
    # Linux-specific USB setup
    if [ "$OS" = "linux" ]; then
        setup_udev_rules_linux
    fi
    
    # Install project dependencies
    if check_platformio; then
        install_dependencies
    fi
    
    # Windows-specific notes
    if [ "$OS" = "windows" ]; then
        log_warn ""
        log_warn "Windows Notes:"
        log_warn "- Use PowerShell or CMD for 'pio' commands (NOT Git Bash)"
        log_warn "- Set: \$env:PYTHONUTF8=1 before running 'pio run -t upload'"
        log_warn "- Or use the provided flash-board.ps1 script"
    fi
    
    log_info ""
    log_info "=========================================="
    log_info "Setup complete!"
    log_info "=========================================="
    log_info ""
    log_info "Next steps:"
    log_info "  1. Connect your XIAO ESP32C3 board via USB"
    log_info "  2. Run the flash script:"
    if [ "$OS" = "windows" ]; then
        log_info "     PowerShell: .\\flash-board.ps1"
    else
        log_info "     ./flash-board.sh"
    fi
    log_info ""
    log_info "Or manually:"
    log_info "  cd firmware/esp32-gimbal-bridge"
    log_info "  pio run -t upload       # flash firmware"
    log_info "  pio run -t uploadfs     # flash web UI"
    log_info "  pio device monitor      # view serial output"
    log_info ""
}

main "$@"

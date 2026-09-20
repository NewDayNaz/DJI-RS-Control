# PowerShell script to build and flash ESP32 gimbal bridge
# Automatically finds the board and handles Windows-specific quirks

param(
    [switch]$BuildOnly,
    [switch]$FirmwareOnly,
    [switch]$WebOnly,
    [switch]$Monitor,
    [string]$Port = "",
    [switch]$Help
)

$ErrorActionPreference = "Stop"

function Write-ColorOutput {
    param([string]$Message, [string]$Color = "White")
    Write-Host $Message -ForegroundColor $Color
}

function Log-Info { Write-ColorOutput "[INFO] $args" "Green" }
function Log-Warn { Write-ColorOutput "[WARN] $args" "Yellow" }
function Log-Error { Write-ColorOutput "[ERROR] $args" "Red" }
function Log-Step { Write-ColorOutput "[STEP] $args" "Cyan" }

function Show-Help {
    Write-Host @"
ESP32 Gimbal Bridge Flash Tool (PowerShell)

Usage: .\flash-board.ps1 [OPTIONS]

Options:
  -BuildOnly          Build without flashing
  -FirmwareOnly       Flash firmware only (skip filesystem)
  -WebOnly            Flash web UI only (skip firmware)
  -Monitor            Open serial monitor after flashing
  -Port <COM#>        Specify serial port manually (e.g., COM3)
  -Help               Show this help message

Examples:
  .\flash-board.ps1                # Build and flash everything
  .\flash-board.ps1 -Monitor       # Flash and open serial monitor
  .\flash-board.ps1 -WebOnly       # Update web UI only
  .\flash-board.ps1 -Port COM3     # Use specific port

"@
    exit 0
}

function Test-PlatformIO {
    try {
        $null = Get-Command pio -ErrorAction Stop
        return $true
    } catch {
        return $false
    }
}

function Find-Board {
    Log-Info "Searching for XIAO ESP32C3 board..."
    
    # Get list of COM ports
    $ports = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
    
    if ($ports.Count -eq 0) {
        Log-Error "No COM ports found!"
        Log-Info "Please check:"
        Log-Info "  1. Board is connected via USB"
        Log-Info "  2. USB cable supports data (not just power)"
        Log-Info "  3. USB drivers are installed"
        Log-Info ""
        Log-Info "Run 'pio device list' to see available devices"
        return $null
    }
    
    # Try to identify ESP32-C3
    $foundPort = $null
    foreach ($port in $ports) {
        try {
            # Try to get device info from WMI
            $wmiPort = Get-WmiObject Win32_PnPEntity | Where-Object {
                $_.Caption -match $port -and 
                ($_.Caption -match "USB.*Serial|CH340|CP210|ESP32|Silicon Labs")
            }
            
            if ($wmiPort) {
                $foundPort = $port
                Log-Info "Found board at: $port ($($wmiPort.Caption))"
                break
            }
        } catch {
            continue
        }
    }
    
    # Fallback: use first available port
    if (-not $foundPort -and $ports.Count -gt 0) {
        $foundPort = $ports[0]
        Log-Warn "Could not identify ESP32-C3 specifically, using first port: $foundPort"
    }
    
    if (-not $foundPort) {
        Log-Error "Could not find suitable serial port"
        Log-Info "Available ports: $($ports -join ', ')"
        return $null
    }
    
    return $foundPort
}

function Build-Firmware {
    Log-Step "Building firmware..."
    
    $scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
    Set-Location $scriptDir
    
    # Set UTF-8 encoding for esptool
    $env:PYTHONUTF8 = "1"
    $env:PYTHONIOENCODING = "utf-8"
    chcp 65001 | Out-Null
    
    pio run -e seeed_xiao_esp32c3
    
    if ($LASTEXITCODE -ne 0) {
        Log-Error "Build failed!"
        exit 1
    }
    
    Log-Info "Build successful!"
}

function Flash-Firmware {
    param([string]$PortName)
    
    Log-Step "Flashing firmware to $PortName..."
    
    $scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
    Set-Location $scriptDir
    
    # Set UTF-8 encoding for esptool
    $env:PYTHONUTF8 = "1"
    $env:PYTHONIOENCODING = "utf-8"
    chcp 65001 | Out-Null
    
    if ($PortName) {
        pio run -e seeed_xiao_esp32c3 -t upload --upload-port $PortName
    } else {
        pio run -e seeed_xiao_esp32c3 -t upload
    }
    
    if ($LASTEXITCODE -ne 0) {
        Log-Error "Firmware flash failed!"
        Log-Warn "Try: Hold BOOT button, tap RESET, release BOOT when upload starts"
        exit 1
    }
    
    Log-Info "Firmware flashed successfully!"
}

function Flash-Filesystem {
    param([string]$PortName)
    
    Log-Step "Flashing web UI (LittleFS filesystem)..."
    
    $scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
    Set-Location $scriptDir
    
    # Set UTF-8 encoding
    $env:PYTHONUTF8 = "1"
    $env:PYTHONIOENCODING = "utf-8"
    chcp 65001 | Out-Null
    
    if ($PortName) {
        pio run -e seeed_xiao_esp32c3 -t uploadfs --upload-port $PortName
    } else {
        pio run -e seeed_xiao_esp32c3 -t uploadfs
    }
    
    if ($LASTEXITCODE -ne 0) {
        Log-Error "Filesystem flash failed!"
        exit 1
    }
    
    Log-Info "Filesystem flashed successfully!"
}

function Start-SerialMonitor {
    param([string]$PortName)
    
    Log-Step "Opening serial monitor (Ctrl+C to exit)..."
    Log-Info "Waiting for board to boot..."
    Start-Sleep -Seconds 2
    
    $scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
    Set-Location $scriptDir
    
    if ($PortName) {
        pio device monitor -p $PortName -b 115200
    } else {
        pio device monitor -b 115200
    }
}

# Main execution
try {
    if ($Help) {
        Show-Help
    }
    
    Log-Info "ESP32 Gimbal Bridge Flash Tool"
    Log-Info "==============================="
    
    # Check PlatformIO
    if (-not (Test-PlatformIO)) {
        Log-Error "PlatformIO not found in PATH"
        Log-Info "Please run .\setup-dev-environment.ps1 first"
        exit 1
    }
    
    # Find board if port not specified
    if (-not $Port -and -not $BuildOnly) {
        $Port = Find-Board
        if (-not $Port) {
            exit 1
        }
    }
    
    # Build
    if (-not $WebOnly) {
        Build-Firmware
    }
    
    if ($BuildOnly) {
        Log-Info "Build-only mode: skipping flash"
        exit 0
    }
    
    # Flash firmware
    if (-not $WebOnly) {
        Flash-Firmware -PortName $Port
    }
    
    # Flash filesystem
    if (-not $FirmwareOnly) {
        Flash-Filesystem -PortName $Port
    }
    
    Log-Info ""
    Log-Info "==============================="
    Log-Info "Flash complete!"
    Log-Info "==============================="
    Log-Info ""
    Log-Info "The board will now boot and create a WiFi access point:"
    Log-Info "  SSID: DJI-Gimbal-Setup"
    Log-Info "  Connect to configure your WiFi network"
    Log-Info ""
    
    if ($Monitor) {
        Start-SerialMonitor -PortName $Port
    } else {
        Log-Info "To view serial output, run:"
        Log-Info "  pio device monitor -b 115200"
    }
    
} catch {
    Log-Error "An error occurred: $_"
    exit 1
}

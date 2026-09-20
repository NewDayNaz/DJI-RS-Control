# PowerShell setup script for ESP32 gimbal bridge development environment
# Run as: .\setup-dev-environment.ps1

$ErrorActionPreference = "Stop"

function Write-ColorOutput {
    param(
        [string]$Message,
        [string]$Color = "White"
    )
    Write-Host $Message -ForegroundColor $Color
}

function Log-Info { Write-ColorOutput "[INFO] $args" "Green" }
function Log-Warn { Write-ColorOutput "[WARN] $args" "Yellow" }
function Log-Error { Write-ColorOutput "[ERROR] $args" "Red" }

function Test-Python {
    $pythonCommands = @("python3", "python")
    
    foreach ($cmd in $pythonCommands) {
        try {
            $version = & $cmd --version 2>&1
            if ($version -match "Python (\d+)\.(\d+)") {
                $major = [int]$Matches[1]
                $minor = [int]$Matches[2]
                
                if ($major -ge 3 -and $minor -ge 7) {
                    return $cmd
                }
            }
        } catch {
            continue
        }
    }
    
    return $null
}

function Install-Python {
    Log-Info "Python 3.7+ is required but not found."
    Log-Info "Please install Python from: https://www.python.org/downloads/"
    Log-Warn "IMPORTANT: Check 'Add Python to PATH' during installation!"
    Log-Info ""
    Log-Info "After installation, restart PowerShell and run this script again."
    
    $response = Read-Host "Open Python download page in browser? (Y/N)"
    if ($response -eq "Y" -or $response -eq "y") {
        Start-Process "https://www.python.org/downloads/"
    }
    
    exit 1
}

function Test-PlatformIO {
    try {
        $null = Get-Command pio -ErrorAction Stop
        return $true
    } catch {
        # Check user Scripts directory
        $userScripts = "$env:APPDATA\Python\Python*\Scripts"
        $pioPath = Get-ChildItem -Path $userScripts -Filter "pio.exe" -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
        
        if ($pioPath) {
            $env:PATH = "$($pioPath.Directory.FullName);$env:PATH"
            return $true
        }
        
        return $false
    }
}

function Install-PlatformIO {
    param([string]$PythonCmd)
    
    Log-Info "Installing PlatformIO Core..."
    
    try {
        & $PythonCmd -m pip install --upgrade pip
        & $PythonCmd -m pip install --user platformio
        
        Log-Info "PlatformIO installed successfully!"
        
        # Add to PATH
        $userScripts = "$env:APPDATA\Python\Python*\Scripts"
        $pioPath = Get-ChildItem -Path $userScripts -Filter "pio.exe" -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
        
        if ($pioPath) {
            $env:PATH = "$($pioPath.Directory.FullName);$env:PATH"
            
            Log-Warn "PlatformIO installed to: $($pioPath.Directory.FullName)"
            Log-Warn "You may need to restart PowerShell for 'pio' command to work"
        }
        
    } catch {
        Log-Error "Failed to install PlatformIO: $_"
        exit 1
    }
}

function Install-Dependencies {
    Log-Info "Installing project dependencies..."
    
    $scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
    Set-Location $scriptDir
    
    if (Test-PlatformIO) {
        pio pkg install
        Log-Info "PlatformIO libraries installed"
    } else {
        Log-Error "PlatformIO not found in PATH after installation"
        Log-Info "Please restart PowerShell and run this script again"
        exit 1
    }
}

function Show-CompletionMessage {
    Log-Info ""
    Log-Info "=========================================="
    Log-Info "Setup complete!"
    Log-Info "=========================================="
    Log-Info ""
    Log-Info "Next steps:"
    Log-Info "  1. Connect your XIAO ESP32C3 board via USB"
    Log-Info "  2. Run the flash script: .\flash-board.ps1"
    Log-Info ""
    Log-Info "Or manually:"
    Log-Info "  cd firmware\esp32-gimbal-bridge"
    Log-Info "  pio run -t upload       # flash firmware"
    Log-Info "  pio run -t uploadfs     # flash web UI"
    Log-Info "  pio device monitor      # view serial output"
    Log-Info ""
}

# Main execution
try {
    Log-Info "ESP32 Gimbal Bridge Development Environment Setup"
    Log-Info "=================================================="
    
    # Check Python
    Log-Info "Checking for Python 3.7+..."
    $pythonCmd = Test-Python
    
    if (-not $pythonCmd) {
        Install-Python
    }
    
    Log-Info "Found Python: $pythonCmd ($(&$pythonCmd --version))"
    
    # Check PlatformIO
    Log-Info "Checking for PlatformIO..."
    
    if (-not (Test-PlatformIO)) {
        Install-PlatformIO -PythonCmd $pythonCmd
        
        if (-not (Test-PlatformIO)) {
            Log-Warn ""
            Log-Warn "=========================================="
            Log-Warn "PlatformIO installed but not in PATH"
            Log-Warn "=========================================="
            Log-Warn "Please restart PowerShell, then run this script again to complete setup."
            Log-Warn ""
            exit 0
        }
    } else {
        $pioVersion = pio --version
        Log-Info "PlatformIO already installed ($pioVersion)"
    }
    
    # Install project dependencies
    Install-Dependencies
    
    # Windows-specific notes
    Log-Warn ""
    Log-Warn "Windows Flashing Notes:"
    Log-Warn "- Always use PowerShell or CMD (NOT Git Bash) for flashing"
    Log-Warn "- The flash-board.ps1 script handles UTF-8 encoding automatically"
    Log-Warn "- If upload fails, hold BOOT button, tap RESET, release BOOT"
    
    Show-CompletionMessage
    
} catch {
    Log-Error "Setup failed: $_"
    exit 1
}

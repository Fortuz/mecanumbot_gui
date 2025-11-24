################################################################################
# Mecanumbot GUI System Startup Script (PowerShell)
# 
# This script starts all components of the mecanumbot GUI system in order:
# 1. joy_node - Reads PS4/Xbox controller hardware
# 2. xbox_controller_node - Publishes controller events to ROS2 topics
# 3. mapping_listener - Maps controller inputs to robot actions
# 4. Docker GUI - Web interface for configuration
#
# Usage: .\start_system.ps1
################################################################################

# Requires PowerShell 5.0+
#Requires -Version 5.0

$ErrorActionPreference = "Stop"

# Colors for output
function Write-ColorOutput($ForegroundColor) {
    $fc = $host.UI.RawUI.ForegroundColor
    $host.UI.RawUI.ForegroundColor = $ForegroundColor
    if ($args) {
        Write-Output $args
    }
    $host.UI.RawUI.ForegroundColor = $fc
}

function Write-Success { Write-ColorOutput Green $args }
function Write-Info { Write-ColorOutput Cyan $args }
function Write-Warning { Write-ColorOutput Yellow $args }
function Write-Error { Write-ColorOutput Red $args }

# Get script directory
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

Write-Info "========================================"
Write-Info "Mecanumbot GUI System Startup"
Write-Info "========================================"
Write-Output ""

# Function to check if a process is running
function Test-ProcessRunning {
    param([string]$ProcessName)
    return (Get-Process -Name "*$ProcessName*" -ErrorAction SilentlyContinue) -ne $null
}

# Function to wait for process to start
function Wait-ForProcess {
    param(
        [string]$ProcessName,
        [int]$MaxWaitSeconds = 10
    )
    
    $count = 0
    while ($count -lt ($MaxWaitSeconds * 2)) {
        if (Test-ProcessRunning $ProcessName) {
            return $true
        }
        Start-Sleep -Milliseconds 500
        $count++
    }
    return $false
}

# Cleanup function
function Stop-AllProcesses {
    Write-Warning "`nShutting down all processes..."
    
    # Stop Python processes
    Get-Process -Name "python*" -ErrorAction SilentlyContinue | Where-Object {
        $_.CommandLine -like "*joy_node*" -or
        $_.CommandLine -like "*xbox_controller_node*" -or
        $_.CommandLine -like "*mapping_listener*"
    } | Stop-Process -Force
    
    # Stop Docker container
    docker stop mecanumbot-gui 2>$null
    
    Write-Success "All processes stopped"
    exit 0
}

# Set trap for Ctrl+C
Register-EngineEvent PowerShell.Exiting -Action { Stop-AllProcesses }

# Check if ROS2 is installed
if (-not (Get-Command ros2 -ErrorAction SilentlyContinue)) {
    Write-Error "ERROR: ROS2 not found in PATH"
    Write-Warning "Please install ROS2 Humble and add to PATH"
    Write-Warning "Visit: https://docs.ros.org/en/humble/Installation.html"
    exit 1
}

# Check ROS2 environment
if (-not $env:ROS_DISTRO) {
    Write-Warning "Sourcing ROS2 Humble..."
    # On Windows, ROS2 is typically sourced via:
    # C:\dev\ros2_humble\local_setup.ps1
    $ros2SetupPath = "C:\dev\ros2_humble\local_setup.ps1"
    if (Test-Path $ros2SetupPath) {
        . $ros2SetupPath
    } else {
        Write-Error "ERROR: Cannot find ROS2 setup script"
        Write-Warning "Please source ROS2 manually before running this script"
        exit 1
    }
}

Write-Success "✓ ROS2 environment configured ($env:ROS_DISTRO)"
Write-Output ""

################################################################################
# Step 1: Start joy_node
################################################################################
Write-Info "[1/4] Starting joy_node..."

# Check if already running
if (Test-ProcessRunning "joy_node") {
    Write-Warning "  joy_node already running, stopping old instance..."
    Get-Process -Name "python*" | Where-Object { $_.CommandLine -like "*joy_node*" } | Stop-Process -Force
    Start-Sleep -Seconds 1
}

# Start joy_node in background
$joyLogPath = "$env:TEMP\joy_node.log"
Start-Process -FilePath "ros2" -ArgumentList "run", "joy", "joy_node" `
    -RedirectStandardOutput $joyLogPath `
    -RedirectStandardError $joyLogPath `
    -WindowStyle Hidden

if (Wait-ForProcess "python") {
    Write-Success "  ✓ joy_node started"
} else {
    Write-Error "  ✗ Failed to start joy_node"
    Write-Warning "  Check log: $joyLogPath"
    exit 1
}
Start-Sleep -Seconds 1

################################################################################
# Step 2: Start xbox_controller_node
################################################################################
Write-Info "[2/4] Starting xbox_controller_node..."

if (Test-ProcessRunning "xbox_controller_node") {
    Write-Warning "  xbox_controller_node already running, stopping old instance..."
    Get-Process -Name "python*" | Where-Object { $_.CommandLine -like "*xbox_controller_node*" } | Stop-Process -Force
    Start-Sleep -Seconds 1
}

# Start xbox_controller_node in background
$xboxLogPath = "$env:TEMP\xbox_controller_node.log"
$xboxPkgPath = Join-Path $ScriptDir "xbox_controller_pkg"
Push-Location $xboxPkgPath
Start-Process -FilePath "python" -ArgumentList "-m", "xbox_controller_pkg.xbox_controller_node" `
    -RedirectStandardOutput $xboxLogPath `
    -RedirectStandardError $xboxLogPath `
    -WindowStyle Hidden
Pop-Location

if (Wait-ForProcess "python") {
    Write-Success "  ✓ xbox_controller_node started"
} else {
    Write-Error "  ✗ Failed to start xbox_controller_node"
    Write-Warning "  Check log: $xboxLogPath"
    exit 1
}
Start-Sleep -Seconds 2

# Verify controller connection
$statusFile = Join-Path $env:USERPROFILE "Documents\controller_status.json"
if (Test-Path $statusFile) {
    $status = Get-Content $statusFile | ConvertFrom-Json
    if ($status.connected -eq $true) {
        Write-Success "  ✓ Controller connected and active"
    } else {
        Write-Warning "  ⚠ Controller status file exists but shows disconnected"
    }
} else {
    Write-Warning "  ⚠ Waiting for controller status file..."
}

################################################################################
# Step 3: Start mapping_listener
################################################################################
Write-Info "[3/4] Starting mapping_listener..."

if (Test-ProcessRunning "mapping_listener") {
    Write-Warning "  mapping_listener already running, stopping old instance..."
    Get-Process -Name "python*" | Where-Object { $_.CommandLine -like "*mapping_listener*" } | Stop-Process -Force
    Start-Sleep -Seconds 1
}

# Start mapping_listener in background
$mappingLogPath = "$env:TEMP\mapping_listener.log"
$mappingPkgPath = Join-Path $ScriptDir "button_mapping_ros"
$env:PYTHONPATH = "$mappingPkgPath;$env:PYTHONPATH"
Push-Location $mappingPkgPath
Start-Process -FilePath "python" -ArgumentList "-m", "button_mapping_ros.mapping_listener" `
    -RedirectStandardOutput $mappingLogPath `
    -RedirectStandardError $mappingLogPath `
    -WindowStyle Hidden
Pop-Location

if (Wait-ForProcess "python") {
    Write-Success "  ✓ mapping_listener started"
} else {
    Write-Error "  ✗ Failed to start mapping_listener"
    Write-Warning "  Check log: $mappingLogPath"
    exit 1
}
Start-Sleep -Seconds 1

################################################################################
# Step 4: Start Docker GUI
################################################################################
Write-Info "[4/4] Starting Docker GUI..."

# Check if Docker is running
if (-not (Get-Process -Name "Docker Desktop" -ErrorAction SilentlyContinue)) {
    Write-Error "  ✗ Docker Desktop is not running"
    Write-Warning "  Please start Docker Desktop and try again"
    exit 1
}

# Check if Docker container is already running
$existingContainer = docker ps -a --filter "name=mecanumbot-gui" --format "{{.Names}}"
if ($existingContainer -eq "mecanumbot-gui") {
    Write-Warning "  Docker container exists, stopping and removing..."
    docker stop mecanumbot-gui 2>$null
    docker rm mecanumbot-gui 2>$null
}

# Ensure Documents directory exists
$docsPath = Join-Path $env:USERPROFILE "Documents"
if (-not (Test-Path $docsPath)) {
    New-Item -ItemType Directory -Path $docsPath | Out-Null
}

# Start Docker container
Push-Location $ScriptDir
$result = docker run -d `
    --name mecanumbot-gui `
    -p 8080:8080 `
    -v "${env:USERPROFILE}\Documents:/host_docs" `
    --network host `
    mecanumbot-gui 2>&1

if ($LASTEXITCODE -eq 0) {
    Write-Success "  ✓ Docker GUI started"
    Start-Sleep -Seconds 2
    
    # Get IP addresses
    $localIP = "127.0.0.1"
    $networkIP = (Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.IPAddress -like "192.168.*" } | Select-Object -First 1).IPAddress
    
    Write-Output ""
    Write-Success "========================================"
    Write-Success "✓ All systems started successfully!"
    Write-Success "========================================"
    Write-Output ""
    Write-Info "Web Interface:"
    Write-Output "  Local:   http://${localIP}:8080"
    if ($networkIP) {
        Write-Output "  Network: http://${networkIP}:8080"
    }
    Write-Output ""
    Write-Info "Log Files:"
    Write-Output "  joy_node:            $joyLogPath"
    Write-Output "  xbox_controller:     $xboxLogPath"
    Write-Output "  mapping_listener:    $mappingLogPath"
    Write-Output "  docker:              docker logs mecanumbot-gui"
    Write-Output ""
    Write-Warning "Press Ctrl+C to stop all processes"
    Write-Output ""
    
    # Wait indefinitely (until Ctrl+C)
    while ($true) {
        Start-Sleep -Seconds 1
    }
} else {
    Write-Error "  ✗ Failed to start Docker GUI"
    Write-Warning "  Error: $result"
    Write-Warning "  Check Docker logs: docker logs mecanumbot-gui"
    exit 1
}
Pop-Location

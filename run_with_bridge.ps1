# PowerShell script to run the complete controller bridge solution

Write-Host "Controller Bridge Solution - Windows Setup" -ForegroundColor Green
Write-Host "=========================================" -ForegroundColor Green

# Check if controller_bridge.py exists
if (-not (Test-Path "controller_bridge.py")) {
    Write-Host "Error: controller_bridge.py not found!" -ForegroundColor Red
    Write-Host "Make sure you're running this from the correct directory." -ForegroundColor Red
    exit 1
}

Write-Host "Step 1: Starting Controller Bridge Service on host..." -ForegroundColor Yellow
Write-Host "This will run in a separate window. Keep it open!" -ForegroundColor Yellow

# Start the bridge service in a new PowerShell window
Start-Process powershell -ArgumentList "-NoExit", "-Command", "cd '$PWD'; python controller_bridge.py"

# Wait a moment for the service to start
Write-Host "Waiting for bridge service to start..." -ForegroundColor Yellow
Start-Sleep -Seconds 3

# Test if bridge service is running
try {
    $response = Invoke-WebRequest -Uri "http://localhost:8899/status" -TimeoutSec 5
    Write-Host "[OK] Bridge service is running!" -ForegroundColor Green
} catch {
    Write-Host "[WARNING] Bridge service may not be ready yet. Continue anyway..." -ForegroundColor Yellow
}

Write-Host "`nStep 2: Building Docker image..." -ForegroundColor Yellow
docker build -t controller-app .

if ($LASTEXITCODE -ne 0) {
    Write-Host "Error building Docker image!" -ForegroundColor Red
    exit 1
}

Write-Host "`nStep 3: Starting Docker container..." -ForegroundColor Yellow
Write-Host "The app will be available at http://localhost:8080" -ForegroundColor Cyan
Write-Host "Controllers will be detected via the bridge service!" -ForegroundColor Cyan

# Run Docker with host networking for bridge communication
docker run -p 8080:8080 --add-host=host.docker.internal:host-gateway controller-app

Write-Host "`nDocker container stopped." -ForegroundColor Yellow
Write-Host "Please close the Controller Bridge Service window manually." -ForegroundColor Yellow
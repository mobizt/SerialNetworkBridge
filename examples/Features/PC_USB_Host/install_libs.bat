@echo off
:: Check for admin rights (Required for installing global packages and for netsh commands later)
net session >nul 2>&1
if %errorLevel% == 0 (
    echo Running as Administrator...
) else (
    echo Requesting Administrator privileges...
    powershell -Command "Start-Process '%~f0' -Verb RunAs"
    exit /b
)

cd /d "%~dp0"
echo Installing Python Dependencies...
python -m pip install --upgrade pip
:: 'flask' is optional but included for the HTTP Streaming server example
python -m pip install pyserial cobs websocket-client flask

echo.
echo Dependencies installed successfully.
echo You can now run 'run.bat' to start the bridge.
pause
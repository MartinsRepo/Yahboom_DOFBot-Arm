@echo off
echo ===================================================
echo   Yahboom DOFBot Arm - USB Serial Device Attach
echo ===================================================
echo.

where usbipd >nul 2>nul
if %errorlevel% neq 0 (
    echo [ERROR] usbipd-win ist nicht installiert oder nicht im PATH.
    echo Bitte installieren Sie usbipd (winget install dorssel.usbipd-win).
    echo.
    pause
    exit /b 1
)

echo Verbinde Arm-Seriell-Hardware (CH340 VID: 1A86, PID: 7523) mit WSL...
usbipd attach --wsl --hardware-id 1a86:7523

echo.
echo ===================================================
echo Status der USB-Gerate in usbipd:
echo ===================================================
usbipd list | findstr /i "1a86:7523"

echo.
echo Die Arm-Seriell-Hardware wurde an WSL / Podman ubergeben (/dev/ttyUSB0).
timeout /t 4

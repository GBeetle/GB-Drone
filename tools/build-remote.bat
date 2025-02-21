@echo off
setlocal enabledelayedexpansion

call env_setting.bat

:GetComPort
    set "COM_PORT="
    set "COM_COUNT=0"

    :: get com port and count
    for /f "tokens=*" %%A in ('wmic path Win32_SerialPort get DeviceID ^| findstr /R "^COM[0-9]"') do (
        set /a COM_COUNT+=1
        set COM_PORT=%%A
    )

cd ../source/remote

if "%~1"=="init" (
    idf.py set-target esp32s3
    idf.py add-dependency esp_tinyusb
    :: create soft link
    call pre_processing.bat
    exit /b
)

if "%~1"=="config" (
    start cmd /k "idf.py menuconfig"
    exit /b
)

if "%~1"=="clean" (
    idf.py clean
    exit /b
)

if "%~1"=="fullclean" (
    idf.py fullclean
    exit /b
)

if "%~1"=="build" (
    echo "start building"
    idf.py build
    exit /b
)

if "%~1"=="flash" (
    if !COM_COUNT! equ 1 (
        echo only have one device: !COM_PORT!
    ) else if !COM_COUNT! gtr 1 (
        echo ERROR: TOO MANY DEVICES
        exit /b
    ) else (
        echo ERROR: NO DEVICE CONNECTED
        exit /b
    )

    idf.py -p %COM_PORT% flash
    exit /b
)

if "%~1"=="run" (
    if !COM_COUNT! equ 1 (
        echo only have one device: !COM_PORT!
    ) else if !COM_COUNT! gtr 1 (
        echo ERROR: TOO MANY DEVICES
        exit /b
    ) else (
        echo ERROR: NO DEVICE CONNECTED
        exit /b
    )

    idf.py -p %COM_PORT% monitor
    exit /b
)

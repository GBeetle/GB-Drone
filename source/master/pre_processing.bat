@echo off
setlocal

rem set source dir and target dir
set "vendorSourceDir=%cd%\..\vendor\esp32"
set "utilitySourceDir=%cd%\..\utility"
set "targetDir=%cd%\components"
set "vendorSourceFolders=i2c_bus log_sys spi_bus gb_timer gpio_setting esc file_system"
set "utilitySourceFolders=Fusion error_handle mpu_driver lis3mdl bmp280 nrf24"

rem create target dir
if not exist "%targetDir%" (
    mkdir "%targetDir%"
)

echo %vendorSourceDir%
echo %utilitySourceDir%
echo %targetDir%

rem create soft-link
for %%f in (%vendorSourceFolders%) do (
    if not exist "%targetDir%\%%f" (
        mklink /j "%targetDir%\%%f" "%vendorSourceDir%\%%f"
        echo Created symlink for %%f
    ) else (
        echo Symlink for %%f already exists
    )
)

for %%f in (%utilitySourceFolders%) do (
    if not exist "%targetDir%\%%f" (
        mklink /j "%targetDir%\%%f" "%utilitySourceDir%\%%f"
        echo Created symlink for %%f
    ) else (
        echo Symlink for %%f already exists
    )
)

endlocal

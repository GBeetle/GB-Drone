@echo off
setlocal

rem set source dir and target dir
set "sourceDir=%cd%\..\vendor\esp32"
set "targetDir=%cd%\components"
set "sourceFolders=i2c_bus error_handle log_sys spi_bus"

rem create target dir
if not exist "%targetDir%" (
    mkdir "%targetDir%"
)

echo %sourceDir%
echo %targetDir%

rem create soft-link
for %%f in (%sourceFolders%) do (
    if not exist "%targetDir%\%%f" (
        mklink /j "%targetDir%\%%f" "%sourceDir%\%%f"
        echo Created symlink for %%f
    ) else (
        echo Symlink for %%f already exists
    )
)

endlocal

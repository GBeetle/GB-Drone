call env_setting.bat

cd ../source/remote

if "%~1"=="config" (
    start cmd /k "idf.py menuconfig"
    exit /b
)

if "%~1"=="clean" (
    idf.py fullclean
    exit /b
)

echo "start building"
idf.py build

# AGENTS.md — GB-Drone

## Project overview

Two ESP-IDF C projects in one repo:

| Target | MCU | Role | Source |
|---|---|---|---|
| **master** | ESP32-S3 | Drone flight controller | `source/master/` |
| **remote** | ESP32-P4 | Handheld controller | `source/remote/` |

Shared code lives in `source/utility/` (drone-specific drivers) and `source/vendor/esp32/` (ESP32 HAL). Both are symlinked into each project's `components/` by `pre_processing.bat`.

## Build commands (Windows only)

**Prerequisite:** ESP-IDF v5.4 installed at `D:\Espressif` (see `.vscode/settings.json`).

All builds run from `tools/`:

```
cd tools

# Master (drone) — target esp32s3
build-master.bat init     # one-time: set-target + symlinks
build-master.bat build
build-master.bat flash
build-master.bat run      # serial monitor

# Remote (controller) — target esp32p4
build-remote.bat init
build-remote.bat build
build-remote.bat flash
build-remote.bat run

# Clean
build-master.bat clean     # idf.py clean
build-master.bat fullclean # idf.py fullclean (removes sdkconfig, managed_components, etc.)

# Menuconfig
build-master.bat config
```

- `build-*.bat flash` / `build-*.bat run` auto-detect the single COM port. They **refuse** if 0 or >1 serial devices are connected.
- Never run `idf.py` directly; the BAT scripts handle environment setup, working directory, and COM port detection.

## Symlink architecture

`pre_processing.bat` creates Windows directory junctions (`mklink /j`) under `components/`:
- `source/vendor/esp32/<name>` → `components/<name>`
- `source/utility/<name>` → `components/<name>`

**Always run `init` once per project** before the first build. If a new component is added to vendor/ or utility/, update both `pre_processing.bat` and the component's CMakeLists.txt. Symlinks are in `.gitignore` — they are not committed.

## Component pattern (ESP-IDF)

Each component's `CMakeLists.txt` follows this exact pattern:

```cmake
set(COMPONENT_SRCS "file1.c" "file2.c")
set(COMPONENT_REQUIRES log_sys other_dep)
set(COMPONENT_ADD_INCLUDEDIRS "include")
register_component()
```

Do not use `idf_component_register()` — this codebase uses the legacy `register_component()`.

## Key entry points

- Master main: `source/master/main/app_main.c` — creates 4 FreeRTOS tasks (sensor fusion/PID, sensor read, nRF24 radio, UART debug)
- Remote main: `source/remote/main/app_main.c` — creates 3 FreeRTOS tasks (controller input, LVGL GUI, nRF24 radio)
- Flight control loop: `source/master/components/task_manager/flight_control.c`
- Radio protocol: `source/utility/nrf24/include/lora_state.h`
- Pin definitions: `source/master/components/io_define/` and `source/remote/components/io_define/`

## Naming conventions

- `GB_` / `GB_` prefix for project-global types, macros, and enums
- `gb_` prefix for functions
- Include guards: `#ifndef _NAME__` / `#define _NAME__` / `#endif`
- Error handling: `CHK_FUNC_EXIT()`, `CHK_LOGE()` macros; `GB_RESULT` return pattern

## Calibration (Python)

Gyro/accel calibration: `tools/gyro_acc_calibration.py`
Magnetometer calibration: `tools/mag_calibration.py`

These are standalone Python scripts, not part of the ESP-IDF build.

## License

GPL v3. All source files carry a GPL license header.

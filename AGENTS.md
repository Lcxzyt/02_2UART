# Repository Guidelines

## Project Structure & Module Organization

This repository contains TI MSPM0G3507 firmware built with Code Composer Studio (CCS), DriverLib, and SysConfig. Application sources are grouped under `User/`: `main/` owns startup and the superloop, `bsp/` contains hardware drivers, `app/` implements control and UI features, `task/` coordinates autonomous behavior, and `systool/` provides shared utilities such as PID and delays. Hardware configuration lives in `empty.syscfg`; debugger settings are in `targetConfigs/`. Keep design notes and wiring records in `docs/`, reference manuals in `资料/`, and offline analysis utilities in `py/`. `Debug/` contains CCS-generated build metadata and outputs.

## Build, Test, and Development Commands

- `gmake -C Debug all` builds `Debug/02_2UART.out` using the TI Arm Clang toolchain configured in the generated makefile.
- `gmake -C Debug clean` removes generated objects and the firmware image.
- In CCS, use **Project > Build Project** after changing `empty.syscfg`; this regenerates pin, peripheral, and make metadata before compiling.
- `python py/yaw_test_analysis.py` (and the other `py/*_analysis.py` scripts) runs captured-data analysis; inspect each script's input constants before use.

The generated makefile currently expects CCS under `D:/CCS/ccs`. If your installation differs, build through CCS or regenerate the Debug configuration instead of hand-editing generated files.

## Coding Style & Naming Conventions

Use C with four-space indentation and opening braces on the next line for functions. Match existing naming: public APIs use `Module_Action`, types use `Module_Name`, macros use `UPPER_SNAKE_CASE`, and file-local state uses `static` lower-case names. Pair each module's `.c` and `.h`, use fixed-width integer types, suffix unsigned constants (`20U`), and mark intentionally ignored return values with `(void)`. Do not manually edit files labeled “Automatically-generated.”

## Testing Guidelines

There is no unit-test framework or coverage gate. A change must compile cleanly and be validated on hardware. Exercise affected USB/Bluetooth commands, confirm OLED and sensor behavior, and verify the 20 ms control loop and fail-safe stop remain responsive. For control changes, record representative telemetry and analyze it with the scripts in `py/`. Document new test procedures or pin changes in `docs/` and `README.md`.

## Commit & Pull Request Guidelines

History favors short, imperative, feature-focused subjects, in either Chinese or English (for example, `Stabilize ADC line following`). Keep each commit scoped to one behavior. Pull requests should explain the hardware/configuration affected, list build and bench-test results, link the relevant issue, and include logs, plots, or wiring photos when behavior cannot be reviewed from code alone.

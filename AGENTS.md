# AGENTS.md — Solar Panel I-V Curve Tracer

## Project overview

ESP32-C3 firmware that traces the I-V curve of a solar panel. It sweeps a PWM-controlled electronic load (op-amp VCCS + MOSFET), reads voltage and current via an INA219 over I2C, and serves the results through a Wi-Fi soft-AP web interface built with Chart.js. A SH1106 OLED and a rotary encoder provide a local menu UI.

Target: **ESP32-C3**, ESP-IDF **5.5.1**, RISC-V toolchain.

---

## Repository layout

```text
main/
  main.c              # app_main — hardware init, task launch, deep-sleep wakeup
  measurement.c/h     # auto-ranged PWM sweep loop, INA219 acquisition, demo producer
  dynamic_load.c/h    # encoder-driven manual electronic load (independent of the sweep)
  ui.c/h              # OLED menu state machine and rendering
  app/
    app_state.h/.c    # g_app singleton + mutex
    app_tasks.h/.c    # FreeRTOS task creation (display, encoder, producer)
    app_hw.h/.c       # I2C bus init
  drivers/
    driver_ina219     # INA219 I2C driver (32 V / 10 A calibration preset)
    driver_sh1106     # SH1106 OLED driver (framebuffer, text, QR)
    driver_encoder    # Rotary encoder GPIO ISR
    pwm_controller    # LEDC wrapper (GPIO 8, 8 kHz, 13-bit)
  utils/
    init.c/h          # Wi-Fi soft-AP, NVS, SPIFFS, HTTP server startup
    json_builder      # Lightweight JSON array serialiser for /data endpoint
    led_controller    # WS2812 RGB LED (GPIO 10)
  server/
    server.c/h        # HTTP handlers: /, /data, /start-measurement, /ota, /ota/spiffs, /version, /wifi-config
  db/
    db.c/h            # Circular sample buffer (max 20 points), mutex-protected
components/
  sweep_plan          # Pure sweep math: per-step duty placement + auto-range state
                       # machine. No hardware/FreeRTOS deps, so it builds on the
                       # `linux` IDF target — this is what test/host/ exercises.
  json_builder        # Pure JSON array serialiser for /data (also linux-target testable)
test/host/            # Standalone ESP-IDF project, `linux` target, Unity host tests
                       # for components/sweep_plan and components/json_builder
spiffs/               # Static web files bundled into SPIFFS partition
  index.html / script.js / chart.umd.min.js / ota.html
partitions.csv        # Custom flash layout (see below)
sdkconfig.defaults    # Canonical build config — edit this, not sdkconfig
```

---

## Hardware pinout

| Signal | GPIO |
| --- | --- |
| I2C SDA (INA219 + SH1106) | 6 |
| I2C SCL | 7 |
| PWM load control (LEDC) | 8 |
| WS2812 LED | 10 |
| Encoder DT | 2 |
| Encoder CLK | 3 |
| Encoder button (SW) | 4 |

I2C addresses: INA219 → `0x40`, SH1106 → `0x3C`, bus speed 100 kHz.

---

## Flash layout

Partition table offset: `0xD000` (pushed up to fit the secure-boot-signed bootloader).

| Partition | Offset | Size |
| --- | --- | --- |
| nvs | 0xE000 | 16 KB |
| otadata | 0x12000 | 8 KB |
| phy_init | 0x14000 | 4 KB |
| ota_0 | 0x20000 | 1088 KB |
| ota_1 | 0x130000 | 1088 KB |
| storage (SPIFFS) | 0x240000 | 1792 KB |

---

## Key design patterns

- **Global state**: single `g_app` (app_state_t) struct; always acquire `g_app.state_mtx` before touching measurement fields.
- **Producer/consumer**: producer task sweeps PWM and writes to `db`; display task reads from `db` and renders; HTTP `/data` snapshots `db`.
- **Two producer modes**: `producer_task` (real INA219 hardware) and `dummy_producer_task` (synthetic curve for testing without hardware).
- **Dynamic load**: encoder adjusts PWM setpoint live; soft power cap at 2000 mW with 150 mW hysteresis.
- **OTA**: dual A/B slots; update via `/ota` HTTP endpoint or `idf.py ota`.
- **Auto-range sweep**: each trace probes Voc at zero load, then doubles the commanded PWM duty until the panel
  collapses, to locate the knee of the I-V curve without an operator-entered current range. The sweep's 20 points
  are then placed mostly across that knee (a coarse leg below it, most of the budget through it, a short tail up
  to Isc), so a small panel still gets a well-resolved curve shape. The sweep hard-stops (aborts, keeping points
  already recorded) if measured power reaches 5000 mW.

---

## Build & flash

```sh
# First time — set target
idf.py set-target esp32c3

# Build
idf.py build

# Flash + monitor
idf.py flash monitor
```

`sdkconfig` is generated from `sdkconfig.defaults` — do not commit `sdkconfig`.

---

## Tests

Host unit tests cover the pure sweep math in `components/sweep_plan` (duty
placement in `sweep_plan_build`, the auto-range state machine in
`sweep_range_*`) and the JSON serialiser in `components/json_builder`. They
run on the ESP-IDF `linux` target via Unity, with no ESP32 hardware and no
QEMU, and are wired into CI as the `host-tests` job.

```sh
cd test/host
idf.py --preview set-target linux   # once, or after switching from another target
idf.py build
./build/host_tests.elf              # exit code is Unity's failure count
```

The firmware itself is only compile-checked in CI (`idf.py build` at the
repo root); it has no on-device test harness.

---

## Secure boot

RSA Secure Boot V2 is enabled. A signing key is required:

```sh
idf.py secure-generate-signing-key secure_boot_signing_key.pem
```

Current mode: **Development** — flash encryption active but UART download still works.  
`secure_boot_signing_key.pem` must never be lost; without it OTA updates are impossible on Release-mode devices.

---

## Web interface

Connect to Wi-Fi SSID `ESP32_PLOT` (no password) then open `http://192.168.4.1`.  
The `/data` endpoint returns `[{x: voltage, y: current}, ...]` as JSON.

# AGENTS.md: Solar Panel I-V Curve Tracer

## Project overview

ESP32-C3 firmware that traces the I-V curve of a solar panel. It sweeps a PWM-controlled electronic load (op-amp VCCS + MOSFET), reads voltage and current via an INA219 over I2C, and serves the results through a Wi-Fi soft-AP web interface built with Chart.js. A SH1106 OLED and a rotary encoder provide a local menu UI.

Target: **ESP32-C3**, ESP-IDF **5.5.1**, RISC-V toolchain.

---

## Repository layout

```text
main/
  main.c              # app_main: hardware init, task launch, deep-sleep wakeup
  measurement.c/h     # auto-ranged PWM sweep loop, INA219 acquisition, demo producer,
                       # producer task creation
  dynamic_load.c/h    # encoder-driven manual electronic load (independent of the sweep)
  ui.c/h              # OLED menu state machine and rendering
  app/
    app_state.h/.c    # g_app singleton + mutex
    app_tasks.h/.c    # FreeRTOS task creation (display, encoder, producer)
    app_hw.h/.c       # I2C bus init
  drivers/
    driver_ina219     # INA219 I2C driver (32 V / 10 A calibration preset)
    driver_sh1106     # SH1106 OLED driver (framebuffer, text, QR)
    driver_encoder    # Rotary encoder GPIO ISR: quadrature decode (components/quadrature)
                       # for rotation, esp_timer settle + ANYEDGE state machine for the button
    pwm_controller    # LEDC wrapper (GPIO 8, 8 kHz, 13-bit)
  utils/
    init.c/h          # Wi-Fi soft-AP, NVS, HTTP server startup
    led_controller    # WS2812 RGB LED (GPIO 10)
  server/
    server.c/h        # HTTP handlers, see endpoint table below
  db/
    db.c/h            # Circular sample buffer (max 20 points), mutex-protected
  web/                # Web UI (HTML/CSS/JS + Chart.js), gzip-compressed and embedded
                       # into the app image at build time, no SPIFFS and no
                       # separate storage partition
components/
  sweep_plan          # Pure sweep math: per-step duty placement + auto-range state
                       # machine. No hardware/FreeRTOS deps, so it builds on the
                       # `linux` IDF target, which is what test/host/ exercises.
  json_builder        # Pure JSON array serialiser for /data (also linux-target testable)
  quadrature          # Pure Gray-code quadrature decoder (no hardware/FreeRTOS deps,
                       # linux-target testable) used by driver_encoder for rotation
test/host/            # Standalone ESP-IDF project, `linux` target, Unity host tests
                       # for components/sweep_plan and components/json_builder
partitions.csv        # Custom flash layout (see below); storage partition unused
sdkconfig.defaults    # Canonical build config: edit this, not sdkconfig
sdkconfig.secure      # Secure Boot overlay, merged on top of sdkconfig.defaults for
                       # the secure-boot build variant only
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
| storage | 0x240000 | 1792 KB |

`storage` is unused: the web UI is gzip-embedded in the app image, not served from SPIFFS.

---

## Key design patterns

- **Global state**: single `g_app` (app_state_t) struct; always acquire `g_app.state_mtx` before touching measurement fields.
- **Producer/consumer**: producer task sweeps PWM and writes to `db`; display task reads from `db` and renders; HTTP `/data` snapshots `db`.
- **Two producer modes**: `producer_task` (real INA219 hardware) and `dummy_producer_task` (synthetic curve for testing without hardware).
- **Detent-less encoder**: the rotary encoder has no tactile detents, so `ui.c` clamps list navigation
  (home sections, submenus, measure screen items) at the first/last item instead of wrapping, and draws
  the selected row inverted with a "N/total" position hint. The OLED blanks itself (`0xAE`) after
  `OLED_IDLE_TIMEOUT_S` (default 60 s) of no encoder activity, except on the dynamic load screen; the
  first encoder event after that only wakes the panel and is otherwise swallowed.
- **Dynamic load**: encoder adjusts PWM setpoint live, capped at `DYNAMIC_LOAD_DUTY_MAX_PERCENT` (10% duty). It
  shares the same `LOAD_POWER_LIMIT_MW` (5000 mW) power cap as the sweep, with a hysteresis margin
  (`LOAD_POWER_NEAR_MARGIN_MW`) before backing off duty.
- **OTA**: single-file app update via the `/ota` HTTP endpoint (`app-standard.bin`), no `idf.py ota` command. No
  app rollback: the newly flashed OTA slot is committed on the next boot.
- **Auto-range sweep**: each trace probes Voc at zero load, then doubles the commanded PWM duty until the panel
  collapses, to locate the knee of the I-V curve without an operator-entered current range. The sweep's 20 points
  are then placed mostly across that knee (a coarse leg below it, most of the budget through it, a short tail up
  to Isc), so a small panel still gets a well-resolved curve shape. The sweep hard-stops (aborts, keeping points
  already recorded) if measured power reaches the shared 5000 mW power cap.

---

## Build & flash

```sh
# First time: set target
idf.py set-target esp32c3

# Build
idf.py build

# Flash + monitor
idf.py flash monitor
```

`sdkconfig` is generated from `sdkconfig.defaults`: do not commit `sdkconfig`.

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

Required checks before merge: `compile-check` (standard variant), `compile-check-secure`
(secure-boot variant), `host-tests`.

---

## Releases

Every push to `main` replaces a single rolling GitHub release tagged `firmware`
(https://github.com/fborello-lambda/solar_panel_curve_tracer/releases/latest) with three assets:

- `app-standard.bin`: OTA update image for everyone, upload via `/ota`.
- `factory-standard.bin`: full-flash image for a brand-new or blank board over USB:
  `esptool.py --chip esp32c3 write_flash 0x0 factory-standard.bin`.
- `app-secure-boot.bin`: signed OTA image for the maintainer's one Secure Boot board only.

---

## Secure boot

Secure Boot is **not** enabled by default. There are two build variants, both from the same
`sdkconfig.defaults`:

- **Standard** (`idf.py build`): no Secure Boot, no flash encryption. What everyone builds and
  flashes.
- **Secure** (`sdkconfig.defaults` plus the `sdkconfig.secure` overlay): RSA Secure Boot V2 for
  the maintainer's one fused board only. Build with:

  ```sh
  idf.py -B build-secure -D SDKCONFIG=build-secure/sdkconfig \
      -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.secure" build
  ```

  and flash with `./secure_boot_flash.sh`, which refuses to run if the build isn't actually
  Secure Boot signed.

Flash encryption is not used in either variant. `secure_boot_signing_key.pem` is committed to the
repo on purpose for this experimental board; see [LESSONS.md](LESSONS.md) for why that would be a
bad idea in production. Once Secure Boot's public key digest is burned into a board's eFuses it
cannot be disabled, and losing the signing key means that board can never be updated again.

---

## Web interface

Connect to Wi-Fi SSID `ESP32_PLOT` (no password) then open `http://192.168.4.1`.

| Endpoint | Method | Purpose |
| --- | --- | --- |
| `/` | GET | Web UI (ES/EN, light/dark theme) |
| `/guide` | GET | Operator quick guide (ES/EN) |
| `/ota` | GET | Firmware update page |
| `/ota` | POST | Firmware update upload (single app image) |
| `/data?have=N` | GET | Sample data; `[{x: voltage, y: current}, ...]`, or `{"count":N}` if the client already has all points |
| `/status` | GET | Current measurement status |
| `/measurement/start` | POST | Start a sweep |
| `/measurement/stop` | POST | Stop a sweep |
| `/version` | GET | Firmware version string |
| `/wifi-config` | POST | Update the Wi-Fi soft-AP configuration |

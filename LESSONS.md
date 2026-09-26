# LESSONS.md: Hardware lessons learnt

## No battery disconnect switch

The prototype has no switch between the battery and the rest of the circuit. This makes it impossible to cut power without physically unplugging the battery, which is inconvenient during development and potentially unsafe during debugging.

**Next revision:** add a simple SPDT or slide switch (or a P-channel MOSFET soft-power switch) in series with the battery positive rail.

## No copper plane under the MOSFET

The MOSFET dissipates heat during load sweeps and the perfboard layout has no thermal relief. Without a copper pour or heatsink pad under the package the junction temperature rises quickly, affecting the VCCS linearity and risking thermal shutdown.

**Next revision:** add a copper polygon (ground plane) under the MOSFET's drain pad and consider a small heatsink or exposed-pad footprint tied to a copper fill.

## OLED screen footprint is mirrored

The SH1106 OLED footprint on the PCB was placed mirrored, so the connector pins are reversed relative to the actual module. The screen had to be bodge-wired to fit.

**Next revision:** verify connector orientation against the physical module datasheet before placing the footprint, and add a pin-1 marker to the silkscreen.

## C1, C2 and R8 missing their ground connection

On the rev1 board, `C1`, `C2` and `R8` are part of the RC low-pass filter that
turns the PWM load-control signal into the analog reference voltage feeding
the op-amp's (U1A) non-inverting input, but the filter's return path to GND
was left unconnected in the layout/schematic: none of the three net to
ground. This was missed because nothing about the board flags an unconnected
pin as an error by itself; it only shows up as filter/VCCS misbehavior on the
bench.

**Next revision:** tie C1, C2 and R8 to GND, and run a DRC/ERC check for
unconnected pins before ordering.

## Secure Boot

Secure Boot on ESP32 burns the public key digest into eFuses permanently. Once enabled it cannot be disabled, and flashing any firmware signed with a different key will cause the device to boot-loop and become unrecoverable. Losing the private signing key bricks the device.

**When it is worth using:** only if the firmware contains secrets or logic that must not be tampered with (proprietary algorithms, credential storage, safety-critical control). For a development board or open-source project it adds risk with little benefit.

**If you do use it:**
- Back up `secure_boot_signing_key.pem` immediately after generation: store it in a password manager or encrypted offline location, never only on the project machine.
- Add `*.pem` to `.gitignore` to avoid accidental commits, but keep the backup elsewhere.
- Be aware that `esptool` in Secure Download Mode cannot read flash or eFuses, and a large partition erase (over roughly 1 MB at one address range) may fail; `secure_boot_flash.sh` derives its flash offsets from the build's own `flash_args` instead of hardcoding them, so it stays correct if the partition table changes.
- The web UI is embedded in the app image now (no SPIFFS, no storage partition to flash separately), so OTA updates are a single app image over `/ota` and this workaround no longer applies to it.

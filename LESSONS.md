# LESSONS.md — Hardware lessons learnt

## No battery disconnect switch

The prototype has no switch between the battery and the rest of the circuit. This makes it impossible to cut power without physically unplugging the battery, which is inconvenient during development and potentially unsafe during debugging.

**Next revision:** add a simple SPDT or slide switch (or a P-channel MOSFET soft-power switch) in series with the battery positive rail.

## No copper plane under the MOSFET

The MOSFET dissipates heat during load sweeps and the perfboard layout has no thermal relief. Without a copper pour or heatsink pad under the package the junction temperature rises quickly, affecting the VCCS linearity and risking thermal shutdown.

**Next revision:** add a copper polygon (ground plane) under the MOSFET's drain pad and consider a small heatsink or exposed-pad footprint tied to a copper fill.

## OLED screen footprint is mirrored

The SH1106 OLED footprint on the PCB was placed mirrored, so the connector pins are reversed relative to the actual module. The screen had to be bodge-wired to fit.

**Next revision:** verify connector orientation against the physical module datasheet before placing the footprint, and add a pin-1 marker to the silkscreen.

## Secure Boot

Secure Boot on ESP32 burns the public key digest into eFuses permanently. Once enabled it cannot be disabled, and flashing any firmware signed with a different key will cause the device to boot-loop and become unrecoverable. Losing the private signing key bricks the device.

**When it is worth using:** only if the firmware contains secrets or logic that must not be tampered with (proprietary algorithms, credential storage, safety-critical control). For a development board or open-source project it adds risk with little benefit.

**If you do use it:**
- Back up `secure_boot_signing_key.pem` immediately after generation — store it in a password manager or encrypted offline location, never only on the project machine.
- Add `*.pem` to `.gitignore` to avoid accidental commits, but keep the backup elsewhere.
- Be aware that `esptool` in Secure Download Mode cannot read flash or eFuses, and large partition erases (>~1 MB at one address range) may fail — flash the SPIFFS partition separately with `--flash_size 4MB` instead of `keep`.
- Use the `/ota/spiffs` HTTP endpoint to update the SPIFFS image over the network and avoid the serial flashing issue entirely.

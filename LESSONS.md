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

## One-file OTA: embed the web UI in the firmware

The first versions kept the web UI in a separate SPIFFS partition, so every update needed two
files (`app.bin` and `storage.bin`). That caused real problems: a failed or wrong storage upload
erased every page including the update page itself (USB-only recovery), the app and the pages
could drift to different versions, and on the Secure Boot board the large SPIFFS erase didn't
work in Secure Download Mode.

**Before:** the build packed `spiffs/` into its own partition and the server read files from a
filesystem at runtime.

```cmake
spiffs_create_partition_image(storage ../spiffs FLASH_IN_PROJECT)   # -> storage.bin
```

```c
FILE *f = fopen("/spiffs/index.html", "rb");        // filesystem read per request
httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=86400");
while ((n = fread(buf, 1, sizeof(buf), f)) > 0) httpd_resp_send_chunk(req, buf, n);
```

**After:** CMake gzips each web file at build time and links it into the app as a byte array;
the server sends those bytes straight from flash, no filesystem involved.

```cmake
add_custom_command(OUTPUT index.html.gz COMMAND python3 -c "gzip ... mtime=0" ...)
target_add_binary_data(${COMPONENT_LIB} index.html.gz BINARY)   # -> _binary_index_html_gz_start/_end
```

```c
extern const uint8_t index_html_gz_start[] asm("_binary_index_html_gz_start");
httpd_resp_set_hdr(req, "Content-Encoding", "gzip");   // the browser unzips it on its own
httpd_resp_set_hdr(req, "ETag", "\"<firmware version>\"");   // 304 until the next update
httpd_resp_send(req, (const char *)start, end - start);
```

The ESP32 never decompresses anything: it serves the gzipped bytes as-is with
`Content-Encoding: gzip`, and every browser inflates them transparently. That is what makes it
fit: the UI is about 250 KB raw but about 90 KB gzipped (Chart.js alone goes from 208 KB to
70 KB), plus `-Os` and dropping unused IPv6/WPA3 to free app space.

| | Before | After |
| --- | --- | --- |
| Files per update | `app.bin` + `storage.bin` | `app-standard.bin` |
| Update endpoints | `/ota` + `/ota/spiffs` (erased first) | `/ota` |
| Bad upload can delete the update page | yes | no, it ships inside the app |
| App and pages out of sync | possible (plus a 24 h cache) | impossible (one image, ETag = version) |

**Takeaway:** for a small web UI, embed it gzipped in the app image. One file, one version, and
the update page can never be deleted by an update.

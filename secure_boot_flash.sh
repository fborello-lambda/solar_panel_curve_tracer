#!/bin/bash
# Builds the secure-boot variant and flashes it to the maintainer's ONE
# fused board. Do not run this against a standard (non-secure) board: once
# a secure-boot bootloader is flashed and the board's eFuses are burned, it
# will only ever accept images signed with secure_boot_signing_key.pem.
set -e

PORT=${PORT:-/dev/ttyACM0}
BUILD_DIR=build-secure

echo "=== Building secure-boot variant into ${BUILD_DIR} ==="
idf.py -B "$BUILD_DIR" \
    -D "SDKCONFIG=${BUILD_DIR}/sdkconfig" \
    -D "SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.secure" \
    build

# Refuse to flash a build that isn't actually secure-boot signed. This
# catches the case where a stale build-secure/sdkconfig was reused without
# the secure overlay (e.g. after switching branches).
if ! grep -q '^CONFIG_SECURE_BOOT=y' "${BUILD_DIR}/sdkconfig"; then
    echo "ERROR: ${BUILD_DIR}/sdkconfig does not have CONFIG_SECURE_BOOT=y." >&2
    echo "Refusing to flash - this would not produce a signed image." >&2
    exit 1
fi

# Derive flash offsets from the build's own flash_args instead of
# hardcoding them, so this script stays correct if partitions.csv or the
# bootloader offset ever changes.
FLASH_ARGS_FILE="${BUILD_DIR}/flash_args"
if [ ! -f "$FLASH_ARGS_FILE" ]; then
    echo "ERROR: ${FLASH_ARGS_FILE} not found." >&2
    exit 1
fi

# flash_args looks like:
#   --flash_mode dio --flash_freq 80m --flash_size 4MB
#   0xd000 partition_table/partition-table.bin
#   0x0 bootloader/bootloader.bin
#   0x20000 web_server.bin
#   0x12000 ota_data_initial.bin
FLASH_OPTS=$(head -n1 "$FLASH_ARGS_FILE")
APP_LINE=$(grep 'web_server.bin' "$FLASH_ARGS_FILE")
BOOTLOADER_LINE=$(grep 'bootloader.bin' "$FLASH_ARGS_FILE")
PARTTABLE_LINE=$(grep 'partition-table.bin' "$FLASH_ARGS_FILE")
OTADATA_LINE=$(grep 'ota_data_initial.bin' "$FLASH_ARGS_FILE")

echo "=== Flashing bootloader, app, partition table, ota_data ==="
# shellcheck disable=SC2086
esptool.py -p "$PORT" -b 460800 \
    --before default_reset --after hard_reset \
    --chip esp32c3 --no-stub \
    write_flash $FLASH_OPTS --flash_size keep \
    $(awk -v d="$BUILD_DIR" '{print $1, d"/"$2}' <<< "$BOOTLOADER_LINE") \
    $(awk -v d="$BUILD_DIR" '{print $1, d"/"$2}' <<< "$APP_LINE") \
    $(awk -v d="$BUILD_DIR" '{print $1, d"/"$2}' <<< "$PARTTABLE_LINE") \
    $(awk -v d="$BUILD_DIR" '{print $1, d"/"$2}' <<< "$OTADATA_LINE")

echo "=== Done ==="

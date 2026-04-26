#!/bin/bash
set -e

PORT=${PORT:-/dev/ttyACM0}

echo "=== Building ==="
idf.py build

echo "=== Flashing bootloader, app, partition table, ota_data ==="
esptool.py -p $PORT -b 460800 \
    --before default_reset --after no_reset \
    --chip esp32c3 --no-stub \
    write_flash --flash_mode dio --flash_freq 80m --flash_size keep \
    0x0     build/bootloader/bootloader.bin \
    0x20000 build/web_server.bin \
    0xd000  build/partition_table/partition-table.bin \
    0x12000 build/ota_data_initial.bin

echo "=== Flashing storage (SPIFFS) ==="
esptool.py -p $PORT -b 460800 \
    --before no_reset --after hard_reset \
    --chip esp32c3 --no-stub \
    write_flash --flash_mode dio --flash_freq 80m --flash_size 4MB \
    0x240000 build/storage.bin

echo "=== Done ==="

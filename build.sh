#!/usr/bin/env bash
# Build MicroPython firmware for ESP32-S3 with CSI module
# Usage: ./build.sh [PORT]   e.g.  ./build.sh /dev/ttyUSB0
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
IDF_PATH="$(cd "$SCRIPT_DIR/../esp-idf" && pwd)"
MP_PORT="$(cd "$SCRIPT_DIR/../micropython/ports/esp32" && pwd)"
BOARD="ESP32_GENERIC_S3"
BUILD_DIR="$MP_PORT/build-$BOARD-CSI"

echo "=== Environment setup ==="
source "$IDF_PATH/export.sh"

echo "=== Building MicroPython with CSI module ==="
cmake -S "$MP_PORT" -B "$BUILD_DIR" \
    -DMICROPY_BOARD="$BOARD" \
    -DUSER_C_MODULES="$SCRIPT_DIR/micropython.cmake" \
    -DCMAKE_BUILD_TYPE=Release

cmake --build "$BUILD_DIR" -- -j"$(nproc)"

echo ""
echo "=== Build complete ==="
echo "Firmware:   $BUILD_DIR/micropython.bin"
echo "Bootloader: $BUILD_DIR/bootloader/bootloader.bin"
echo "Partitions: $BUILD_DIR/partition_table/partition-table.bin"

# Flash if port provided
if [ -n "$1" ]; then
    echo "=== Flashing to $1 ==="
    esptool.py \
        --chip esp32s3 \
        --port "$1" \
        --baud 921600 \
        write_flash \
        --flash_mode dio \
        --flash_freq 80m \
        0x0     "$BUILD_DIR/bootloader/bootloader.bin" \
        0x8000  "$BUILD_DIR/partition_table/partition-table.bin" \
        0x10000 "$BUILD_DIR/micropython.bin"
    echo "Done. Connect with: screen $1 115200"
fi

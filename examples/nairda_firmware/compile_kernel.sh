#!/usr/bin/env bash
set -euo pipefail

# ── Compile the Nairda ESP32 kernel ──────────────────────────────────────────
#
# Reproduces the exact build that produces a kernel binary compatible with
# the permanent-programming partition layout (partitions.csv next to this
# script): app0 = 0x1EE000 (1.93 MB), userapp = 0x2000 (last 8 KB).
#
# The two custom build properties below are NOT defaults of the board:
#   - build.partitions=partitions       → forces the local partitions.csv
#   - upload.maximum_size=2027520       → reflects the new app0 size (0x1EE000)
#
# Without these, arduino-cli falls back to the board's default partition
# (1.25 MB app, no userapp) and the kernel boots without the userapp slot,
# making permanent programming silently non-functional.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LIB_PARENT="$(cd "$SCRIPT_DIR/../../.." && pwd)"   # parent of NairdaArduinoLibrary

cd "$SCRIPT_DIR"

echo "▸ Compiling Nairda kernel for ESP32 (DOIT DevKit V1)..."

arduino-cli compile \
  --fqbn esp32:esp32:esp32doit-devkit-v1 \
  --build-property "build.partitions=partitions" \
  --build-property "upload.maximum_size=2027520" \
  --libraries "$LIB_PARENT" \
  --export-binaries \
  nairda_firmware.ino

echo ""
echo "✔ Build complete. Artifacts under build/esp32.esp32.esp32doit-devkit-v1/"
echo "  Flash with:"
echo "    arduino-cli upload --fqbn esp32:esp32:esp32doit-devkit-v1 \\"
echo "      --port /dev/cu.usbserial-XXXX \\"
echo "      $(basename "$SCRIPT_DIR")/nairda_firmware.ino"

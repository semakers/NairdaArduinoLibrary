#!/bin/bash
# Compila user_app.c a binario crudo para ESP32
TOOLCHAIN=~/Library/Arduino15/packages/esp32/tools/esp-x32/2511/bin

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

$TOOLCHAIN/xtensa-esp32-elf-gcc \
  -Os \
  -nostartfiles \
  -nostdlib \
  -fno-builtin \
  -mtext-section-literals \
  -I"$SCRIPT_DIR" \
  -Wl,--entry=_start \
  -o user_app.elf \
  user_app.c

echo "=== ELF sections ==="
$TOOLCHAIN/xtensa-esp32-elf-objdump -h user_app.elf

echo ""
echo "=== Disassembly ==="
$TOOLCHAIN/xtensa-esp32-elf-objdump -d user_app.elf

# Extraer entry point offset
ENTRY=$($TOOLCHAIN/xtensa-esp32-elf-readelf -h user_app.elf | grep "Entry point" | awk '{print $4}')
TEXT_START=$($TOOLCHAIN/xtensa-esp32-elf-objdump -h user_app.elf | grep "\.text" | head -1 | awk '{print "0x"$4}')
ENTRY_OFFSET=$(printf "%d" $(( $ENTRY - $TEXT_START )))
echo ""
echo "=== Entry point ==="
echo "Entry: $ENTRY, .text start: $TEXT_START, offset: $ENTRY_OFFSET bytes"

$TOOLCHAIN/xtensa-esp32-elf-objcopy -O binary user_app.elf user_app.bin

echo ""
echo "=== Binary size ==="
ls -la user_app.bin
echo ""
echo "=== Binary hex dump ==="
xxd user_app.bin

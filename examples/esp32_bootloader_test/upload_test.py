#!/usr/bin/env python3
# Upload test - Protocolo chunked compatible AVR
# Uso: python3 upload_test.py <puerto> <binario.bin> [baudrate]

import serial
import sys
import time
import os

CHUNK_SIZE = 64

def get_entry_offset(elf_path):
    """Lee el entry offset del ELF (distancia de _start al inicio de .text)"""
    import subprocess
    toolchain = os.path.expanduser("~/Library/Arduino15/packages/esp32/tools/esp-x32/2511/bin")

    # Entry point
    out = subprocess.check_output([f"{toolchain}/xtensa-esp32-elf-readelf", "-h", elf_path], text=True)
    entry = int([l for l in out.split('\n') if 'Entry point' in l][0].split()[-1], 16)

    # .text start
    out = subprocess.check_output([f"{toolchain}/xtensa-esp32-elf-objdump", "-h", elf_path], text=True)
    for l in out.split('\n'):
        if '.text' in l and 'CONTENTS' not in l:
            text_start = int(l.split()[3], 16)
            break

    return entry - text_start

def upload(port, bin_path, baud=9600):
    with open(bin_path, 'rb') as f:
        raw = f.read()

    # Calcular entry offset desde el ELF
    elf_path = bin_path.replace('.bin', '.elf')
    entry_offset = get_entry_offset(elf_path) if os.path.exists(elf_path) else 0

    # Prepend [0x00, entry_offset] (flag + entry_offset)
    firmware = bytes([0x00, entry_offset]) + raw

    print(f"Binario: {bin_path} ({len(raw)} bytes code + 2 header = {len(firmware)} total)")
    print(f"Entry offset: {entry_offset} bytes")
    print(f"Puerto: {port} @ {baud}")

    total_chunks = (len(firmware) + CHUNK_SIZE - 1) // CHUNK_SIZE
    print(f"Chunks: {total_chunks} x {CHUNK_SIZE} bytes max")

    ser = serial.Serial(port, baud, timeout=2)
    time.sleep(2)
    ser.reset_input_buffer()

    # Leer boot message
    boot = ser.read(ser.in_waiting or 500)
    if boot:
        print(boot.decode('utf-8', errors='replace'))

    # Enviar comando 150 (bootloader)
    print(">> Enviando comando 150...")
    ser.write(bytes([150]))
    time.sleep(0.3)

    r = ser.read(ser.in_waiting or 500)
    if r:
        print(r.decode('utf-8', errors='replace'))

    # Enviar chunks: [len][checksum][data]
    for i in range(total_chunks):
        offset = i * CHUNK_SIZE
        chunk = firmware[offset:offset + CHUNK_SIZE]
        length = len(chunk)

        checksum = 0
        for b in chunk:
            checksum = (checksum + b) % 64

        frame = bytes([length, checksum]) + chunk
        ser.write(frame)
        time.sleep(0.1)

        # Leer respuesta del chunk
        r = ser.read(ser.in_waiting or 200)
        if r:
            print(r.decode('utf-8', errors='replace'), end='')

        print(f"  Chunk {i+1}/{total_chunks}: {length} bytes, checksum={checksum}")

    # Terminador [0][0]
    print(">> Enviando terminador [0][0]...")
    ser.write(bytes([0x00, 0x00]))
    time.sleep(0.3)

    # Leer respuesta final
    time.sleep(3)
    response = ser.read(ser.in_waiting or 4000)
    if response:
        print(response.decode('utf-8', errors='replace'))

    time.sleep(2)
    remaining = ser.read(ser.in_waiting or 1000)
    if remaining:
        print(remaining.decode('utf-8', errors='replace'))

    ser.close()
    print(">> Done")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(f"Uso: {sys.argv[0]} <puerto> <binario.bin> [baudrate]")
        sys.exit(1)

    port = sys.argv[1]
    bin_path = sys.argv[2]
    baud = int(sys.argv[3]) if len(sys.argv) > 3 else 9600

    upload(port, bin_path, baud)

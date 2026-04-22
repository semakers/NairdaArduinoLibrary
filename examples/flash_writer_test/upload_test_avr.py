#!/usr/bin/env python3
"""
Upload test AVR — Protocolo chunked idéntico a Flutter (firmware_upload_bloc.dart)

Envía un binario de usuario al kernel Nairda en ATmega328P vía Serial.
Protocolo:
  1. Enviar cmd 100 (jmp 0 reset)
  2. Esperar ventana de boot (2s)
  3. Enviar cmd 150 (enterBootloaderMode)
  4. Enviar header [0x00, 0xFF] + binario en chunks de 64:
     [len][checksum % 64][datos...]
  5. Terminador [0x00, 0x00]

Uso: python3 upload_test_avr.py <puerto> <binario.bin> [baudrate]
"""

import serial
import sys
import time

CHUNK_SIZE = 64


def read_all(ser, label="KERNEL"):
    """Lee todo lo disponible en el buffer serial y lo imprime."""
    data = ser.read(ser.in_waiting or 1)
    if data:
        print(f"[{label}] {data.decode('utf-8', errors='replace')}", end='')
    return data


def upload(port, bin_path, baud=9600):
    with open(bin_path, 'rb') as f:
        raw = f.read()

    # Header idéntico a Flutter: [0x00 (flag invalido), 0xFF (padding AVR)]
    firmware = bytes([0x00, 0xFF]) + raw

    print(f"Binario: {bin_path} ({len(raw)} bytes code + 2 header = {len(firmware)} total)")
    print(f"Puerto: {port} @ {baud}")

    total_chunks = (len(firmware) + CHUNK_SIZE - 1) // CHUNK_SIZE
    print(f"Chunks: {total_chunks} x {CHUNK_SIZE} bytes max")
    print()

    # Abrir serial — DTR resetea el Arduino automáticamente
    print(">> Abriendo serial (DTR reset)...")
    ser = serial.Serial(port, baud, timeout=2)

    # Esperar que el kernel arranque e imprima NK:1..NK:BOOT
    # El kernel tarda ~200ms en llegar a la ventana de boot
    time.sleep(1.0)
    read_all(ser, "BOOT")

    # Enviar cmd 150 DENTRO de la ventana de boot (2 segundos desde reset)
    print("\n>> Enviando cmd 150 (bootloader mode)...")
    ser.write(bytes([150]))
    ser.flush()
    time.sleep(0.3)
    read_all(ser)

    # Enviar chunks [len][checksum][datos...]
    print(f"\n>> Enviando {total_chunks} chunks...")
    for i in range(total_chunks):
        offset = i * CHUNK_SIZE
        chunk = firmware[offset:offset + CHUNK_SIZE]
        length = len(chunk)

        checksum = sum(chunk) % 64

        frame = bytes([length, checksum]) + chunk
        ser.write(frame)
        ser.flush()
        time.sleep(0.15)  # Dar tiempo al kernel para procesar

        r = ser.read(ser.in_waiting or 1)
        if r:
            print(f"  [KERNEL] {r.decode('utf-8', errors='replace')}", end='')

        pct = int((i + 1) / total_chunks * 100)
        print(f"  Chunk {i+1}/{total_chunks}: {length} bytes, chk={checksum} [{pct}%]")

    # Terminador [0][0]
    print(">> Enviando terminador [0x00, 0x00]...")
    ser.write(bytes([0x00, 0x00]))
    ser.flush()
    time.sleep(1.0)

    # El kernel escribe la primera página con flag=0x01, luego hace jmp 0
    # Debería imprimir NK:1..NK:BOOT..NK:FLAG 1..NK:EXEC
    print(">> Esperando reboot y ejecución del user program...")
    time.sleep(4)
    read_all(ser, "REBOOT")
    print()

    time.sleep(2)
    read_all(ser, "FINAL")
    print()

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

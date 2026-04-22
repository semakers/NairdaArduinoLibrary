#!/usr/bin/env python3
"""
test_nairda_kernel.py — Simulates the Flutter upload flow for the Nairda kernel.

Compiles a user blink program via Cloud Run, then uploads it to the Arduino
using the BootJacker protocol (cmd 100 → 150 → J/E/W for JT → F/L/E/W per page → R).

Usage:
    python3 test_nairda_kernel.py <port> [--run]
"""

import serial
import sys
import time
import urllib.request
import json

# ── Cloud Run compiler ──────────────────────────────────────────────
COMPILE_URL = 'https://nairda-user-compiler-mpjkn2bhoq-uc.a.run.app/compile?target=avr'

# ── Protocol constants ──────────────────────────────────────────────
PAGE_SIZE = 128
CHUNK_SIZE = 64
JT_ADDR = 0x5F80
USER_SPACE = 0x6000
WDT_DELAY = 0.2   # 200ms after each E/W (WDT reset + boot)
CHUNK_DELAY = 0.05  # 50ms between chunks

# ── Blink program using nairda_user.h syscalls ──────────────────────
BLINK_CODE = r"""
#include "nairda_user.h"

uint8_t led[NAIRDA_COMP_SIZE] = {0};

void __attribute__((section(".text"))) _start(void) {
  nairda_setupDigitalOut(led, 13);

  while ((1 == 1)) {
    nairda_runDigitalOut(led, (int)(100));
    nairda_delay((unsigned long)(500));
    nairda_runDigitalOut(led, (int)(0));
    nairda_delay((unsigned long)(500));
  }

  while (1) { nairda_delay(100); }
}
"""


def compile_user_program(code: str) -> bytes:
    """Compile C code via Cloud Run, returns raw binary."""
    print("Compiling via Cloud Run...")
    req = urllib.request.Request(
        COMPILE_URL,
        data=code.encode('utf-8'),
        headers={'Content-Type': 'text/plain'},
    )
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            binary = resp.read()
            print(f"  Compiled: {len(binary)} bytes")
            return binary
    except urllib.error.HTTPError as e:
        print(f"  Compile error ({e.code}): {e.read().decode()}")
        sys.exit(1)


def send_byte(ser, b):
    ser.write(bytes([b]))


def send_bytes(ser, data):
    ser.write(bytes(data))


def wait_boot(ser, timeout=5):
    """Wait for kernel to boot (drain serial output)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if line:
                print(f"  << {line}")
        else:
            time.sleep(0.02)


def upload(port: str, binary: bytes, auto_run: bool = False):
    # Prepend header: [flag=0x01, padding=0xFF]
    data = bytes([0x01, 0xFF]) + binary
    total_pages = (len(data) + PAGE_SIZE - 1) // PAGE_SIZE

    print(f"\nBinary: {len(binary)} bytes + 2-byte header = {len(data)} bytes")
    print(f"Pages:  {total_pages}")
    print()

    ser = serial.Serial(port, 9600, timeout=1)
    time.sleep(2)

    # Drain boot messages
    print("Waiting for kernel boot...")
    wait_boot(ser, timeout=3)

    # ── cmd 100: reset ──────────────────────────────────────────────
    print("Sending cmd 100 (reset)...")
    send_byte(ser, 100)
    time.sleep(0.5)
    wait_boot(ser, timeout=3)

    # ── cmd 150: enter bootloader ───────────────────────────────────
    print("Sending cmd 150 (bootloader mode)...")
    send_byte(ser, 150)
    time.sleep(0.3)

    # ── Jump Table: J → E → J → W ──────────────────────────────────
    print("Writing jump table at 0x5F80...")

    # J: fill page_buf with JT data
    send_byte(ser, ord('J'))
    time.sleep(CHUNK_DELAY)

    # E: erase JT page
    jt_lo = JT_ADDR & 0xFF
    jt_hi = (JT_ADDR >> 8) & 0xFF
    send_bytes(ser, [ord('E'), jt_lo, jt_hi])
    print(f"  E:0x{JT_ADDR:04X} → WDT")
    time.sleep(WDT_DELAY)
    wait_boot(ser, timeout=2)

    # J again: refill after WDT
    send_byte(ser, ord('J'))
    time.sleep(CHUNK_DELAY)

    # W: write JT page
    send_bytes(ser, [ord('W'), jt_lo, jt_hi])
    print(f"  W:0x{JT_ADDR:04X} → WDT")
    time.sleep(WDT_DELAY)
    wait_boot(ser, timeout=2)

    print("  JT done")

    # ── User program pages ──────────────────────────────────────────
    for page_idx in range(total_pages):
        page_offset = page_idx * PAGE_SIZE
        page_addr = USER_SPACE + page_offset
        addr_lo = page_addr & 0xFF
        addr_hi = (page_addr >> 8) & 0xFF

        # Get page data (pad last page)
        page_data = data[page_offset:page_offset + PAGE_SIZE]
        if len(page_data) < PAGE_SIZE:
            page_data = page_data + bytes([0xFF] * (PAGE_SIZE - len(page_data)))

        print(f"Page {page_idx}: 0x{page_addr:04X}")

        # F: clear buffer
        send_byte(ser, ord('F'))
        time.sleep(CHUNK_DELAY)

        # L: load chunks
        for off in range(0, PAGE_SIZE, CHUNK_SIZE):
            chunk_len = min(CHUNK_SIZE, PAGE_SIZE - off)
            chunk = page_data[off:off + chunk_len]
            send_bytes(ser, [ord('L'), off, chunk_len] + list(chunk))
            time.sleep(CHUNK_DELAY)

        # E: erase → WDT
        send_bytes(ser, [ord('E'), addr_lo, addr_hi])
        print(f"  E → WDT")
        time.sleep(WDT_DELAY)
        wait_boot(ser, timeout=2)

        # W: write → WDT
        send_bytes(ser, [ord('W'), addr_lo, addr_hi])
        print(f"  W → WDT")
        time.sleep(WDT_DELAY)
        wait_boot(ser, timeout=2)

    print()

    if auto_run:
        print("Sending 'R' (run)...")
        send_byte(ser, ord('R'))
        time.sleep(0.5)
        wait_boot(ser, timeout=2)

    print("\nDone!")
    ser.close()


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <port> [--run]")
        sys.exit(1)

    port = sys.argv[1]
    auto_run = '--run' in sys.argv

    # Compile
    binary = compile_user_program(BLINK_CODE)

    # Upload
    upload(port, binary, auto_run)


if __name__ == '__main__':
    main()

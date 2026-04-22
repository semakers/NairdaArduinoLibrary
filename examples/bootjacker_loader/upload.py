#!/usr/bin/env python3
"""
upload.py — Send a compiled user binary to bootjacker_loader via Serial.

Usage:
    python3 upload.py <port> <binary.bin> [--run]

Protocol (script-driven, no ACK — delay-based):
    Per page:
      1. 'F'              → reset page buffer to 0xFF
      2. 'L' off len data → load chunk at offset
      3. 'E' lo hi        → erase page (WDT reset, wait 300ms)
      4. 'W' lo hi        → write page (WDT reset, wait 300ms)
    After all pages:
      5. 'R'              → run user program
"""

import serial
import sys
import time

CHUNK_SIZE = 64
PAGE_SIZE = 128
USER_SPACE = 0x6000
WDT_DELAY = 0.4  # seconds to wait after WDT reset


def wait_ready(ser, timeout=10):
    """Wait for BJ:READY after boot/WDT reset."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if line:
                print(f"  << {line}")
            if 'BJ:READY' in line:
                return True
        time.sleep(0.05)
    return False


def send_page(ser, page_data, page_addr):
    """Send one page: fill buffer, erase, write."""
    # Step 1: Reset buffer
    ser.write(b'F')
    time.sleep(0.1)
    while ser.in_waiting:
        ser.readline()  # consume F:OK

    # Step 2: Load chunks into buffer
    offset = 0
    while offset < len(page_data):
        chunk_len = min(CHUNK_SIZE, len(page_data) - offset)
        chunk = page_data[offset:offset + chunk_len]
        ser.write(bytes([ord('L'), offset, chunk_len]) + chunk)
        time.sleep(0.05)
        while ser.in_waiting:
            ser.readline()  # consume L:OK
        offset += chunk_len

    # Step 3: Erase
    lo = page_addr & 0xFF
    hi = (page_addr >> 8) & 0xFF
    ser.write(bytes([ord('E'), lo, hi]))
    time.sleep(WDT_DELAY)
    if not wait_ready(ser, timeout=5):
        print("  !! Timeout after erase WDT")
        return False

    # Step 4: Write
    ser.write(bytes([ord('W'), lo, hi]))
    time.sleep(WDT_DELAY)
    if not wait_ready(ser, timeout=5):
        print("  !! Timeout after write WDT")
        return False

    return True


def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <port> <binary.bin> [--run]")
        sys.exit(1)

    port = sys.argv[1]
    binary_path = sys.argv[2]
    auto_run = '--run' in sys.argv

    with open(binary_path, 'rb') as f:
        raw = f.read()

    # Prepend header: [flag=0x01, padding=0xFF]
    data = bytes([0x01, 0xFF]) + raw
    total_pages = (len(data) + PAGE_SIZE - 1) // PAGE_SIZE

    print(f"Binary: {len(raw)} bytes + 2-byte header = {len(data)} bytes")
    print(f"Pages:  {total_pages} ({PAGE_SIZE} bytes each)")
    print()

    ser = serial.Serial(port, 115200, timeout=2)
    time.sleep(2)

    print("Waiting for BJ:READY...")
    if not wait_ready(ser):
        print("ERROR: Timeout")
        ser.close()
        return

    # Upload page by page
    for page_idx in range(total_pages):
        offset = page_idx * PAGE_SIZE
        page_data = data[offset:offset + PAGE_SIZE]
        # Pad last page to full size
        if len(page_data) < PAGE_SIZE:
            page_data = page_data + bytes([0xFF] * (PAGE_SIZE - len(page_data)))

        page_addr = USER_SPACE + page_idx * PAGE_SIZE
        print(f"Page {page_idx}: addr=0x{page_addr:04X} ({len(data) - offset} bytes remaining)")

        if not send_page(ser, page_data, page_addr):
            print("ERROR: Page write failed")
            ser.close()
            return

    print()

    if auto_run:
        print("Sending 'R' (run)...")
        ser.write(b'R')
        time.sleep(0.5)
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if line:
                print(f"  << {line}")
    else:
        # Verify
        ser.reset_input_buffer()
        ser.write(b'?')
        time.sleep(0.5)
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if line:
                print(f"  << {line}")

    print("\nDone!")
    ser.close()


if __name__ == '__main__':
    main()

"""
pi5_pico_link_test.py — Verify Pi 5 <-> Pico CDC link and event logging

What it does:
- Opens the Pico CDC device (/dev/ttyACM*)
- Sends a SYNC marker command periodically
- Reads 12-byte event records (t_us, code, aux, seq) and writes them to a log
- Prints a short heartbeat so you can see it working

Run:
  python3 pi5_pico_link_test.py
"""

import glob
import os
import struct
import sys
import time
from datetime import datetime

try:
    import serial  # pyserial
except Exception as e:
    print("pyserial not installed. Install with: pip install pyserial", file=sys.stderr)
    raise


def find_pico_port():
    # Prefer ACM devices; fallback to USB serial
    candidates = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    return candidates[0] if candidates else None


def open_serial(port: str):
    # For CDC, baud is typically ignored but some stacks require a value
    return serial.Serial(port, 115200, timeout=0.02)


def send_cmd_sync_marker(ser, marker: int = 1):
    # Frame: b'CMD0' | op | ver | len(2 LE) | payload
    payload = struct.pack('<H', marker)
    frame = b'CMD0' + bytes([0x06, 0]) + struct.pack('<H', len(payload)) + payload
    ser.write(frame)


def main():
    port = find_pico_port()
    if not port:
        print("No Pico CDC device found (/dev/ttyACM*). Plug the Pico and try again.")
        sys.exit(1)

    print(f"Opening {port} ...")
    ser = open_serial(port)

    # Binary event record
    EV_REC = struct.Struct('<I H h I')

    # Log file in current directory
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_path = os.path.abspath(f'linktest_{ts}.bin')
    print(f"Logging events to {log_path}")
    out = open(log_path, 'ab')

    buf = bytearray()
    last_beat = time.time()
    marker = 1
    try:
        while True:
            # Read bytes from Pico
            chunk = ser.read(4096)
            if chunk:
                buf += chunk
                while len(buf) >= 12:
                    rec = bytes(buf[:12]); del buf[:12]
                    out.write(rec)
            # Send a SYNC marker periodically to test Pi->Pico->Pi loop
            now = time.time()
            if now - last_beat >= 1.0:
                last_beat = now
                send_cmd_sync_marker(ser, marker)
                marker = (marker + 1) & 0xFFFF
                out.flush()
                print("heartbeat: wrote SYNC cmd; bytes buffered:", len(buf))
    except KeyboardInterrupt:
        pass
    finally:
        out.flush(); out.close()
        ser.close()
        print("Closed.")


if __name__ == '__main__':
    main()



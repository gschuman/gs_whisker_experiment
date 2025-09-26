# 01_sync_test.py — emits a 1 ms TTL pulse on GP13 once per second and prints the
# microsecond timestamp at the start of each pulse ("SYNC <t_us>"). Use this to
# verify sync timing with a scope/logic analyzer or to drive external systems.
# Wiring: GP13 = sync out (3.3 V logic), connect grounds in common.
# Notes: Busy-wait is used only for the short pulse width; the 1 s period uses sleep.

from machine import Pin
import time

# Sync TTL output pin (wire this to your external system; 3.3 V logic)
SYNC_PIN = 13
PULSE_WIDTH_US = 1000     # 1 ms high pulse
PERIOD_MS = 1000          # pulse once per second

sync = Pin(SYNC_PIN, Pin.OUT, value=0)

try:
    while True:
        t0 = time.ticks_us()
        sync.value(1)
        # Short, precise busy-wait for pulse width
        t_start = time.ticks_us()
        while time.ticks_diff(time.ticks_us(), t_start) < PULSE_WIDTH_US:
            pass
        sync.value(0)
        # Print timestamp (microseconds since boot) for verification
        print("SYNC", t0)
        time.sleep_ms(PERIOD_MS)
except KeyboardInterrupt:
    sync.value(0)

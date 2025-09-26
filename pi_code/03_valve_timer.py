# 03_valve_timer.py — MOSFET valve test (7 s ON, 7 s OFF, repeat)
# Wiring (low-side switch, 12 V solenoid):
#   - Pico GP15 -> 100 Ω -> MOSFET G (gate)
#   - 100 kΩ from MOSFET G to GND (gate pulldown)
#   - MOSFET S (source) -> GND (logic + 12 V supply ground common with Pico)
#   - Valve - lead -> MOSFET D (drain)
#   - Valve + lead -> +12 V
#   - Flyback diode across valve: diode cathode to +12 V, anode to valve - (across coil)
# Notes:
#   - Do NOT power the valve from the Pico. Use an external 12 V supply and common ground.
#   - This test uses a hardware one-shot timer to turn the valve off precisely after N ms.

from machine import Pin, Timer
import time

VALVE_PIN = 15           # GP15: MOSFET gate drive
GATE_ACTIVE_HIGH = 1     # 1 for N-MOSFET low-side (typical)
PULSE_MS = 7000          # ON duration (ms)
PAUSE_MS = 7000          # OFF pause before next pulse (ms)

valve = Pin(VALVE_PIN, Pin.OUT, value=0 if GATE_ACTIVE_HIGH else 1)
_timer = Timer()


def _valve_off_cb(_t):
    # Drive gate to OFF level
    valve.value(0 if GATE_ACTIVE_HIGH else 1)
    print("VALVE OFF", time.ticks_ms(), "ms")


def valve_pulse_ms(ms: int) -> None:
    # Turn valve ON and arm one-shot to turn it OFF
    valve.value(1 if GATE_ACTIVE_HIGH else 0)
    print("VALVE ON ", time.ticks_ms(), "ms for", ms, "ms")
    _timer.init(mode=Timer.ONE_SHOT, period=ms, callback=_valve_off_cb)


if __name__ == "__main__":
    print("Valve timer test: ON {} ms, OFF {} ms on GP{}".format(PULSE_MS, PAUSE_MS, VALVE_PIN))
    while True:
        valve_pulse_ms(PULSE_MS)
        # Sleep until after the off period completes (on-time + off-time)
        time.sleep_ms(PULSE_MS + PAUSE_MS)

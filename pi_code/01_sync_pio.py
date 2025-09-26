# 01_sync_pio.py — PIO-based continuous sync square wave on GP13 (3.3 V)
# Configure F_HZ and DUTY to get a low-jitter clock; runs independently of Python.
# Wiring: GP13 -> external system TTL input; share ground.

from machine import Pin
import time
from rp2 import PIO, StateMachine, asm_pio

# ---- Configuration ----
SYNC_PIN = 13       # output pin for sync TTL
F_HZ = 100          # square wave frequency (Hz)
DUTY = 0.50         # duty cycle (0.0..1.0)
SM_ID = 0           # PIO state machine ID to use
SM_CLK_HZ = 125_000_000  # PIO clock; default matches system clock on Pico


@asm_pio(set_init=PIO.OUT_LOW)
def pio_square():
    # Expects two pulls before start: high_ticks then low_ticks (both > 0)
    pull(block)
    mov(x, osr)        # X = high_ticks
    pull(block)
    mov(y, osr)        # Y = low_ticks
    label("loop")
    set(pins, 1)
    label("h1")
    jmp(x_dec, "h1")  # wait high_ticks cycles
    set(pins, 0)
    label("l1")
    jmp(y_dec, "l1")  # wait low_ticks cycles
    jmp("loop")


# Initialize state machine at a known clock
sm = StateMachine(SM_ID, pio_square, freq=SM_CLK_HZ, set_base=Pin(SYNC_PIN))

# Compute ticks for requested frequency/duty
# One PIO instruction issues per system clock at sm.freq; each loop iteration spends
# ~ (high_ticks + low_ticks + small overhead). We model just the wait loops for stability.
period_ticks = max(2, int(sm.freq // F_HZ))
high_ticks = max(1, int(period_ticks * max(0.0, min(1.0, DUTY))))
low_ticks = max(1, period_ticks - high_ticks)

# Program the PIO and start
sm.put(high_ticks)
sm.put(low_ticks)
sm.active(1)

# Optional: report once, then idle while PIO runs independently
actual_period_s = (high_ticks + low_ticks) / sm.freq
actual_f_hz = 1.0 / actual_period_s
print("SYNC_PIO on GP{}: target={} Hz duty={:.0f}% -> actual={:.3f} Hz".format(
    SYNC_PIN, F_HZ, DUTY * 100.0, actual_f_hz
))

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    sm.active(0)
    Pin(SYNC_PIN, Pin.OUT).value(0)

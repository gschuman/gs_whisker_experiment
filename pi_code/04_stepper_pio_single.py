# 04_stepper_pio_single.py — PIO STEP generator for one stepper driver (NEMA17)
# Driver inputs (single-ended): PUL+ (STEP), DIR+, ENA+, GND
# Wiring (3.3 V logic; share GND):
#   - Pico GP16 -> PUL+   (STEP)
#   - Pico GP17 -> DIR+
#   - Pico GP20 -> ENA+   (optional; or leave unconnected and set USE_ENA=False)
#   - Pico GND  -> Driver GND
# Notes:
#   - Many drivers accept 3.3 V HIGH. If yours needs 5 V logic, add a 74AHCT125
#     (3.3 V in → 5 V out) or an NPN/open-collector interface with a pull-up to 5 V.
#   - Motor power (VMOT) stays on the driver’s supply; only logic lines connect to the Pico.
#   - Set your driver’s microstep DIP switches to 1600 steps/rev (1/8) for this test.

from machine import Pin
import time
from rp2 import PIO, StateMachine, asm_pio

# ---------------- Configuration ----------------
STEP_PIN = 16
DIR_PIN  = 17
ENA_PIN  = 20
USE_ENA = True
ENA_ACTIVE_HIGH = True   # set False if your driver enables on LOW

SM_ID = 0
SM_CLK_HZ = 125_000_000

# Steps-per-revolution on the DRIVER (set by its DIP switches)
STEPS_PER_REV = 1600      # 1600 µsteps = 1 full revolution in this test

# Demo params
STEP_HZ       = 2000      # step frequency (Hz) — safe starting point
MOVE_STEPS    = STEPS_PER_REV  # one full turn

# ---------------- PIO: counted STEP pulses ----------------
@asm_pio(set_init=PIO.OUT_LOW)
def pio_step():
    # pull -> half_period_ticks; pull -> step_count
    pull(block)
    mov(x, osr)          # X = half period
    pull(block)
    mov(y, osr)          # Y = steps remaining
    label("loop")
    set(pins, 1)
    mov(isr, x)
    label("h1")
    jmp(x_dec, "h1")
    set(pins, 0)
    mov(isr, x)
    label("h2")
    jmp(x_dec, "h2")
    jmp(y_dec, "loop")
    irq(0)

sm = StateMachine(SM_ID, pio_step, freq=SM_CLK_HZ, set_base=Pin(STEP_PIN))

# ---------------- GPIO helpers ----------------
dir_pin = Pin(DIR_PIN, Pin.OUT, value=0)
if USE_ENA:
    ena_pin = Pin(ENA_PIN, Pin.OUT, value=1 if ENA_ACTIVE_HIGH else 0)
else:
    ena_pin = None


def enable_driver(enable: bool) -> None:
    if not USE_ENA:
        return
    ena_pin.value(1 if (enable == ENA_ACTIVE_HIGH) else 0)


def step_move(steps: int, step_hz: int, forward: bool) -> None:
    # Set direction
    dir_pin.value(1 if forward else 0)
    # Compute half period ticks at PIO clock
    if step_hz <= 0:
        return
    half = max(1, int(SM_CLK_HZ // (2 * step_hz)))
    # Load params and run
    sm.active(0)
    sm.put(half)
    sm.put(int(steps))
    sm.active(1)
    # Blocking wait based on duration (adequate for a bench test)
    est_s = steps / max(1, step_hz)
    time.sleep(est_s + 0.02)


if __name__ == "__main__":
    print("Stepper PIO test")
    print("Pins: STEP=GP{}, DIR=GP{}{}; Pico GND -> driver GND".format(
        STEP_PIN, DIR_PIN, ", ENA=GP{}".format(ENA_PIN) if USE_ENA else ""
    ))
    print("Driver microstep setting: {} steps/rev (configure on driver DIP)".format(STEPS_PER_REV))
    print("Sequence: +360° then -360° at {} Hz".format(STEP_HZ))
    try:
        enable_driver(True)
        # +360° (clockwise)
        print("CW: +{} steps".format(MOVE_STEPS))
        step_move(MOVE_STEPS, STEP_HZ, True)
        time.sleep(0.5)
        # -360° (counter-clockwise)
        print("CCW: -{} steps".format(MOVE_STEPS))
        step_move(MOVE_STEPS, STEP_HZ, False)
        time.sleep(0.5)
    finally:
        enable_driver(False)
        sm.active(0)
        Pin(STEP_PIN, Pin.OUT).value(0)

# 02_lick_irq.py — Lick detection via GPIO interrupt with debounce + refractory
# What it does:
#   - Uses a hardware GPIO interrupt to timestamp both edges from a lick sensor
#   - Debounces edges in the IRQ (microseconds) to reject chatter
#   - In the main loop, applies a refractory window to count discrete licks on the
#     configured "active" edge and prints events with timestamps
#   - Optional: flashes the onboard LED on detected licks
#
# Wiring (3.3 V logic only):
#   - Lick sensor output -> GP14 (default). Ground common with the Pico
#   - If the sensor is open-collector/drain, enable the internal pull-up here OR use
#     an external 10 kΩ pull-up to 3.3 V
#   - If the sensor outputs 5 V or analog, DO NOT connect directly. Use a comparator
#     or level shifting so the Pico pin sees 0..3.3 V only
#
# Old-to-new mapping (two-pin phototransistor receiver):
#   - Formerly A0  -> GP14 (collector)
#   - Formerly 5V  -> GND  (emitter)
#   - IR LED is separate: 3V3(OUT) -> ~220 Ω -> LED anode; LED cathode -> GND
#
# Test hint: with USE_INTERNAL_PULLUP=True, GP14 should idle HIGH. Touch GP14 to GND
# briefly with a jumper to simulate a lick; you should see EDGE/LICK printed and the
# onboard LED flash.
#
# Note: This IRQ version is DIGITAL; it has no analog threshold. If you want to
#       keep analog sensing on an ADC (e.g., GP26/ADC0) with a threshold, use a
#       separate script (02b_lick_adc.py).

from machine import Pin
import time, micropython
from array import array

# ---------------- Configuration ----------------
LICK_PIN = 14                 # GPIO for lick input (formerly A0 -> GP14)
USE_INTERNAL_PULLUP = True    # True for open-drain sensors to GND
ACTIVE_LEVEL = 0              # 1 if "lick" is logic HIGH; 0 if logic LOW (typical with pull-up)
DEBOUNCE_US = 1000            # ignore edges that occur within this many microseconds
REFRACTORY_MS = 60            # minimum time between counted licks (on active edge)
LED_FLASH_MS = 30             # onboard LED flash duration on lick
LEVEL_PRINT_MS = 1000         # print idle pin level every N ms (set 0 to disable)

# ---------------- Setup pins ----------------
if USE_INTERNAL_PULLUP:
    lick = Pin(LICK_PIN, Pin.IN, Pin.PULL_UP)
else:
    lick = Pin(LICK_PIN, Pin.IN)

# Onboard LED (Pico W uses "LED" alias; original Pico uses GP25)
try:
    led = Pin("LED", Pin.OUT)
except Exception:
    led = Pin(25, Pin.OUT)
led.value(0)

# Boot heartbeat (script alive)
for _ in range(2):
    led.value(1); time.sleep_ms(80)
    led.value(0); time.sleep_ms(80)

# ---------------- IRQ-safe event ring buffer ----------------
micropython.alloc_emergency_exception_buf(256)

BUF_N = 256
EV_NONE = 0
EV_RISE = 1
EV_FALL = 2

evt_time_us = array('I', [0] * BUF_N)   # 32-bit timestamps (wraps ~71 min)
evt_code = bytearray(BUF_N)             # small code per event
head = 0
tail = 0
_dropped = 0
last_edge_us = 0


def _push_event(t_us, code):
    global head, _dropped
    nxt = (head + 1) & (BUF_N - 1) if (BUF_N & (BUF_N - 1)) == 0 else (head + 1) % BUF_N
    if nxt == tail:
        _dropped += 1
        return
    evt_time_us[head] = t_us & 0xFFFFFFFF
    evt_code[head] = code
    head = nxt


# ---------------- IRQ handler ----------------

def _on_edge(pin):
    global last_edge_us
    t = time.ticks_us()
    # Debounce in IRQ
    if time.ticks_diff(t, last_edge_us) < DEBOUNCE_US:
        return
    last_edge_us = t
    lvl = pin.value()
    code = EV_RISE if lvl else EV_FALL
    _push_event(t, code)


# Use both edges
lick.irq(handler=_on_edge, trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, hard=True)

# ---------------- Main loop: drain buffer, apply refractory, print ----------------
refract_until_ms = 0
last_level_print_ms = time.ticks_ms()

try:
    while True:
        had_event = False
        # Drain buffered edges
        while tail != head:
            had_event = True
            t_us = evt_time_us[tail]
            code = evt_code[tail]
            tail = (tail + 1) & (BUF_N - 1) if (BUF_N & (BUF_N - 1)) == 0 else (tail + 1) % BUF_N

            # Print raw edge for diagnostics
            if code == EV_RISE:
                print("EDGE RISE", t_us)
            elif code == EV_FALL:
                print("EDGE FALL", t_us)

            # Count a lick on the configured active edge with refractory
            is_active_edge = (code == (EV_RISE if ACTIVE_LEVEL == 1 else EV_FALL))
            now_ms = time.ticks_ms()
            if is_active_edge and time.ticks_diff(now_ms, refract_until_ms) >= 0:
                print("LICK", t_us)
                refract_until_ms = time.ticks_add(now_ms, REFRACTORY_MS)
                # brief LED flash
                led.value(1)
                t0 = time.ticks_ms()
                while time.ticks_diff(time.ticks_ms(), t0) < LED_FLASH_MS:
                    pass
                led.value(0)

        # Periodic idle level print (helps verify wiring/pull-up)
        if LEVEL_PRINT_MS:
            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, last_level_print_ms) >= LEVEL_PRINT_MS:
                last_level_print_ms = now_ms
                print("LEVEL", lick.value())

        # Idle politely
        time.sleep_ms(1)
except KeyboardInterrupt:
    led.value(0)

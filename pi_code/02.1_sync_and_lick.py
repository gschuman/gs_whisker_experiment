# 03_sync_and_lick.py — Combined sync pulses (Timer) + lick detection (GPIO IRQ)
# What it does:
#   - Emits short TTL sync pulses on GP13 using a hardware Timer (high-rate, low jitter)
#   - Captures lick sensor edges on GP14 via a GPIO interrupt with IRQ-level debounce
#   - Applies a refractory window to count discrete licks and briefly flashes LED
#   - Prints startup status and LICK events
#
# Wiring (3.3 V logic only):
#   - Sync out: GP13 -> external system TTL input (share ground)
#   - Lick receiver (2-pin phototransistor):
#       Formerly A0  -> GP14 (collector)
#       Formerly 5V  -> GND  (emitter)
#     Enable internal pull-up below (or add 10 kΩ from GP14 to 3V3(OUT))
#   - IR LED (emitter): 3V3(OUT) -> ~150 Ω -> LED anode; LED cathode -> GND

from machine import Pin, Timer
import time, micropython
from array import array
# PIO removed for this variant

# ---------------- Configuration ----------------
# Sync (Timer)
SYNC_PIN = 13
SYNC_F_HZ = 1000           # sync pulse frequency (Hz) — increase as needed
PULSE_WIDTH_US = 200       # pulse width (microseconds); keep small at high rates

# Lick input
LICK_PIN = 14             # GPIO for lick input (formerly A0 -> GP14)
USE_INTERNAL_PULLUP = True
ACTIVE_LEVEL = 0          # 1 if "lick" is logic HIGH; 0 if logic LOW (typical with pull-up)
DEBOUNCE_US = 300         # IRQ debounce (us)
REFRACTORY_MS = 30        # lick refractory (ms)
LED_FLASH_MS = 30         # LED flash duration (ms)
LEVEL_PRINT_MS = 0        # set >0 to print idle level periodically

# ---------------- Pins ----------------
sync = Pin(SYNC_PIN, Pin.OUT, value=0)
if USE_INTERNAL_PULLUP:
    lick = Pin(LICK_PIN, Pin.IN, Pin.PULL_UP)
else:
    lick = Pin(LICK_PIN, Pin.IN)

try:
    led = Pin("LED", Pin.OUT)
except Exception:
    led = Pin(25, Pin.OUT)
led.value(0)

# Boot heartbeat
for _ in range(2):
    led.value(1); time.sleep_ms(80)
    led.value(0); time.sleep_ms(80)

# Startup status
_init_level = lick.value()
print("Tripwire init: GP14={} -> {}".format(
    _init_level,
    "LIGHT detected (beam present)" if _init_level else "DARK / beam broken"
))

# ---------------- IRQ-safe event buffers ----------------
micropython.alloc_emergency_exception_buf(256)

# Main ring buffer (consumer in main loop)
BUF_N = 256
MASK = BUF_N - 1
EV_RISE = 1
EV_FALL = 2

evt_time_us = array('I', [0] * BUF_N)
evt_code = bytearray(BUF_N)
head = 0
tail = 0

# Tiny pending buffer for IRQ (power-of-two length)
PEND_N = 16
PEND_MASK = PEND_N - 1
pend_time_us = array('I', [0] * PEND_N)
pend_code = bytearray(PEND_N)
pend_head = 0
pend_tail = 0
scheduled = 0
last_edge_us = 0

# Non-blocking LED flash state
led_flash_until_ms = 0


def _push_main(t_us, code):
    global head
    nxt = (head + 1) & MASK
    if nxt == tail:
        return  # drop if full
    evt_time_us[head] = t_us
    evt_code[head] = code
    head = nxt


def _drain_pending(_):
    global pend_tail, scheduled
    # Move all pending items to main buffer
    while pend_tail != pend_head:
        t_us = pend_time_us[pend_tail]
        code = pend_code[pend_tail]
        pend_tail = (pend_tail + 1) & PEND_MASK
        _push_main(t_us, code)
    scheduled = 0


@micropython.native
def _on_edge(pin):
    # Minimal ISR: timestamp, debounce, enqueue pending, schedule drain
    global last_edge_us, pend_head, pend_tail, scheduled
    t = time.ticks_us()
    if time.ticks_diff(t, last_edge_us) < DEBOUNCE_US:
        return
    last_edge_us = t
    code = EV_RISE if pin.value() else EV_FALL
    nxt = (pend_head + 1) & PEND_MASK
    if nxt != pend_tail:  # not full
        pend_time_us[pend_head] = t
        pend_code[pend_head] = code
        pend_head = nxt
    if not scheduled:
        scheduled = 1
        micropython.schedule(_drain_pending, 0)

# Soft IRQ
lick.irq(handler=_on_edge, trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING)

# ---------------- Timer-based sync pulses ----------------
sync_timer = Timer()

def _sync_tick(_t):
    sync.value(1)
    tstart = time.ticks_us()
    while time.ticks_diff(time.ticks_us(), tstart) < PULSE_WIDTH_US:
        pass
    sync.value(0)

sync_timer.init(freq=SYNC_F_HZ, mode=Timer.PERIODIC, callback=_sync_tick)

# ---------------- Main loop: drain events, refractory, optional prints ----------------
refract_until_ms = 0
last_level_print_ms = time.ticks_ms()

try:
    while True:
        # Drain queued lick edges
        while tail != head:
            t_us = evt_time_us[tail]
            code = evt_code[tail]
            tail = (tail + 1) & MASK

            is_active_edge = (code == (EV_RISE if ACTIVE_LEVEL == 1 else EV_FALL))
            now_ms = time.ticks_ms()
            if is_active_edge and time.ticks_diff(now_ms, refract_until_ms) >= 0:
                print("LICK", t_us)
                refract_until_ms = time.ticks_add(now_ms, REFRACTORY_MS)
                led.value(1)
                led_flash_until_ms = time.ticks_add(now_ms, LED_FLASH_MS)

        # Non-blocking LED flash off
        if led_flash_until_ms:
            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, led_flash_until_ms) >= 0:
                led.value(0)
                led_flash_until_ms = 0

        # Optional level print
        if LEVEL_PRINT_MS:
            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, last_level_print_ms) >= LEVEL_PRINT_MS:
                last_level_print_ms = now_ms
                print("LEVEL", lick.value())

        time.sleep_ms(1)
except KeyboardInterrupt:
    pass
finally:
    try:
        sync_timer.deinit()
    except Exception:
        pass
    sync.value(0)
    led.value(0)

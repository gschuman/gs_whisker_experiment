# 02.2_sync_analog_lick.py — PIO sync clock on GP13 + analog lick on GP26 (ADC)
# What it does:
#   - Generates a low-jitter square wave on GP13 using PIO (hardware-timed)
#   - Reads the lick sensor on GP26 (ADC0) using a threshold with hysteresis
#   - Adds IIR smoothing and requires consecutive samples to declare/clear lick
#   - Auto-calibrates baseline at startup, prints threshold, reports LICK events
#
# Wiring (3.3 V logic only):
#   - Sync out: GP13 -> external system TTL input (share ground)
#   - Analog lick (2-pin receiver → ADC):
#       Formerly A0  -> GP26 (ADC0)  [this is the analog node]
#       Formerly 5V  -> GND          [receiver return]
#       47 kΩ: GP26  -> 3V3(OUT)     [pull-up creates a measurable voltage]
#   - IR LED: 3V3(OUT) -> 150 Ω -> LED anode; LED cathode -> GND
#   - All grounds common

from machine import Pin, ADC
import time
from rp2 import PIO, StateMachine, asm_pio

# ---------------- Sync (PIO) configuration ----------------
SYNC_PIN = 13
SYNC_F_HZ = 100            # desired frequency (Hz)
DUTY = 0.50                # duty cycle (0..1)
SM_ID = 0                  # PIO state machine index
SM_CLK_HZ = 125_000_000    # PIO clock (matches system clock by default)

@asm_pio(set_init=PIO.OUT_LOW)
def pio_square():
    # pull -> high_ticks, pull -> low_ticks
    pull(block)
    mov(x, osr)
    pull(block)
    mov(y, osr)
    label("loop")
    set(pins, 1)
    label("h1")
    jmp(x_dec, "h1")
    set(pins, 0)
    label("l1")
    jmp(y_dec, "l1")
    jmp("loop")

sm = StateMachine(SM_ID, pio_square, freq=SM_CLK_HZ, set_base=Pin(SYNC_PIN))
period_ticks = max(2, int(SM_CLK_HZ // SYNC_F_HZ))
high_ticks = max(1, int(period_ticks * max(0.0, min(1.0, DUTY))))
low_ticks = max(1, period_ticks - high_ticks)
sm.put(high_ticks)
sm.put(low_ticks)
sm.active(1)
actual_f = SM_CLK_HZ / (high_ticks + low_ticks)
print("SYNC_PIO GP{}: target={} Hz, duty={:.0f}%, actual={:.3f} Hz".format(
    SYNC_PIN, SYNC_F_HZ, DUTY * 100.0, actual_f
))

# ---------------- Analog lick configuration ----------------
ADC_PIN = 26               # GP26 -> ADC0
SAMPLE_HZ = 2000           # ADC sample rate (Hz)
BASELINE_MS = 600          # baseline capture duration (ms) with beam present
LICK_ON_LOW = False        # True: more light lowers voltage; False: more light raises voltage
# Thresholds: stronger hysteresis and offset
DELTA_FRAC = 0.15          # threshold offset = baseline * DELTA_FRAC
HYST_FRAC  = 0.10          # hysteresis (fraction of baseline)
# Consecutive-sample requirement (debounce in samples)
SAMPLES_ON  = 3            # need N consecutive beyond thr_enter to declare lick
SAMPLES_OFF = 3            # need N consecutive past thr_exit to clear lick
# Simple IIR smoothing: filtered = (filtered*(DEN-1) + val) // DEN
IIR_DEN = 8                # 8 → ~1/8 new, 7/8 history
REFRACTORY_MS = 30         # min time between licks (ms)
# Periodic reporting (set to 0 to disable)
REPORT_MS = 0

adc = ADC(ADC_PIN)

# ---------------- Baseline capture ----------------
print("Calibrating baseline... keep beam in normal (unbroken) state")
start = time.ticks_ms()
sum_val = 0
count = 0
sample_interval_us = max(200, int(1_000_000 // SAMPLE_HZ))
next_t = time.ticks_us()
while time.ticks_diff(time.ticks_ms(), start) < BASELINE_MS:
    # pacing
    now = time.ticks_us()
    if time.ticks_diff(now, next_t) < 0:
        continue
    next_t = time.ticks_add(now, sample_interval_us)
    v = adc.read_u16()
    sum_val += v
    count += 1

baseline = sum_val // max(1, count)
# thresholds with hysteresis
if LICK_ON_LOW:
    thr_enter = max(0, int(baseline - baseline * DELTA_FRAC))
    thr_exit  = max(0, int(baseline - baseline * HYST_FRAC))
else:
    thr_enter = min(65535, int(baseline + baseline * DELTA_FRAC))
    thr_exit  = min(65535, int(baseline + baseline * HYST_FRAC))

# Clear, concise startup summary
print("Baseline = {}  Threshold = {}".format(baseline, thr_enter))

# ---------------- Run loop ----------------
lick_active = False
refract_until_ms = 0
sample_count = 0
filtered = baseline
below_count = 0
above_count = 0
last_report_ms = time.ticks_ms()
last_raw = baseline

try:
    while True:
        # paced sampling
        now = time.ticks_us()
        if time.ticks_diff(now, next_t) > 0:
            next_t = time.ticks_add(now, sample_interval_us)
            val = adc.read_u16()
            last_raw = val
            sample_count += 1

            # IIR smoothing (integer EMA)
            filtered = (filtered * (IIR_DEN - 1) + val) // IIR_DEN

            # Count consecutive samples relative to thresholds
            if LICK_ON_LOW:
                if filtered <= thr_enter:
                    below_count = min(255, below_count + 1)
                else:
                    below_count = 0
                if filtered >= thr_exit:
                    above_count = min(255, above_count + 1)
                else:
                    above_count = 0
            else:
                # Lick on high
                if filtered >= thr_enter:
                    below_count = min(255, below_count + 1)
                else:
                    below_count = 0
                if filtered <= thr_exit:
                    above_count = min(255, above_count + 1)
                else:
                    above_count = 0

            # Determine next state with consecutive-sample requirement
            next_active = lick_active
            if LICK_ON_LOW:
                if not lick_active and below_count >= SAMPLES_ON:
                    next_active = True
                elif lick_active and above_count >= SAMPLES_OFF:
                    next_active = False
            else:
                if not lick_active and below_count >= SAMPLES_ON:
                    next_active = True
                elif lick_active and above_count >= SAMPLES_OFF:
                    next_active = False

            # Rising edge of lick_active counts a lick (with refractory)
            if not lick_active and next_active:
                now_ms = time.ticks_ms()
                if time.ticks_diff(now_ms, refract_until_ms) >= 0:
                    print("LICK", time.ticks_us())
                    refract_until_ms = time.ticks_add(now_ms, REFRACTORY_MS)

            lick_active = next_active

        # Optional: periodic report (disabled if REPORT_MS == 0)
        if REPORT_MS:
            nms = time.ticks_ms()
            if time.ticks_diff(nms, last_report_ms) >= REPORT_MS:
                last_report_ms = nms
                print("ADC raw={} filt={} thr_enter={} thr_exit={} state={}".format(
                    last_raw, filtered, thr_enter, thr_exit, int(lick_active)
                ))

        # small idle to yield
        time.sleep_us(200)

except KeyboardInterrupt:
    pass
finally:
    sm.active(0)
    Pin(SYNC_PIN, Pin.OUT).value(0)

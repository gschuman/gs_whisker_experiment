# pico_lick_sync.py — Final lick detection (IRQ, microsecond clock) + PIO sync + USB event stream
# Purpose:
#   - Pico W firmware (MicroPython rp2 build) for fast, robust lick logging
#   - Timestamps both lick edges using time.ticks_us() in an IRQ-safe path
#   - Emits a hardware-timed sync clock via PIO on GP13
#   - Streams compact 12-byte binary event records to USB CDC (stdout)
#   - Minimal command support: SYNC marker and (optional) ARM window & valve pulse hooks (stubbed)
#
# Event record format (12 bytes, little-endian):
#   uint32 t_us, uint16 code, int16 aux, uint32 seq
# Codes used here:
#   1 = LICK_RISE, 2 = LICK_FALL, 40 = SYNC (aux=marker)
#
# Wiring (3.3 V):
#   - Lick input -> GP14 (use internal pull-up for open-drain sensors)
#   - Sync PIO out -> GP13
#
# Usage:
#   - Copy to Pico as main.py or run via Thonny/rshell
#   - On the Pi: open /dev/ttyACM0 and read 12-byte records; store to disk

from machine import Pin
import time, sys, struct, micropython
try:
    from rp2 import PIO, StateMachine, asm_pio
except Exception:
    PIO = None
    StateMachine = None
    asm_pio = None


# ---------------- Configuration ----------------
SYNC_PIN = 13              # PIO sync out
SYNC_F_HZ = 1000           # Hz, continuous clock for alignment
SYNC_DUTY = 0.50           # duty 0..1

LICK_PIN = 14              # digital lick sensor input
USE_PULLUP = True          # enable internal pull-up if sensor is open-drain
DEBOUNCE_US = 300          # IRQ-level debounce

# Event codes
EV_LICK_RISE = 1
EV_LICK_FALL = 2
EV_SYNC      = 40

# Ring buffer sizing (12 bytes per event)
BUF_N = 4096


# ---------------- Pins ----------------
lick = Pin(LICK_PIN, Pin.IN, Pin.PULL_UP if USE_PULLUP else None)


# ---------------- PIO sync clock ----------------
if PIO is not None:
    @asm_pio(set_init=PIO.OUT_LOW)
    def pio_square():
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

    sm = StateMachine(0, pio_square, freq=125_000_000, set_base=Pin(SYNC_PIN))
    period = max(2, int(sm.freq // SYNC_F_HZ))
    high = max(1, int(period * max(0.0, min(1.0, SYNC_DUTY))))
    low  = max(1, period - high)
    sm.put(high)
    sm.put(low)
    sm.active(1)
else:
    sm = None


# ---------------- Event buffer ----------------
micropython.alloc_emergency_exception_buf(256)
pack = struct.Struct('<I H h I')
evt_buf = bytearray(BUF_N * 12)
head = 0
tail = 0
seq = 0


def _push_event(t_us, code, aux):
    global head, seq
    base = (head % BUF_N) * 12
    pack.pack_into(evt_buf, base, t_us & 0xFFFFFFFF, code, aux, seq)
    head = (head + 1) & 0x3FFFFFFF
    seq = (seq + 1) & 0xFFFFFFFF


def _flush_events():
    global tail
    while tail != head:
        start = (tail % BUF_N) * 12
        end_slots = BUF_N - (tail % BUF_N)
        avail = (head - tail) & 0x3FFFFFFF
        n_slots = end_slots if avail > end_slots else avail
        n_bytes = n_slots * 12
        sys.stdout.buffer.write(memoryview(evt_buf)[start:start + n_bytes])
        tail = (tail + n_slots) & 0x3FFFFFFF


# ---------------- Lick IRQ ----------------
last_edge_us = 0


def _on_edge(pin):
    global last_edge_us
    t = time.ticks_us()
    if time.ticks_diff(t, last_edge_us) < DEBOUNCE_US:
        return
    last_edge_us = t
    lvl = pin.value()
    _push_event(t, EV_LICK_RISE if lvl else EV_LICK_FALL, 0)


lick.irq(handler=_on_edge, trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, hard=True)


# ---------------- Optional simple command parser (Pi->Pico) ----------------
def _read_exact(n):
    buf = bytearray(n)
    mv = memoryview(buf)
    got = 0
    while got < n:
        r = sys.stdin.buffer.readinto(mv[got:])
        if not r:
            return None
        got += r
    return buf


def _handle_cmd(op, payload):
    # 0x06 = SYNC marker (aux=marker uint16)
    if op == 0x06 and payload and len(payload) >= 2:
        marker = payload[0] | (payload[1] << 8)
        _push_event(time.ticks_us(), EV_SYNC, marker)


def _poll_cmd_once():
    hdr = _read_exact(4)
    if not hdr or hdr != b'CMD0':
        return
    meta = _read_exact(4)
    if not meta:
        return
    op = meta[0]
    plen = meta[2] | (meta[3] << 8)
    payload = _read_exact(plen) if plen else b''
    _handle_cmd(op, payload)


# ---------------- Main loop ----------------
try:
    while True:
        _flush_events()
        _poll_cmd_once()
        # yield a bit; sync clock runs in PIO
        time.sleep_ms(1)
except KeyboardInterrupt:
    pass
finally:
    if sm:
        sm.active(0)
    Pin(SYNC_PIN, Pin.OUT).value(0)


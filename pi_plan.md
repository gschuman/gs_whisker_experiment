Below is a conceptual blueprint first, then lean (but concrete) code outlines for **MicroPython on the Pico** (time-critical I/O/timestamps) and **Python on the Pi** (trial logic, orchestration, logging).

---

# Conceptual architecture (who does what, and why)

## The split

* **Pico (RP2040, MicroPython)** = **time-critical layer**

  * Maintains a **single on-board timebase** (microseconds).
  * **Timestamps** every lick edge *at the source* (GPIO IRQ).
  * Generates **exact-duration** valve pulses (one-shot hardware timer).
  * Drives steppers with **hardware-timed pulses** (PIO or PWM), not in Python loops.
  * Emits **TTL sync** for cameras/ephys and logs the exact time it toggled them.
  * Runs a **tiny “safety FSM”** when needed (e.g., “if a lick arrives while the *armed response window* is active, deliver water immediately”).
  * Streams a compact **binary event log** to the Pi (USB CDC), in bursts.

* **Pi 5 (CPython)** = **orchestration & data layer**

  * Schedules trials (GO/NOGO, counterbalancing, ITI distribution).
  * Chooses which texture to present and **tells the Pico** to move steppers (in/out, rotate to index).
  * Arms/disarms **response windows** and configures reward magnitude/punishment policy.
  * **Logs** all events from the Pico to disk (Parquet/Feather or raw binary).
  * Optional: live plots/UI, session management, metadata.

## Signals / wiring (3.3 V logic everywhere)

* **Lick input (to Pico)**: 3.3 V logic from your lickometer (beam-break or conditioned conductive). Add RC + Schmitt if noisy.
* **Solenoid valve (from Pico)**: MOSFET low-side driver + flyback diode; **external 12 V** supply; **common ground** with Pico/Pi.
* **Steppers (from Pico)**: STEP/DIR into drivers (e.g., TMC2209/TMC5160). Motor power is separate; grounds common.
* **Sync TTLs**: Pico produces a short pulse at **trial start**, **stim on**, **water on**, etc. Mirror those pulses into other systems and also log them as events on the Pico.
* **USB**: single USB cable Pico↔Pi carries commands up and events down.

## Timing model (what avoids the “bad limits”)

* **Edge timestamps at the source**: Every lick edge gets a microsecond tick from the Pico’s hardware clock. You never “sample at 1 kHz”; you **record edges** with precise times.
  → Avoids polling/aliasing/jitter from Python scheduling.
* **Outputs scheduled by hardware**: Valve open/close uses a one-shot timer; step pulses come from **PIO** or PWM hardware, not from Python waits.
  → Output durations and pulse trains stay microsecond-accurate even if Python is busy.
* **PC is orchestration, not hard real-time**: The Pi decides *when* to arm a window, where to move steppers, etc., but the **immediate** lick→reward latency can be handled by the Pico’s tiny FSM when armed.
  → Avoids Linux/USB jitter for the critical conditional.

---

# Event vocabulary & binary wire format

We’ll keep it minimal and symmetrical across Pico and Pi.

**Event record (12 bytes):**

```
uint32 t_us   # Pico microsecond clock (wraps ~71 min)
uint16 code   # event code (see below)
 int16 aux    # small parameter; purpose depends on code
uint32 seq    # monotonically increasing sequence number
```

**Example event codes (add as needed):**

* `1` = LICK\_RISE, `2` = LICK\_FALL
* `10` = STATE (aux = enum: IDLE, STIM\_ON, RESPONSE, HIT, MISS, FA, TIMEOUT, …)
* `20` = WATER\_ON (aux=ms), `21` = WATER\_OFF
* `30` = STEP\_START (aux = motor\_id), `31` = STEP\_END (aux = motor\_id)
* `40` = SYNC (aux = marker id)
* `50` = WINDOW\_ARMED (aux=ms), `51` = WINDOW\_CLEARED

**Command frames (Pi → Pico)** (all binary; simple, robust):
Header `"CMD0"`, 1-byte opcode, 1-byte version, 2-byte payload length, payload, 2-byte CRC16.

Useful opcodes:

* `0x01` SET\_PARAMS (e.g., reward\_ms, debounce\_us)
* `0x02` ARM\_WINDOW (go/no-go flag, window\_ms)
* `0x03` SOLENOID\_PULSE (ms)
* `0x04` STEPPER\_MOVE (motor\_id, steps, hz, dir)
* `0x05` STEPPER\_GOTO\_INDEX (motor\_id, index)
* `0x06` SYNC\_PULSE (marker id)
* `0x07` SET\_DIRTY\_BITS (flags), `0x08` ABORT

You can start dirt-simple (no CRC, blocking reads) and add robustness later.

---

# MicroPython on Pico — code outline

> This is a compact, runnable **skeleton** to show the patterns. It uses:
>
> * **GPIO IRQ** for lick edges (timestamps via `time.ticks_us()`).
> * **Timer one-shot** for precise valve duration.
> * **PIO** for stepper pulses (accurate period & step count) — illustrative program.
> * **USB CDC** stdio to stream binary events (run on the Pico as a script).

```python
# pico_firmware.py  (MicroPython, RP2040)
from machine import Pin, Timer
import time, micropython, sys, struct
try:
    import rp2  # PIO
except:
    rp2 = None

# ---------------- Hardware pins (edit to match wiring) ----------------
PIN_LICK   = 14        # GPIO for lick sensor (3.3V logic)
PIN_VALVE  = 15        # MOSFET gate
PIN_SYNC   = 13        # TTL sync out
PIN_STEP1  = 16        # STEP for linear actuator
PIN_DIR1   = 17        # DIR "
PIN_STEP2  = 18        # STEP for rotary index wheel
PIN_DIR2   = 19        # DIR "

lick = Pin(PIN_LICK, Pin.IN, Pin.PULL_UP)
valve = Pin(PIN_VALVE, Pin.OUT, value=0)
sync  = Pin(PIN_SYNC,  Pin.OUT, value=0)
step1 = Pin(PIN_STEP1, Pin.OUT, value=0)
dir1  = Pin(PIN_DIR1,  Pin.OUT, value=0)
step2 = Pin(PIN_STEP2, Pin.OUT, value=0)
dir2  = Pin(PIN_DIR2,  Pin.OUT, value=0)

# ---------------- Event machinery ----------------
micropython.alloc_emergency_exception_buf(256)

EV_LICK_RISE=1; EV_LICK_FALL=2
EV_STATE=10
EV_WATER_ON=20; EV_WATER_OFF=21
EV_SYNC=40
EV_WINDOW_ARMED=50; EV_WINDOW_CLEARED=51
EV_STEP_START=30; EV_STEP_END=31

# Simple ring buffer of packed events (avoid allocs in IRQ)
BUF_N = 4096
evt_buf = bytearray(BUF_N * 12)
head = 0
tail = 0
seq  = 0
pack = struct.Struct('<I H h I')  # t_us, code, aux, seq

def _push_event(t_us, code, aux):
    global head, seq
    base = (head % BUF_N) * 12
    pack.pack_into(evt_buf, base, t_us & 0xFFFFFFFF, code, aux, seq)
    seq = (seq + 1) & 0xFFFFFFFF
    head = (head + 1) % (1<<30)  # big ring; modulo indexing

def flush_events_to_usb():
    global tail
    # stream out as much contiguous data as available
    while tail != head:
        start = (tail % BUF_N) * 12
        # compute contiguous chunk until buffer end or head reached
        end_slots = BUF_N - (tail % BUF_N)
        avail_slots = ((head - tail) & ((1<<30)-1))
        n_slots = min(end_slots, avail_slots if avail_slots < (1<<29) else BUF_N)
        n_bytes = n_slots * 12
        # write slice
        sys.stdout.buffer.write(memoryview(evt_buf)[start:start+n_bytes])
        tail = (tail + n_slots) % (1<<30)

# ---------------- Lick edge capture (IRQ) ----------------
def on_lick(pin):
    lvl = pin.value()
    code = EV_LICK_RISE if lvl else EV_LICK_FALL
    _push_event(time.ticks_us(), code, 0)

lick.irq(handler=on_lick, trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, hard=True)

# ---------------- Valve (one-shot) ----------------
valve_timer = Timer()

def valve_off_cb(_t):
    valve.value(0)
    _push_event(time.ticks_us(), EV_WATER_OFF, 0)

def valve_pulse_ms(ms):
    valve.value(1)
    _push_event(time.ticks_us(), EV_WATER_ON, ms)
    # one-shot: duration in milliseconds
    valve_timer.init(mode=Timer.ONE_SHOT, period=ms, callback=valve_off_cb)

# ---------------- Sync TTL ----------------
def sync_pulse(marker=1, us=1000):
    sync.value(1)
    _push_event(time.ticks_us(), EV_SYNC, marker)
    # crude us pulse; fine at ~1ms scale
    t0 = time.ticks_us()
    while time.ticks_diff(time.ticks_us(), t0) < us:
        pass
    sync.value(0)

# ---------------- Stepper via PIO (illustrative) ----------------
if rp2:
    @rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT)
    def stepper_pio():
        # Protocol: two pulls: period_ticks (half-period), steps (count), then run
        pull(block)               # OSR = period_ticks
        mov x, osr                # X = period
        pull(block)               # OSR = steps
        mov y, osr                # Y = steps
        label("loop")
        set(pins, 1)        .side(1)
        mov isr, x
        label("h1")
        jmp(x_dec, "h1")
        set(pins, 0)        .side(0)
        mov isr, x
        label("h2")
        jmp(x_dec, "h2")
        jmp(y_dec, "loop")
        irq(0)                    # done

    from rp2 import StateMachine, PIO
    sm_lin = StateMachine(0, stepper_pio, freq=125_000_000, sideset_base=step1)  # ~system clk
    sm_rot = StateMachine(1, stepper_pio, freq=125_000_000, sideset_base=step2)

    def stepper_run(sm, step_pin, dir_pin, steps, hz, direction):
        dir_pin.value(1 if direction else 0)
        # convert target frequency to half-period ticks at SM freq
        half_period = max(1, int( (sm.freq) // (2*hz) ))
        _push_event(time.ticks_us(), EV_STEP_START, 1 if step_pin is step2 else 0)
        sm.active(0)
        sm.put(half_period)
        sm.put(int(steps))
        sm.irq(handler=lambda p: _push_event(time.ticks_us(), EV_STEP_END, 1 if step_pin is step2 else 0))
        sm.active(1)
else:
    def stepper_run(*a, **kw):
        raise RuntimeError("PIO not available; flash the rp2 build of MicroPython.")

# ---------------- Minimal command decoder (Pi -> Pico) ----------------
# Frame: b'CMD0' | opcode(1) | ver(1) | len(2 LE) | payload
def read_exact(n):
    buf = bytearray(n)
    mv = memoryview(buf)
    got = 0
    while got < n:
        r = sys.stdin.buffer.readinto(mv[got:])
        if not r:
            return None
        got += r
    return buf

def handle_cmd(opcode, payload):
    # opcodes (examples)
    # 0x02 ARM_WINDOW: payload = go_flag(uint8), window_ms(uint16)
    # 0x03 SOLENOID_PULSE: payload = ms(uint16)
    # 0x04 STEPPER_MOVE: payload = motor_id(uint8), steps(int32), hz(uint32), dir(uint8)
    # 0x06 SYNC_PULSE: payload = marker(uint16)
    if opcode == 0x03:
        ms, = struct.unpack_from('<H', payload, 0)
        valve_pulse_ms(ms)
    elif opcode == 0x04:
        motor_id, = struct.unpack_from('<B', payload, 0)
        steps, hz, direction = struct.unpack_from('<i I B', payload, 1)
        if motor_id == 0:
            stepper_run(sm_lin, step1, dir1, steps, hz, direction)
        else:
            stepper_run(sm_rot, step2, dir2, steps, hz, direction)
    elif opcode == 0x06:
        marker, = struct.unpack_from('<H', payload, 0)
        sync_pulse(marker)
    elif opcode == 0x02:
        go_flag, window_ms = struct.unpack_from('<B H', payload, 0)
        # Minimal "armed" gate: next lick while armed → water if GO
        _push_event(time.ticks_us(), EV_WINDOW_ARMED, window_ms)
        deadline = time.ticks_add(time.ticks_ms(), window_ms)
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            # check lick flag by peeking recent buffer? For simplicity we poll pin level.
            if lick.value() == 1:   # simplistic; you can track last edge time instead
                if go_flag:
                    valve_pulse_ms(30)
                # either way, clear window
                break
        _push_event(time.ticks_us(), EV_WINDOW_CLEARED, 0)
    # add more as needed

def main():
    while True:
        # flush events opportunistically
        flush_events_to_usb()
        # try to read a command
        hdr = read_exact(4)
        if not hdr:
            continue
        if hdr != b'CMD0':
            continue
        meta = read_exact(4)
        if not meta:
            continue
        opcode = meta[0]
        _ver   = meta[1]
        plen   = meta[2] | (meta[3] << 8)
        payload = read_exact(plen) if plen else b''
        handle_cmd(opcode, payload)

main()
```

**Notes**

* The lick IRQ uses `hard=True` so it runs in the fast interrupt context. We avoid allocations inside the IRQ by packing into a pre-allocated ring buffer.
* The valve duration is precise because it’s scheduled by the **hardware timer**; Python execution speed doesn’t affect its end time.
* The example **PIO** program produces a specified number of step pulses at a specified frequency; MicroPython just loads parameters. (For ramps/accel, update `hz` over time or use a smarter PIO.)
* The tiny **armed response window** is shown as a simple loop — for better determinism, you can make it fully event-driven by latching the timestamp of the last lick edge inside the IRQ and having the window just check that variable.

---

# Pi 5 (CPython) — orchestration & logging outline

> This script:
> • Opens the Pico’s CDC serial (`/dev/ttyACM0`).
> • Provides helpers to send commands & read events.
> • Runs a toy trial loop (rotate texture → move in → arm window).

```python
# pi_orchestrator.py  (Python 3, run on Pi 5)
import serial, struct, threading, queue, time, pathlib
from datetime import datetime

PORT = '/dev/ttyACM0'   # adjust as needed
BAUD = 115200           # ignored for CDC but some stacks want it set

ser = serial.Serial(PORT, BAUD, timeout=0.01)

def send_cmd(op, payload=b''):
    hdr = b'CMD0' + bytes([op, 0]) + struct.pack('<H', len(payload))
    ser.write(hdr + payload)

# --- High-level helpers ---
def valve(ms=30):
    send_cmd(0x03, struct.pack('<H', ms))

def step_move(motor_id, steps, hz, direction):
    send_cmd(0x04, struct.pack('<B i I B', motor_id, steps, hz, 1 if direction else 0))

def sync_pulse(marker=1):
    send_cmd(0x06, struct.pack('<H', marker))

def arm_window(go_flag: bool, window_ms: int):
    send_cmd(0x02, struct.pack('<B H', 1 if go_flag else 0, window_ms))

# --- Event reader thread ---
EV_REC = struct.Struct('<I H h I')

evt_q = queue.Queue()

def reader():
    buf = bytearray()
    while True:
        chunk = ser.read(4096)
        if not chunk: 
            continue
        buf += chunk
        # parse complete 12-byte records
        while len(buf) >= 12:
            rec = bytes(buf[:12]); del buf[:12]
            t_us, code, aux, seq = EV_REC.unpack(rec)
            evt_q.put((t_us, code, aux, seq))

threading.Thread(target=reader, daemon=True).start()

# --- Simple logger ---
out = open(f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}.bin", "ab")

def drain_to_disk():
    c = 0
    while True:
        try:
            t_us, code, aux, seq = evt_q.get_nowait()
            out.write(EV_REC.pack(t_us, code, aux, seq))
            c += 1
        except queue.Empty:
            break
    if c: out.flush()

# --- Toy trial loop (replace with your scheduler) ---
TEXTURE_STEPS = [0, 800, 1600, 2400]  # rotary index step positions
current_idx = 0

def goto_texture(idx):
    global current_idx
    steps = TEXTURE_STEPS[idx] - TEXTURE_STEPS[current_idx]
    direction = steps >= 0
    step_move(1, abs(steps), hz=2000, direction=direction)
    current_idx = idx

def panel_in():
    step_move(0, 2000, hz=3000, direction=True)

def panel_out():
    step_move(0, 2000, hz=3000, direction=False)

def run_session(n_trials=100):
    for t in range(n_trials):
        # pick GO/NOGO & texture
        is_go = (t % 2) == 0
        idx = t % len(TEXTURE_STEPS)
        goto_texture(idx)
        time.sleep(0.2)              # allow motion complete (or watch EV_STEP_END)
        panel_in()
        time.sleep(0.8)

        sync_pulse(marker=1)         # trial start sync (also timestamped on Pico)
        arm_window(is_go, 500)       # 500 ms window; Pico will water immediately on lick if GO
        time.sleep(0.6)

        panel_out()
        time.sleep(1.0)              # ITI (randomize in practice)

        drain_to_disk()

try:
    run_session(20)
finally:
    drain_to_disk()
    out.close()
    ser.close()
```

**Why this meets your constraints**

* **All decisive timestamps** are created on the Pico (edge, valve on/off, sync, step start/end). The Pi merely *tells* the Pico when to arm actions and records the stream.
* The **critical conditional** (“lick within the response window”) can be handled *on the Pico* once the Pi arms the window, so **lick→reward** latency is bounded by **GPIO IRQ + timer start** (tens of µs → low ms worst case), not Python wakes on Linux.
* Stepper motion is **hardware-timed** via PIO; the Pi can’t stall pulses.

---

# Practical setup checklist

1. **Flash MicroPython to Pico** (UF2 from micropython.org). Copy `pico_firmware.py` to the Pico and set it to run on boot (e.g., rename to `main.py`).
2. **Wire**:

   * Lick → Pico GPIO with proper conditioning (3.3 V).
   * Valve → MOSFET → 12 V solenoid + flyback; grounds common.
   * STEP/DIR → drivers → motors; motor PSU isolated, grounds common.
3. **Connect Pico to Pi** via USB; it appears as `/dev/ttyACM0`.
4. **Run** `pi_orchestrator.py` and watch `session_*.bin` grow with events.
5. **Inspect events** in Python (Pandas) and verify sync pulses align with camera/ephys TTLs.

---

# How each component “really works” (first principles & limits)

* **Lick detection**: a digital input line fires a **hardware interrupt** on RISE/FALL. In that ISR we grab the Pico’s **microsecond counter**.
  *Constraint avoided:* polling jitter — the ISR runs quickly regardless of what the rest of the code is doing.

* **Reward solenoid**: a **one-shot hardware timer** turns the MOSFET off after `N` ms; we log both ON and OFF times.
  *Constraint avoided:* Python scheduling jitter — duration accuracy is set by the timer, not by sleep loops.

* **Steppers**: a **PIO state machine** emits evenly-spaced STEP pulses. You pass it frequency and count; it runs independently of Python.
  *Constraint avoided:* “missed steps” from software delays — pulses keep coming on schedule even if your code is busy logging.

* **Sync TTLs**: Pico toggles the pin and logs the **exact tick** of the toggle; anything downstream gets the same pulse with the same absolute time to align analyses.
  *Constraint avoided:* trusting PC time or USB arrival time — both are noisy.

* **Pi orchestration**: Linux is not hard real-time, so we let it **plan** and **log**, not **react at microsecond scale**. The only reactive bit we keep on the Pico (in Python too, but MicroPython on a microcontroller) is the armed window gating.

---

# Where you might extend this

* **Better “armed window”**: Instead of polling `lick.value()`, track `last_lick_ts` updated in the IRQ and just check if it fell within the armed interval.
* **Debounce/glitch filter**: Track the last edge time and reject edges < N µs apart in the IRQ.
* **Acceleration profiles**: Update PIO frequency in small steps for smooth motion.
* **Richer protocol**: Sequence numbers on commands, ACKs, CRC16 on frames.
* **On-device logging**: Add microSD to the Pico for belt-and-suspenders redundancy.

---

## TL;DR

* Keep **all timestamps and pulse timing** on the **Pico** (MicroPython + PIO + timers).
* Let the **Pi** do **trial logic**, scheduling, and **logging** in Python.
* Use a simple **binary protocol**; **arm** minimal MCU-side gates (e.g., response window) whenever you need sub-ms deterministic reactions.

If you’d like, I can package these two outlines into a tiny repo with a Makefile/UF2 instructions and a pandas reader to turn `session_*.bin` into a tidy DataFrame for analysis.

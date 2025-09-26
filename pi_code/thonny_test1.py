from machine import Pin
import time

# Prefer Pico W's LED alias; fall back to GP25 for original Pico
try:
    led = Pin("LED", Pin.OUT)
except Exception:
    led = Pin(25, Pin.OUT)

try:
    while True:
        led.value(1)
        time.sleep(5)
        led.value(0)
        time.sleep(5)
except KeyboardInterrupt:
    led.value(0)
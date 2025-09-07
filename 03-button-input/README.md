# Project 03: Interactive Control with a Debounced Button

**Goal:** To create a truly interactive circuit by introducing a physical input (a push button) to control an output (an LED). This project implements a stateful toggle switch with software debouncing for a reliable user experience.

### Hardware Used

* NVIDIA Jetson Orin Nano
* 1x Push Button (and 1x 2.1K Resistor)
* 1x LED (and 1x 330Ω Resistor)
* Jumper Wires & Breadboard

### The Circuit

![3-button-input-circuit](https://github.com/user-attachments/assets/ad967d82-adfc-446e-b471-96e01794ffd0)

### Watch the video '04-button-input'

### The Code
This Python script uses the `Jetson.GPIO` library to read the button state and toggle the LED. It includes a custom function to "debounce" the mechanical switch, preventing multiple triggers from a single press.

```python
import Jetson.GPIO as GPIO
import time

LED = 7         # physical pin 7
BTN = 12        # physical pin 12

PRESS_CONFIRM_MS = 30
RELEASE_CONFIRM_MS = 30
POLL_MS = 5

def stable_read(expected_level, hold_ms):
    """Return True if BTN stays at expected_level for hold_ms consecutively."""
    samples_needed = max(1, hold_ms // POLL_MS)
    for _ in range(samples_needed):
        if GPIO.input(BTN) != expected_level:
            return False
        time.sleep(POLL_MS / 1000.0)
    return True

GPIO.setmode(GPIO.BOARD)
GPIO.setup(LED, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(BTN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # idle HIGH

print("Ready. Press the button (pin 12) to toggle LED on pin 7. CTRL+C to exit.")

try:
    # Wait for the button to be in a stable, released (HIGH) state initially.
    while not stable_read(GPIO.HIGH, RELEASE_CONFIRM_MS):
        time.sleep(POLL_MS / 1000.0)

    led_on = False

    while True:
        # Check for a stable press (LOW)
        if GPIO.input(BTN) == GPIO.LOW and stable_read(GPIO.LOW, PRESS_CONFIRM_MS):
            # Toggle the LED state
            led_on = not led_on
            GPIO.output(LED, GPIO.HIGH if led_on else GPIO.LOW)
            print("LED", "ON" if led_on else "OFF")

            # Wait here until the button is released to prevent multiple toggles
            while not (GPIO.input(BTN) == GPIO.HIGH and stable_read(GPIO.HIGH, RELEASE_CONFIRM_MS)):
                time.sleep(POLL_MS / 1000.0)

        time.sleep(POLL_MS / 1000.0)

except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()


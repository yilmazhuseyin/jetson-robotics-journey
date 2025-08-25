# Project 02: The LED Chaser ("Knight Rider")

**Goal:** To expand on the first project by controlling a sequence of 6 LEDs to create a scanning light effect. This project focuses on managing multiple outputs.

### Hardware Used

* NVIDIA Jetson Orin Nano
* 6x LEDs
* 6x 330Ω Resistors
* MB-102 Breadboard
* Jumper Wires

### Circuit

![02-led-chaser-circuit-photo](https://github.com/user-attachments/assets/67f26e27-7be9-44c6-b7b1-094d76eb2047)

The 6 LED circuits were connected to the Jetson pins as follows:

| LED | Physical Pin |
| :-- | :----------- |
| 1   | Pin 7        |
| 2   | Pin 12       |
| 3   | Pin 13       |
| 4   | Pin 15       |
| 5   | Pin 32       |
| 6   | Pin 35       |

### Challenges & Learnings

The main challenge in this project was a hardware bug. Initially, all 6 LEDs would turn on dimly at the same time as soon as the Jetson was powered on, and the script had no effect.

After simplifying the circuit to two LEDs, the problem persisted. This indicated a fault with a **shared connection**. I discovered that I had accidentally connected the breadboard's main ground rail to a **3.3V power pin** instead of a **GND pin**. This created a small, constant current path through all the LEDs.

Once this single ground wire was moved to the correct **Pin 6 (GND)**, the circuit behaved exactly as expected.

### Final Working Code

```python
import Jetson.GPIO as GPIO
import time

# Set the pin numbering mode to BOARD (physical pin numbers)
GPIO.setmode(GPIO.BOARD)

# Create a list of the physical pin numbers we are using for our LEDs
LED_PINS = [12, 35, 32, 7, 13, 15]
SCAN_DELAY = 2 # The delay between LED steps

# Set up all the pins in our list as outputs
for pin in LED_PINS:
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

print("Starting LED chaser. Press Ctrl+C to stop.")

try:
    while True:
        # Loop forwards through the list of pins
        for pin in LED_PINS:
            GPIO.output(pin, GPIO.HIGH) # Turn current LED ON
            time.sleep(SCAN_DELAY)
            GPIO.output(pin, GPIO.LOW)  # Turn current LED OFF

        # Loop backwards through the list of pins (from the second-to-last to the second)
        # We use slicing [::-1] to reverse the list easily
        for pin in LED_PINS[::-1]:
            GPIO.output(pin, GPIO.HIGH)
            time.sleep(SCAN_DELAY)
            GPIO.output(pin, GPIO.LOW)
            
finally:
    print("\nProgram stopped. Turning off all LEDs.")
    # Ensure all LEDs are turned off
    for pin in LED_PINS:
        try:
            GPIO.output(pin, GPIO.LOW)
        except Exception as e:
            print(f"Could not turn off pin {pin}: {e}")
            
    # Reset GPIO settings
    GPIO.cleanup()

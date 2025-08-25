# Project 01: The "Hello, World!" of Hardware - Blinking an LED

**Goal:** To control a physical component (an LED) with a Python script running on the Jetson Orin Nano. This served as the foundational project for setting up the entire hardware and software development environment.

### Hardware Used

* NVIDIA Jetson Orin Nano
* 1x LED (Blue)
* 1x 330Ω Resistor
* MB-102 Breadboard
* Jumper Wires

### Circuit

![01-blinking-led-circuit-photo](https://github.com/user-attachments/assets/c7454fd9-94fd-4683-ad47-7b0dad7a8f3b)

The circuit connects the Jetson's physical **Pin 12** to the long leg of the LED via the resistor, and the short leg of the LED to **Pin 6 (GND)**.

### The Journey & Challenges

This seemingly simple project turned into a deep and valuable debugging exercise.

1.  **Initial Library Failure:** The default `Jetson.GPIO` library failed with a `Could not determine Jetson model` error. This was solved by purging the system's library and doing a clean reinstall via `pip`.
2.  **Process Conflict:** During the execution of the blink.py test script, the system generated an error indicating that the requested GPIO pin was already in use by another process. The conflict was resolved by identifying the process ID (PID) associated with the pin and forcefully terminating it. This action released the pin, allowing the script to access it without issue.
3.  **Final Success, GPIO State:** Following the resolution of the process conflict, the GPIO pins remained unresponsive to the script. The pins were manually activated via the NVIDIA command-line interface. A subsequent reboot of the Orin Nano was required, after which the GPIO pins functioned as expected.

### Final Working Code

```python
import Jetson.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

# Set actual pin number like on the NVIDIA board
LED_PIN = 38
BLINK_TIME = 0.5
GPIO.setup(LED_PIN, GPIO.OUT)

print("Press Ctrl+C to stop")

try:
    while True:
        GPIO.output(LED_PIN, GPIO.HIGH)
        print("LED ON") 
        time.sleep(BLINK_TIME)                   
        GPIO.output(LED_PIN, GPIO.LOW)
        print("LED OFF")  
        time.sleep(BLINK_TIME)                  
except KeyboardInterrupt:
    print("Exiting program")

finally:
    GPIO.output(LED_PIN, GPIO.LOW) # led off on final
    GPIO.cleanup()  # reset all GPIO pins

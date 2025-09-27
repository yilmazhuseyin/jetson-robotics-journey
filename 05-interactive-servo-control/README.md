# Project 05: Interactive Servo Control

**Goal:** To build a complete interactive system by combining a physical input (a debounced push button) with a complex output (a servo motor). Each press of the button cycles the servo through a series of predefined angles.

### Hardware Used

* NVIDIA Jetson Orin Nano
* PCA9685 16-Channel Servo Driver
* 1x 15kg 180° Digital Servo Motor
* 1x Push Button and 2K Resistor
* External 5V 2A Power Supply
* Jumper Wires & Breadboard

### The Circuit

This circuit combines the hardware from the previous two projects. The Jetson simultaneously handles direct GPIO input for the button and I2C communication for the servo driver.

#### 1- Control Circuit (I2C):

* Jetson 5V (Pin 2) → PCA9685 VCC

* Jetson SDA (Pin 3) → PCA9685 SDA

* Jetson SCL (Pin 5) → PCA9685 SCL

* Jetson GND (Pin 6) → PCA9685 GND

#### 2- Motor Power Circuit:

* The external 5V 2A power supply is connected to the HW-131 module.
* Jumper wires run from the HW-131's input pins to the breadboard's power rails.

#### 3- Servo Connections:

* The servo is connected to Channel 0 of the PCA9685.

#### 4- Button Circuit: 

* One leg of the push button is connected to Physical Pin 12 on the Jetson. The other leg is connected to Ground (GND). The script uses an internal pull-up resistor, so no external resistor is needed.
* 
![IMG_6445](https://github.com/user-attachments/assets/d31d4796-f651-4f85-80c1-6dcedbb949b0)


### Watch the video 'Robotics Journey #05: Building an Interactive Servo Controller!'

https://youtube.com/shorts/9MNFPlrR364


### The Code
This Python script uses the adafruit_servokit library to control two servos simultaneously, sweeping them back and forth in sync. It explicitly defines the I2C bus to ensure compatibility with the Jetson platform.

```python
import Jetson.GPIO as GPIO
import time
import board
import busio
from adafruit_servokit import ServoKit

BTN_PIN = 12        # Physical pin 12 for the button
SERVO_CHANNEL = 0   # Servo on channel 0 of the PCA9685

PRESS_CONFIRM_MS = 30
RELEASE_CONFIRM_MS = 30
POLL_MS = 5

SERVO_POSITIONS = [0, 45, 90, 135, 180]
current_position_index = 0

def stable_read(pin, expected_level, hold_ms):
    samples_needed = max(1, hold_ms // POLL_MS)
    for _ in range(samples_needed):
        if GPIO.input(pin) != expected_level:
            return False
        time.sleep(POLL_MS / 1000.0)
    return True

def main_loop(servo):
    global current_position_index

    # Wait for the button to be in a stable, released (HIGH) state
    while not stable_read(BTN_PIN, GPIO.HIGH, RELEASE_CONFIRM_MS):
        time.sleep(POLL_MS / 1000.0)

    while True:
        # Check for a stable press (LOW)
        if GPIO.input(BTN_PIN) == GPIO.LOW and stable_read(BTN_PIN, GPIO.LOW, PRESS_CONFIRM_MS):
            current_position_index = (current_position_index + 1) % len(SERVO_POSITIONS)
            target_angle = SERVO_POSITIONS[current_position_index]
            servo.angle = target_angle
            print(f"Button pressed! Moving servo to: {target_angle}°")

            # Wait here until the button is released
            while not (GPIO.input(BTN_PIN) == GPIO.HIGH and stable_read(BTN_PIN, GPIO.HIGH, RELEASE_CONFIRM_MS)):
                time.sleep(POLL_MS / 1000.0)
        
        time.sleep(POLL_MS / 1000.0)


if __name__ == '__main__':
    try:
        # Force a cleanup of any previous state before starting
        GPIO.cleanup()
        
        # -- GPIO Setup --
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(BTN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # -- ServoKit Setup --
        i2c = busio.I2C(board.SCL, board.SDA)
        kit = ServoKit(channels=16, i2c=i2c)
        servo = kit.servo[SERVO_CHANNEL]

        print("Project 05: Interactive Servo Control")
        print("Press the button to cycle the servo through its positions.")
        print("Press Ctrl-C to exit.")

        # Set servo to initial position
        initial_angle = SERVO_POSITIONS[current_position_index]
        servo.angle = initial_angle
        print(f"Servo initialized to: {initial_angle}°")

        # Start the main loop
        main_loop(servo)

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:
        # Final cleanup when the program exits
        GPIO.cleanup()
        print("GPIO cleaned up and program exited.")
```

### Key Learnings

* Integrating Multiple Libraries: Successfully managed two different hardware paradigms in one script: direct Jetson.GPIO for the button and adafruit_servokit for I2C communication.

* Event-Driven Robotics: This is the first project where the robot's action is a direct response to an external user event, a core concept in robotics and automation.

* State Management: The script successfully tracks the servo's state (current_position_index) to make intelligent decisions about its next movement.


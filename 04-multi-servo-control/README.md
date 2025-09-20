# Project 04: Multi-Servo Control & Hardware Debugging

**Goal:** o control multiple servo motors in synchronized motion using a PCA9685 PWM driver. This project evolved into a deep dive into systematic hardware debugging, from verifying I2C communication to diagnosing and solving a critical power delivery failure.
### Hardware Used

* NVIDIA Jetson Orin Nano
* AZDelivery PCA9685 16-Channel Servo Driver
* 2x 15kg 180° Digital Servo Motors
* External 5V 2A Power Supply
* HW-131 Breadboard Power Module (used as a barrel jack adapter)
* Jumper Wires & Breadboard

### The Circuit

This circuit involves two separate power systems: one for the Jetson and PCA9685's logic, and a separate, high-current system for the servo motors.

#### 1- Control Circuit (I2C):

* Jetson 5V (Pin 2) → PCA9685 VCC

* Jetson SDA (Pin 3) → PCA9685 SDA

* Jetson SCL (Pin 5) → PCA9685 SCL

* Jetson GND (Pin 6) → PCA9685 GND

#### 2- Motor Power Circuit:

* The external 5V 2A power supply is connected to the HW-131 module.
* Jumper wires run from the HW-131's input pins to the breadboard's power rails.
* The breadboard's power rails are connected to the PCA9685's green screw terminal (V+ and GND).

#### 3- Servo Connections:

* Servo 1 is connected to Channel 0 on the PCA9685.
* Servo 2 is connected to Channel 1 on the PCA9685.

<img width="643" height="933" alt="Screenshot 2025-09-20 at 16 33 25" src="https://github.com/user-attachments/assets/9f8ead56-5dc7-448f-ba9d-9d3e8369dcb7" />


### Watch the video '04-multi-servo-control'

https://youtu.be/bdHVsnsglB8

### The Code
This Python script uses the adafruit_servokit library to control two servos simultaneously, sweeping them back and forth in sync. It explicitly defines the I2C bus to ensure compatibility with the Jetson platform.

```python
import time
import board
import busio
from adafruit_servokit import ServoKit

NUM_CHANNELS = 16
SERVO1_CHANNEL = 0
SERVO2_CHANNEL = 1
SERVO_MIN_ANGLE = 0
SERVO_MAX_ANGLE = 180
SWEEP_DELAY = 0.01

def main():
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        
        kit = ServoKit(channels=NUM_CHANNELS, i2c=i2c)
        
        servo1 = kit.servo[SERVO1_CHANNEL]
        servo2 = kit.servo[SERVO2_CHANNEL]
        
        print("Starting servo sweep. Press Ctrl+C to exit.")

        while True:
            for angle in range(SERVO_MIN_ANGLE, SERVO_MAX_ANGLE + 1):
                servo1.angle = angle
                servo2.angle = angle
                time.sleep(SWEEP_DELAY)

            for angle in range(SERVO_MAX_ANGLE, SERVO_MIN_ANGLE - 1, -1):
                servo1.angle = angle
                servo2.angle = angle
                time.sleep(SWEEP_DELAY)

    except (IOError, ValueError) as e:
        print(f"Error communicating with PCA9685: {e}")
        print("Please check I2C connections and ensure the PCA9685 is powered.")
    except KeyboardInterrupt:
        print("\nProgram interrupted.")
    finally:
        if 'kit' in locals():
            servo1.angle = 90
            servo2.angle = 90
        print("Exiting.")

if __name__ == '__main__':
    main()
```

### Key Learnings
This project was a masterclass in real-world hardware troubleshooting.

* Multi-Actuator Control: Learned to use the PCA9685 to easily control multiple servos over a single two-wire I2C bus, a critical skill for scaling robotics projects.

* Systematic Debugging: Diagnosed a non-working system by methodically testing each component in the chain:

  1. Used i2cdetect to confirm the software-to-hardware communication link was working.

  2. Used the "servo twitch test" to bypass the PCA9685 and isolate the problem to the power delivery system.

  3. Used a simple LED circuit to finally identify the root cause: a poor physical connection.

* Importance of Solid Connections: Discovered that the thin pins on some modules (like the HW-131's DC jack) make unreliable breadboard connections for high-current applications. Motors need a solid, low-resistance path for power.

* Resourceful Engineering: Solved the physical connection issue with a practical, hands-on fix using cable tie clamps to ensure the jumper wires maintained firm contact with the power input pins.


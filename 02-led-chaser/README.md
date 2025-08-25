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
| 1   | Pin 12       |
| 2   | Pin 33       |
| 3   | Pin 29       |
| 4   | Pin 31       |
| 5   | Pin 32       |
| 6   | Pin 36       |

### Challenges & Learnings

The main challenge in this project was a hardware bug. Initially, all 6 LEDs would turn on dimly at the same time as soon as the Jetson was powered on, and the script had no effect.

After simplifying the circuit to two LEDs, the problem persisted. This indicated a fault with a **shared connection**. I discovered that I had accidentally connected the breadboard's main ground rail to a **3.3V power pin** instead of a **GND pin**. This created a small, constant current path through all the LEDs.

Once this single ground wire was moved to the correct **Pin 6 (GND)**, the circuit behaved exactly as expected.

### Final Working Code

```python

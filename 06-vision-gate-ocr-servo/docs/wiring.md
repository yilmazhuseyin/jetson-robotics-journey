# Wiring (PCA9685 + 2 Servos)

- **Board**: e.g., Jetson Orin Nano (I2C pins: SCL/SDA)
- **I2C**: Jetson SCL → PCA9685 SCL, Jetson SDA → PCA9685 SDA, 3V3 → VCC, GND → GND
- **Servos**: Signal wires to PCA9685 CH0 (Left) and CH1 (Right), +5–6V servo PSU to V+ and GND on servo bank.
- **Power**: Use a separate 5–6V supply for servos (2–3A), common ground with Jetson.
- **Safety**: Keep horns off hard end-stops. Tune angles in `vision_gate.py`:
  - `LEFT_CLOSED`, `RIGHT_CLOSED`
  - `OPEN_SWEEP_DEG`
  - Optional `MIN_US`, `MAX_US` if servo pulses need trimming.

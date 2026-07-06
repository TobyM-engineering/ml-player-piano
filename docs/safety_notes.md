# Safety Notes for Solenoid-Driven Piano System

This system uses high-current 12V solenoids and requires careful electrical, thermal, and mechanical safety practices.

---

## 1. Electrical Safety

### High-Current 12V Rail

* Solenoids draw large current spikes during activation.
* The 12V solenoid rail should be fused and protected.
* Individual solenoid groups should use separate fuses where possible.
* PCA9685 boards only provide logic-level PWM signals. They should never directly power the solenoids.

### MOSFET Drivers

Each solenoid channel should use:

* N-channel MOSFET driver
* Flyback diode across the solenoid
* Gate resistor, typically 100–220Ω
* Common ground between the ESP32, PCA9685 boards, and MOSFET driver circuits

These parts help protect against:

* Voltage spikes
* MOSFET overheating
* PCA9685 damage
* Electrical noise from solenoid switching

---

## 2. Thermal Protection

* A temperature sensor monitors the MOSFET / power electronics area.
* Firmware can reduce power or disable outputs if temperature rises too high.
* Long-duration testing should include temperature monitoring, especially when many notes are played quickly.

---

## 3. Stuck-Note Protection

The firmware includes safety behavior for:

* Maximum pulse-width limits
* Automatic note-off enforcement
* Stuck-note detection
* PCA9685 reinitialization if needed
* Manual panic / all-notes-off control

This helps prevent solenoids from staying energized too long and reduces the risk of damage to the keys, MOSFETs, or solenoids.

---

## 4. Mechanical Safety

* Solenoids should be aligned so each plunger strikes the key mechanism cleanly.
* Couplers should be lightweight to reduce stress on the piano action.
* Solenoids should not be held on continuously, since most small solenoids are not designed for 100% duty cycle.
* Mechanical testing should start with low power and short pulses before increasing force.

---

## 5. Power and Signal Layout

* The ESP32 runs from regulated 5V / 3.3V power.
* The solenoid rail and logic rail should be separated as much as possible.
* Solenoid ground returns should be thick, short, and routed carefully.
* The PCA9685 boards use a dedicated I²C bus to reduce noise-related issues.
* Signal wires should be kept away from high-current solenoid wiring when possible.

---

## 6. Recommended Testing Procedure

1. Test PCA9685 boards with LEDs before connecting solenoids.
2. Test one solenoid at a time.
3. Confirm pulse width and PWM behavior.
4. Monitor temperature during repeated activations.
5. Add solenoids in small groups while checking current draw.
6. Test panic / all-notes-off behavior before running full songs.
7. Run long-duration tests while monitoring heat and stuck-note behavior.

---

Following these safeguards helps improve reliability and reduces the risk of electrical, thermal, or mechanical damage during operation.

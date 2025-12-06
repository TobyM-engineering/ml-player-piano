
# Safety Notes for Solenoid-Driven Piano System

This system uses high-current 12V solenoids and must follow strict safety practices.

---

## 1. Electrical Safety

### High Current Rail
- Solenoids draw large current spikes.
- A fused 12V rail must be used (10–20A fuse).
- PCA9685 boards only provide logic-level signals — **never** power solenoids from PCA boards.

### MOSFET Drivers
Each solenoid uses:
- N-channel MOSFET
- Flyback diode (1N4007 or SS14)
- Gate resistor (100–220Ω)

These prevent:
- Voltage spikes
- MOSFET overheating
- PCA board damage

---

## 2. Thermal Protection
- A DS18B20 or MICREEN temperature sensor monitors MOSFET bank temperature.
- Firmware throttles power if temperature rises above safe limits.
- Logging can be added to track long-term temperature trends.

---

## 3. Stuck-Note Protection
The firmware implements:
- Maximum pulse width limitation
- Automatic "note off" enforcement
- PCA re-initialization if a channel latches
- Manual panic button

This prevents physical damage to keys and solenoids.

---

## 4. Mechanical Safety
- Solenoids must be aligned so the plunger strikes the key vertically.
- Use lightweight couplers to reduce wear on the key mechanism.
- Avoid sustained key pressing — solenoids are not designed for 100% duty cycle.

---

## 5. Power Isolation
- ESP32 runs from regulated 5V/3.3V.
- PCA boards share an isolated I²C bus to avoid noise from the solenoid rail.
- Keep solenoid ground return traces thick and short.

---

## 6. Recommended Testing Procedure
1. Test PCA boards with LEDs before connecting solenoids.  
2. Run one solenoid at a time.  
3. Confirm temperature readings.  
4. Test pulse-width scaling with MIDI.  
5. Add solenoids in small batches and monitor current draw.  

---

Following these safeguards ensures reliable long-term operation and reduces risk of electrical or mechanical damage.

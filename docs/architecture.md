# ML-Assisted Player Piano — System Architecture

This document provides a technical overview of the ML-assisted player piano system, including hardware layout, firmware modules, dataflow, safety architecture, and the velocity-mapping process used to create more human-like key dynamics.

---

# 1. Hardware Architecture

## 1.1 Overview
The system retrofits an early-1900s pneumatic player piano—which originally used vacuum bellows and foot-pedal air pressure—to a modern electromechanical design using:

- ESP32 microcontroller  
- PCA9685 PWM expander boards  
- MOSFET solenoid drivers  
- 12V linear-actuating solenoids  
- Temperature sensors, potentiometers, switches  
- OLED UI screen

The original pneumatic system was heavily damaged and non-functional when acquired, so the entire actuation mechanism was replaced with a new solenoid-based architecture while preserving the piano's acoustic and mechanical integrity.

---

## 1.2 Microcontroller (ESP32)
The ESP32 serves as the central controller and is responsible for:

- BLE-MIDI reception  
- USB-MIDI forwarding from the control panel  
- MIDI event parsing and scheduling  
- Solenoid velocity calculation  
- PCA9685 PWM commands  
- Temperature+safety monitoring  
- UI rendering on the OLED display  
- Reading knobs, buttons, and mode switches  

It uses dual I²C buses:  
- **I²C-A:** OLED, temperature sensor  
- **I²C-B:** All PCA9685 boards (isolated for noise + reliability)

---

## 1.3 Solenoid System
Each piano key is actuated by a 10–12mm stroke 12V solenoid mounted under the key’s lever arm.  
Specifications:

- ~88 key solenoids  
- 1–1.5A peak current per activation  
- MOSFET switch per solenoid  
- PCA9685 PWM modulation per channel  
- Sustain pedal driven using a high-force dual-solenoid assembly  

PWM values correspond to striking strength and are computed from a tuned velocity-mapping table. Machine-learning experiments and decibel measurements were used to help inform and improve this mapping.

---

## 1.4 PCA9685 PWM Driver Boards
- 6 boards, 16 channels each (96 total channels)  
- Independent frequency control  
- Low-jitter PWM for consistent key strikes  
- Dedicated 12V → 5V logic regulation  
- Separated ground planes to minimize noise

Channel mapping includes:

- 0–87: Piano keys  
- 88: Sustain solenoid  
- Optional: Soft pedal, Sostenuto (future expansion)

---

## 1.5 Sensors & Inputs
- **DS18B20 temperature sensor:** monitors MOSFET/power rail thermal load  
- **2× potentiometers:** left-hand and right-hand volume scaling  
- **Mode selector:** LEFT / RIGHT / BOTH routing for MIDI channels  
- **Octave shift selector:** ±3 octaves  
- **Panic/reset switch:** immediately disables all PWM channels  
- **OLED UI display:** system status, temperature, settings  

All controls are mounted in the area formerly used by the original pneumatic control levers.

---

## 1.6 Power System
- **Primary rail:** 12V 100A supply (solenoids)  
- **Logic rail:** 5V/3.3V regulators for ESP32 and sensors  
- **Per-board fuse banks** for electrical fault isolation  
- **Automatic derating** when temperature rises  

Ground reference is unified between ESP32 and solenoid drivers for clean signal integrity.

---

# 2. Firmware Architecture

The firmware is modular C++ for PlatformIO, separated into four main units:

---

## 2.1 `main.cpp` — System Initialization & Control Loop
Responsibilities:

- Boot sequencing & hardware initialization  
- I²C bus configuration  
- BLE MIDI service startup  
- Periodic tasks:
  - Temperature polling  
  - Safety supervision  
  - MIDI buffer draining  
  - OLED UI updates  
  - Solenoid scheduler advancement  
- Handling user inputs (knobs, switches, octave shift)

---

## 2.2 `midi_handler.cpp` — MIDI Parser + Scheduler
Processes incoming MIDI events:

- BLE MIDI packet decoding  
- USB MIDI forwarding from control panel  
- Velocity extraction  
- Note-On / Note-Off routing to left/right/both hands  
- Predictive scheduling for overlapping and repeated notes  
- Handling "velocity zero" as Note-Off  
- Sustain pedal state management  

### **Predictive Scheduling Engine**
Because solenoids require:
- startup time  
- momentum generation  
- fall/recovery time  

…each note event is scheduled *in the future* relative to its MIDI timestamp.

The scheduler accounts for:

- Solenoid activation delays  
- Repeated-note edge cases  
- Key bounce behavior  
- Multi-note timing alignment  
- Universal key delay (max actuation latency)  
- ML-informed / hand-tuned velocity → PWM mappings

This ensures the piano plays rhythmically accurate and synchronized with the incoming MIDI file.

---

## 2.3 `pwm_controller.cpp` — PCA9685 PWM Output Engine
Converts desired key dynamics into electrical drive values.

Core functions:

- Duty cycle + pulse width calculation
- Tuned velocity scaling informed by testing and ML experiments
- Temperature-aware derating  
- Simultaneous multi-key strike management  
- MOSFET-safe activation timing  
- Sustain pedal PWM control  

---

## 2.4 `safety_logic.cpp`
Safety systems include:

- Stuck-note detection  
- PCA9685 communication watchdog  
- Overtemperature slowdown or shutdown  
- Power-rail load limiting  
- Global panic routine  
  - Instantly shuts off all PCA outputs  
  - Resets PWM driver states  
  - Clears MIDI queue  

The firmware is designed so hardware failure **never leads to runaway solenoids**.

---

3. Machine Learning / Velocity Mapping Architecture

Machine learning was used as an offline development tool to help understand how MIDI velocity, solenoid power, pulse duration, and real acoustic loudness relate to each other.

The ESP32 does not run a live ML model in real time. Instead, the final firmware uses tuned velocity-mapping logic that was developed through testing, measurement, and ML-assisted analysis.

MIDI velocity alone is extremely limited and often creates a robotic sound. Humans vary force, timing, and touch, so solenoids need correction to imitate more natural dynamics.

---

## 3.1 Data Collection
To build and improve the velocity mapping:

Individual solenoids were tested at different PWM values
Pulse widths and duty cycles were adjusted
Acoustic loudness was compared across different keys
Timing, strike strength, and recovery behavior were tuned

This helped show how electrical drive energy translates into real piano dynamics.

---

## 3.2 Model Goals
The ML-assisted analysis was used to help:

Improve MIDI velocity → PWM mapping
Reduce uneven loudness between keys
Smooth out harsh jumps between velocity levels
Make chords sound more balanced
Reduce robotic-sounding playback
Create a more expressive final mapping for the firmware

---

## 3.3 Model Pipeline
The machine-learning work was used as an offline tuning process:

Collect test data from key strikes
Compare PWM, pulse duration, and perceived loudness
Test regression-style mappings
Use the results to improve the final velocity table
Implement the final mapping logic in the ESP32 firmware
---

# 4. System Control Flow

```
BLE / USB MIDI → midi_handler → scheduler
                    ↓
          velocity mapping (ML or table)
                    ↓
        pwm_controller → PCA9685 → MOSFET → Solenoids → Piano Keys
                    ↓
         Acoustic sound output (88-key upright)

Temperature sensor → safety_logic → derating / shutoff
User controls → main loop → UI + mode routing
---

# 5. Summary
This project transforms a damaged early-1900s pneumatic player piano into a modern, ML-assisted electromechanical instrument. The architecture blends:

Embedded systems engineering
Real-time MIDI processing
Velocity mapping
Hardware actuation
Music technology

The result is a self-playing piano system designed to sound more expressive and less robotic by tuning how digital MIDI velocity is translated into physical key motion.


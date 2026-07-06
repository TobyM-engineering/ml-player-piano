<p align="center">
  <img src="media/IMG_4957.jpg" width="700">
</p>

# Player Piano
<p align="center">
  <img src="https://img.shields.io/badge/Platform-ESP32-blue?style=flat-square">
  <img src="https://img.shields.io/badge/Firmware-C++-brightgreen?style=flat-square">
  <img src="https://img.shields.io/badge/Hardware-PCA9685%20%2B%20MOSFETs-orange?style=flat-square">
  <img src="https://img.shields.io/badge/Machine%20Learning-Regression%20Model-yellow?style=flat-square">
  <img src="https://img.shields.io/badge/License-MIT-lightgrey?style=flat-square">
</p>

A full electromechanical player piano system combining embedded C++, real-time MIDI processing, PCA9685-controlled solenoids, and a machine-learning velocity model trained on decibel measurements to produce human-like dynamics. Built to retrofit an early-1900s pneumatic piano into a modern expressive instrument.

> ⚠️ This repository is not fully documented yet — more diagrams, notes, and explanations will be added soon.  
> 🎥 See it in action below!

---

# 🎥 Demo Videos

- **[Demo 1](media/IMG_6648%20%281%29.mov)** – A quick demo showing that I can still play the piano manually while the system plays notes at the same time — a human + machine duet.

- **[Demo 2](media/IMG_6644.MOV)** – Full-speed performance of the intro to *Moonlight Sonata*, showing the entire piano as the system plays accurately and smoothly.

- **[Demo 3](media/IMG_6645.MOV)** – Close-up of the solenoids firing during *Moonlight Sonata*, highlighting actuator precision and timing.

---

# 📸 System Photos

### **Internal View of the System**
<img src="media/IMG_6649.jpg" width="600">
A look inside the instrument — solenoid rails, wiring bundles, and PCA9685 distribution. (Yes, it's a nest of wires — it wasn’t designed for beauty)

### **Embedded OLED + Control Interface**
<img src="media/IMG_4962 (1).jpg" width="600">
This screen and control interface sits where the original player-piano controls used to be.  
Since the 1900s pneumatic player mechanism was destroyed and partially burned, I replaced the entire control cluster with modern electronics: buttons, potentiometers, and switches — all mounted into the original wood frame.

### **Front Control Panel Under the Keys**
<img src="media/IMG_4964.jpg" width="600">
These knobs and switches replace the old pneumatic roll levers.  
They now control volume, mode selection (Left hand /Both hands /Right hand), octave shift, system power, and volume. (it wasn’t designed for beauty)

---

# 🎹 Project Background

This piano originally used a **vacuum-driven pneumatic system** built in the early 1900s — powered by foot pedals that pulled air through a paper roll.  
When I got it from OfferUp, the system was:

- non-functional  
- missing tubing and bellows  
- partially burned  
- too heavy to move without fully disassembling it
- $20

I rebuilt the piano structurally, but the original player system was beyond repair.  
So instead, I designed and built a **modern, fully electronic self-playing system**, turning an antique into a hybrid electromechanical instrument.

This project combines:
- Embedded systems engineering  
- Real-time MIDI processing  
- Machine-learning velocity mapping  
- Hardware and electrical design  
- Large-scale actuator control (88 solenoids)

Anyone is welcome to clone or fork this project to learn from it or build their own version.

---

# 🛠 System Overview

## Hardware
- ~88 × 12V solenoids  
- 6 × PCA9685 16-channel PWM drivers  
- 88 MOSFET channels  
- ESP32 MCU (BLE MIDI + control loop)  
- DS18B20 temperature sensor  
- I²C OLED display  
- Volume knobs + mode switch + panic/reset  
- 100A 12V power rail  

Hardware diagrams: **`hardware/` folder**

---

## Firmware Modules

| File | Purpose |
|------|---------|
| `main.cpp` | System init, UI, BLE MIDI, control loop |
| `midi_handler.cpp` | MIDI parsing + predictive scheduling |
| `pwm_controller.cpp` | Solenoid PWM + pulse-width control |
| `safety_logic.cpp` | Overheat detection, stuck-note recovery, panic mode |

Entry point: **`firmware/main.cpp`**

---
## The Coolest Part: Why Machine Learning Is Needed for Human-Like Piano Dynamics

One of the hardest parts of building a DIY player piano is avoiding the robotic sound that happens when the system simply reads MIDI velocity values and triggers solenoids.

MIDI only tells the system two main things about a note:

* Which key to play
* How fast the key should be played, from velocity 1–127

But MIDI velocity is not the same as real hammer force, and it does not directly describe the true sound level of a physical piano.

If MIDI velocity is directly mapped to solenoid power, the piano can sound stiff, uneven, or mechanical.

This is where machine learning becomes useful.

Each solenoid and piano key responds slightly differently because of friction, key weight, spring tension, hammer behavior, and mechanical differences across all 88 keys. That means one universal formula does not work perfectly for every note.

A more advanced version of the system would collect data from each key, such as:

* Solenoid ID
* PWM duty cycle
* Pulse duration
* Measured sound level in dB
* Original MIDI velocity

This data could be used to train a model that learns how each individual key responds to different power levels.

The relationship between MIDI velocity, solenoid force, hammer speed, and actual loudness is not perfectly linear. A human pianist naturally adjusts finger pressure, timing, and touch to make notes sound musical. A raw MIDI file does not contain all of those details.

A machine learning layer could help by learning:

* How hard each solenoid should strike for a desired loudness
* How to smooth out jumps between soft and loud notes
* How to balance loudness across all 88 keys
* How to make chords sound more even
* How to make crescendos and phrasing feel less robotic

Instead of directly converting MIDI velocity into solenoid power, the system could use a learned correction table that the ESP32 reads in real time.

The goal is to transform basic MIDI data into more expressive, natural-sounding piano dynamics.

In short, machine learning is useful because it can help bridge the gap between digital MIDI instructions and the physical behavior of a real piano.

---

# 🚀 Getting Started

## 1. Clone the repo
```bash
git clone https://github.com/TobyM-engineering/ml-player-piano.git


2. Open the firmware

Use:

VS Code + PlatformIO (recommended)
or

Arduino IDE with libraries: PCA9685, OneWire, DallasTemperature, MIDI

Main file: firmware/main.cpp

3. Flash the ESP32

Select your board

Choose correct COM port

Upload firmware

Open Serial Monitor to verify MIDI events

⚠️ Safety Warning

Driving ~100 solenoids from a high-current 12V supply can be dangerous.
Before powering anything, read:

👉 docs/safety_notes.md

📌 Repo Status

✅ Project structure + documentation

✅ Firmware modules implemented

⚙️ ML velocity mapping in progress

🔧 Hardware stable + long-duration testing


ml-player-piano/
│
├── firmware/
│   ├── main.cpp
│   ├── midi_handler.cpp
│   ├── pwm_controller.cpp
│   └── safety_logic.cpp
│
├── docs/
│   ├── architecture.md
│   ├── midi_system.md
│   └── safety_notes.md
│
├── hardware/
│   ├── wiring_diagram.png
│   ├── pca9685_layout.png
│   └── solenoid_driver_schematic.jpg
│
├── ml-model/
│   ├── model_training.ipynb
│   ├── trained_model.pkl
│   └── data/
│       ├── sample_pressures.csv
│       └── sample_velocities.csv
│
├── media/
│   ├── IMG_6648 (1).mov
│   ├── IMG_6644.MOV
│   └── IMG_6645.MOV
│
└── README.md

# ML Player Piano

Machine-learning–enhanced 88-key player piano using an ESP32, six PCA9685 PWM driver boards, and high-current solenoids.

The ESP32 receives MIDI (BLE / serial), parses the stream, and drives one solenoid per piano key. A temperature sensor and safety logic monitor the power rail so the system can run for long periods without cooking coils or hardware.

> ⚠️ This repo is **work in progress**. Firmware is actively evolving along with the mechanical design.

---

## Features

### 88-key solenoid control

- ESP32 controls up to **88 keys + sustain pedal**
- Six PCA9685 boards (16 channels each) over I²C
- Per-key PWM pulse control for dynamic strike force

### MIDI input

- Receives MIDI over Bluetooth (e.g., Synthesia on tablet / laptop)
- Support for standard **Note On / Note Off / Velocity**
- Channel-agnostic parsing (handles any MIDI channel)
- Left / right hand routing and volume split (planned / partly implemented)

### Real-time control UI (OLED + switches)

- Dual volume controls (left / right hand) via potentiometers
- Play-mode switch: `LEFT`, `BOTH`, `RIGHT`
- Octave-shift switch with on-screen indicator
- Uptime + Bluetooth status on an I²C OLED

### Safety and reliability

- Temperature monitoring of the solenoid power rail (DS18B20/MICREEN)
- Global **panic button** for “all notes off”
- Duty-cycle limiting to avoid overheating solenoids
- Power-on reset that forces all PCA9685 outputs low

### ML-assisted velocity mapping (in progress)

- Collects sample data of key press force vs. dB level
- Trains a regression model to map MIDI velocity → PWM pulse width
- Separate datasets for **pressure tests** and **velocity mapping**

---

## Repository structure

```text
ml-player-piano/
│
├── README.md
├── LICENSE
├── .gitignore
│
├── firmware/                 # Embedded firmware for the ESP32
│   ├── main.cpp              # Core control loop, BLE, UI, temp & safety
│   ├── midi_handler.cpp      # MIDI parsing, buffering, predictive timing
│   ├── pwm_controller.cpp    # Velocity → PWM / solenoid pulse control
│   └── safety_logic.cpp      # Panic handling, PCA recovery, stuck-note protection
│
├── ml-model/                 # ML velocity mapping experiments
│   ├── model_training.ipynb  # Jupyter notebook for training models
│   ├── trained_model.pkl     # Saved regression model (placeholder / WIP)
│   └── data/
│       ├── sample_pressures.csv   # Example force / dB measurements
│       └── sample_velocities.csv  # Example velocity → pulse-width samples
│
├── hardware/                 # Hardware drawings and schematics
│   ├── wiring_diagram.png
│   ├── pca9685_layout.png
│   └── solenoid_driver_schematic.jpg
│
├── media/                    # Photos / demo videos
│   ├── piano_photo.jpg
│   ├── demo1.mp4
│   └── demo2.mp4
│
└── docs/                     # Design documentation
    ├── architecture.md
    ├── midi_system.md
    └── safety_notes.md

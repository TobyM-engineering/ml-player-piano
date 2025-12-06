# ML Player Piano

Machine-learning–enhanced 88-key player piano using an ESP32, PCA9685 driver boards, solenoids, and predictive velocity control.  
The system reads BLE-MIDI from a tablet/computer and drives each piano key with a dedicated solenoid, while monitoring temperature and power for safe long-term operation.

---

## Features

- **88-key solenoid control**  
  - ESP32 + six PCA9685 boards driving up to 88 keys + sustain pedal  
  - Per-key PWM control for dynamic strike force

- **BLE-MIDI input**  
  - Receives MIDI over Bluetooth from apps like Synthesia  
  - Channel-based left/right hand routing and volume split

- **Real-time control UI (OLED)**  
  - Dual volume controls (left/right hand) with smooth ADC filtering  
  - Play-mode switch: `LEFT`, `BOTH`, `RIGHT`  
  - Octave shift indicator with 7-step UI  
  - Uptime and Bluetooth status display

- **Safety & reliability**  
  - Panic button: hardware-level “all notes off” and PCA re-init  
  - Temperature sensor (DS18B20 / MICREEN) with visual warnings  
  - Automatic timeouts for stuck notes and I²C recovery for PCA boards

- **ML-assisted velocity mapping (work in progress)**  
  - Collects sample data of key press force vs. dB level  
  - Uses regression models to map MIDI velocity → PWM pulse width  
  - Separate datasets for pressure and velocity tuning

---

## Repository structure

```text
ml-player-piano/
├── firmware/
│   ├── main.cpp           # Core control loop, BLE-MIDI, UI, temperature & safety logic
│   ├── midi_handler.cpp   # MIDI parsing, buffering, predictive note timing
│   ├── pwm_controller.cpp # Velocity → PWM and solenoid pulse control
│   └── safety_logic.cpp   # Panic handling, PCA recovery, stuck-note protection
│
├── ml-model/
│   ├── model_training.ipynb   # Jupyter notebook for training velocity models (planned)
│   ├── trained_model.pkl      # Saved regression model (planned)
│   └── data/
│       ├── sample_pressures.csv   # Example force/dB measurements
│       └── sample_velocities.csv  # Example velocity → pulse-width samples
│
├── hardware/
│   └── (diagrams coming soon: wiring, PCA layout, power distribution)
│
└── media/
    └── (demo photos / videos of the piano in action)

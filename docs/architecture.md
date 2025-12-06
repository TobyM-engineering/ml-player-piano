# ML Player Piano — System Architecture

This document provides an overview of the entire system architecture behind the ML-enhanced player piano, covering hardware layout, firmware modules, dataflow, and planned machine-learning components.

---

## 1. Hardware Architecture

### **Microcontroller**
- ESP32 (BLE + dual I²C support)
- Handles MIDI input, UI drawing, sensor readings, PWM output generation

### **Solenoid Control System**
- 6× PCA9685 16-channel PWM driver boards  
- Each PCA board controls up to 16 solenoids  
- ~88 keys + sustain pedal  
- MOSFET driver boards used to switch 12V solenoid power

### **Sensors and Inputs**
- DS18B20/MICREEN temperature sensor (monitors power rail temp)
- 2× potentiometers for left/right hand volume
- Mode switch (LEFT / BOTH / RIGHT)
- Octave shift switch (+3 to –3)
- Panic button for global “all notes off”
- OLED I²C display for UI

### **Power Distribution**
- 12V high-current supply for solenoids
- ESP32 powered through regulated 5V/3.3V supply
- PCA boards share a separate I²C bus for electrical isolation

---

## 2. Firmware Architecture

The firmware is split into four main modules:

### **`main.cpp`**
- System initialization (I²C, BLE, drivers, sensors)
- Reads analog inputs & updates UI
- Calls periodic tasks:
  - Temperature update
  - Safety checks
  - MIDI buffer processing
  - Solenoid control update loop

### **`midi_handler.cpp`**
- BLE MIDI server implementation
- Parses Note On/Off messages
- Channel-based left/right hand routing
- Predictive note scheduling for rapid repeats
- Applies octave shift and per-hand volume scaling

### **`pwm_controller.cpp`**
- Maps MIDI velocity → PWM duty cycle and solenoid pulse width
- Maintains lookup tables for:
  - Pulse duration
  - Solenoid recovery time
- Handles simultaneous multi-key strikes
- Supports active-low MOSFET logic

### **`safety_logic.cpp`**
- Monitors for stuck notes
- Detects unresponsive PCA boards and reinitializes them
- Temperature-based derating (lower PWM when hot)
- Implements global panic routine:
  - Immediately disables all PWM channels
  - Reinitializes all PCA devices
  - Clears MIDI event queue

---

## 3. ML Model Architecture

### **Data Sources**
- `sample_pressures.csv` — measurements of force/dB vs solenoid pulse width  
- `sample_velocities.csv` — MIDI velocity vs required pulse width

### **Model Goals**
- Predict solenoid pulse duration needed to strike a note at a desired volume
- Produce smoother dynamics than simple linear mapping
- Reduce mechanical stress by optimizing recovery windows

### **Pipeline**
1. Load datasets inside `model_training.ipynb`
2. Visualize outliers & clean data
3. Fit regression model:
   - Linear
   - Polynomial
   - Gradient boosting (future)
4. Export mapping table or `.pkl` model
5. Convert model output into:
   - C++ lookup table  
   - Or embedded model inference (future ESP32 version)

---

## 4. Control Flow Summary

```text
BLE MIDI → midi_handler → event buffer
              ↓
PWM mapping ← pwm_controller ← ML model (optional)
              ↓
PCA9685 drivers → MOSFET boards → Solenoids → Piano keys

Sensors → main loop → UI + safety logic

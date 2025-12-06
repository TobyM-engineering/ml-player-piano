# MIDI System Overview

This document explains how the ML Player Piano receives, parses, and routes MIDI data.

## 1. BLE-MIDI Input
- ESP32 receives MIDI over Bluetooth from apps like Synthesia or a computer.
- MIDI messages are processed in real time with minimal latency.

## 2. Supported MIDI Messages
- **Note On (0x90)**  
  - Velocity is mapped to PWM pulse width.
- **Note Off (0x80)**  
  - Immediately releases the corresponding solenoid.
- **Channel messages**  
  - Piano splits left/right hand based on MIDI channel.
  - CH1 = Left hand  
  - CH2 = Right hand

## 3. Predictive Timing Buffer
The firmware uses a small FIFO buffer to:
- Smooth inconsistent BLE packet timing
- Prevent dropped notes during fast passages
- Maintain correct ordering of overlapping notes

## 4. Velocity Processing
Before routing to the solenoid:
1. Velocity is normalized (0–127).
2. If ML model is loaded:
   - Velocity is passed into the regression model.
3. Output pulse width (µs) is computed.

## 5. Error Handling
- Stuck-note detection
- Panic mode (global all-notes-off)
- Automatic buffer flush on BLE disconnect

## 6. Firmware Modules Involved
- `midi_handler.cpp` → Parses MIDI
- `pwm_controller.cpp` → Converts velocity → PWM
- `safety_logic.cpp` → Stuck note protection

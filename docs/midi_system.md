# MIDI System Overview

This document explains how the ML-assisted player piano receives, parses, and routes MIDI data.

## 1. BLE-MIDI Input

* ESP32 receives MIDI over Bluetooth from apps like Synthesia or a computer.
* MIDI messages are processed in real time with minimal latency.

## 2. Supported MIDI Messages

* **Note On (0x90)**

  * Velocity is mapped to a PWM pulse width.
* **Note Off (0x80)**

  * Immediately releases the corresponding solenoid.
* **Channel messages**

  * Piano parts can be split by MIDI channel.
  * CH1 = Left hand
  * CH2 = Right hand

## 3. Predictive Timing Buffer

The firmware uses a small FIFO buffer to:

* Smooth inconsistent BLE packet timing
* Prevent dropped notes during fast passages
* Maintain correct ordering of overlapping notes
* Improve timing consistency for repeated notes and chords

## 4. Velocity Processing

Before routing a note to the solenoid:

1. MIDI velocity is read from 0–127.
2. The velocity is normalized and adjusted based on the selected volume settings.
3. A tuned velocity-mapping table converts the MIDI velocity into solenoid pulse strength.
4. The mapping was developed through testing, decibel measurements, and ML-assisted analysis.
5. The final output pulse width is sent to the PWM controller.

The ESP32 does not need to run a live ML model during playback. Instead, it uses the final mapping logic created from testing and tuning.

## 5. Error Handling

* Stuck-note detection
* Panic mode / global all-notes-off
* Automatic buffer flush on BLE disconnect
* Note release cleanup for repeated or overlapping notes

## 6. Firmware Modules Involved

* `midi_handler.cpp` → Parses MIDI and schedules note events
* `pwm_controller.cpp` → Converts velocity mapping into PWM output
* `safety_logic.cpp` → Handles stuck-note protection, panic mode, and safety shutdown behavior

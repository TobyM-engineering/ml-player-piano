ML Player Piano — System Architecture

This document describes the full hardware and firmware architecture of the custom ESP32-powered player piano system. It covers the electronics, solenoid control network, firmware modules, MIDI pipeline, and machine-learning calibration workflow used to drive accurate piano key actuation.

1. Hardware Architecture
Microcontroller (ESP32)

Primary controller for the entire system

Handles:

BLE MIDI input

USB MIDI (optional)

I²C communication with all PCA9685 boards

OLED UI rendering

Reading analog inputs (volume knobs, mode switches)

Running the safety loop and PWM scheduling

Solenoid Control System

6× PCA9685 16-channel PWM drivers

96 channels available

~88 piano keys + 1–2 pedal actuators used

Solenoids driven through logic-level MOSFET boards

PCA boards daisy-chained on I²C

Active-low MOSFET design (LOW = fire)

Actuators

Individual solenoid per key

Solenoids mounted below key levers using custom aluminum rod extensions

Sustain pedal actuated with a high-force solenoid

Inputs / Controls

Two volume knobs

Left hand volume

Right hand volume

Play Mode Switch

LEFT / BOTH / RIGHT

Octave Shift Switch

–3 to +3 semitone shifts

Panic Button

Instantly turns off every key

Reinitializes PCA drivers

I²C OLED Display

System info

Active mode

Current temperature

Status messages

Temperature Monitoring

DS18B20 temperature sensor mounted near MOSFET rail

Used to automatically reduce PWM when overheating begins

Power System

12V high-current supply powering all solenoids

5V/3.3V regulated supply for ESP32 + sensors

PCA boards powered separately for isolation

Shared ground across the whole system

2. Firmware Architecture

Firmware is organized into four major modules:

main.cpp

Initializes all buses (I²C, OneWire, BLE)

Boots PCA boards safely by forcing all channels OFF

Reads:

volume knobs

mode switch

octave switch

temperature sensor

Updates the OLED UI

Runs the main scheduler loop:

process MIDI messages

apply safety logic

send PWM pulses to PCA

handle note repeat timing

midi_handler.cpp

Receives BLE MIDI packets

Decodes Note On / Note Off messages

Treats all MIDI channels equally but routes notes by:

Left-hand range

Right-hand range

Applies octave shift

Applies per-hand volume control

Handles rapid sequential Note On events to avoid missed keys

pwm_controller.cpp

Converts MIDI velocity → PWM duty cycle

Applies dynamic pulse-length mapping

Implements:

startup pulse (maximum force)

velocity pulse (scaled)

hold pulse (prevents clicking)

Ensures solenoid cooldown timing

Manages simultaneous note strikes across all PCA boards

safety_logic.cpp

Automatically shuts off stuck notes

Detects unresponsive PCA boards and reinitializes them

Temperature-based derating:

progressively lowers PWM when hot

Global panic system:

disables all channels instantly

blocks incoming MIDI

resets PCA drivers

clears all event queues

3. ML Calibration Workflow

Machine learning is used to generate better mappings between MIDI velocity and solenoid pulse width.

Data Sources

Real measurements:

Solenoid force vs pulse width

Note loudness (dB) vs pulse width

Velocity vs key travel time

Workflow

Load and clean datasets inside model_training.ipynb

Fit regression models to smooth out velocity → pulse mappings

Export lookup curves

Convert final model into:

A compact C++ lookup table used in firmware

Optional on-device inference (ESP32 can run tiny models)

Purpose

Create smoother dynamic response

Reduce mechanical stress on solenoids

Improve consistency across all 88 keys

4. Control Flow Summary
BLE MIDI → midi_handler → MIDI event buffer 
                 ↓
   velocity + mapping → pwm_controller → PCA9685 drivers
                 ↓
       MOSFET boards → Solenoids → Piano keys

Temp sensor → safety_logic → PWM derating + shutdown
User inputs → main loop → UI + mode switching

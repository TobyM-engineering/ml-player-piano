ML Player Piano — System Architecture

This document describes the full hardware and firmware architecture of the custom ESP32-powered player piano system. It outlines the electronics, solenoid control network, firmware modules, MIDI event pipeline, and machine-learning calibration workflow.

1. Hardware Architecture
1.1 Microcontroller (ESP32)

Responsibilities

BLE MIDI input

USB MIDI input (optional)

I²C bus management (PCA9685 drivers + OLED display)

Reading analog controls (volume knobs, switches)

Temperature monitoring

Running safety loop + PWM scheduling

Rendering on-screen UI

1.2 Solenoid Actuation System
PCA9685 PWM Drivers

6× PCA9685 boards

16 channels each → 96 total channels

~88 piano keys + sustain pedal

Connected via daisy-chained I²C addresses

Each PCA channel outputs PWM → MOSFET gate

MOSFET Driver Boards

Logic-level MOSFETs

Active-low trigger design

Drive 12V solenoid load safely

Solenoids

One solenoid per key

Mounted under the key levers using custom rod linkages

Sustain pedal solenoid is a larger high-force actuator

1.3 User Inputs & Controls

Left Volume Knob — scales left-hand MIDI velocities

Right Volume Knob — scales right-hand MIDI velocities

Play Mode Switch — LEFT / BOTH / RIGHT

Octave Shift Switch — –3 to +3

Panic Button — instant all-notes-off + PCA reset

OLED I²C Display — modes, temperature, status

1.4 Temperature Monitoring

DS18B20 temperature sensor mounted near MOSFET rail

Firmware automatically:

reduces PWM duty cycle when hot

prevents thermal overload

1.5 Power Distribution

12V high-current supply for solenoids

5V / 3.3V regulated supply for ESP32 + sensors

PCA boards powered separately for noise isolation

Shared ground across the entire system

2. Firmware Architecture

Firmware is organized into four core modules:

2.1 main.cpp — System Core

Initializes:

I²C bus

BLE MIDI server

PCA9685 drivers (forces all channels OFF at boot)

Temperature sensing

UI display

Runtime Responsibilities

Reads analog inputs (volume knobs)

Reads mode + octave switches

Updates OLED UI

Processes queued MIDI events

Calls PWM controller

Executes safety logic

Handles solenoid scheduling loop at high rate

2.2 midi_handler.cpp — MIDI Processing

Functions

Receives BLE MIDI packets

Decodes Note On / Note Off messages

Handles running status + multi-byte packets

Aggregates all MIDI channels into a single piano bus

Routes notes based on:

left-hand note range

right-hand note range

Applies:

octave shift

per-hand volume curves

Handles rapid note repetitions without missing events

2.3 pwm_controller.cpp — Solenoid Drive Logic

Responsible for converting velocity → PWM waveforms

Includes:

Startup pulse (max force to begin motion)

Scaled velocity pulse (hit strength)

Hold pulse (continuous actuation without clicking)

Other duties

Manages cooldown time per key

Ensures safe multi-key simultaneity

Communicates final PWM signal to the correct PCA board/channel

2.4 safety_logic.cpp — Protection & Recovery

Monitors system health and prevents hardware damage.

Features

Stuck-note detection

PCA board timeout detection + auto-reinit

Temperature-based PWM derating

Solenoid overuse detection

Global panic routine:

kills all PWM outputs

blocks new MIDI

resets PCA hardware

clears note buffers

3. Machine Learning Calibration Workflow

Machine learning is used to generate smooth mappings from MIDI velocity to real solenoid behavior.

3.1 Data Sources

Force/dB vs pulse width curves

Solenoid strike velocity measurements

Recovery time vs pulse duration

Manually collected per-note calibration values

3.2 Training Pipeline

Performed inside model_training.ipynb:

Load raw measurement CSV files

Clean + normalize data

Fit models:

Linear regression

Polynomial fit

(Optional) gradient-boosting regression

Visualize predicted curves

Export:

C++ lookup tables

or serialized model for possible on-device use

3.3 Purpose of ML Calibration

Achieve smoother dynamic response

Reduce mechanical stress

Improve consistency between keys

Compensate for hardware variation

4. Control Flow Diagram
––––––––––––––––––––––––––––––––––––––––––––––––––––
   BLE MIDI → midi_handler → event buffer
                     ↓
      pwm_controller (velocity mapping)
                     ↓
         PCA9685 drivers → MOSFET boards
                     ↓
          Solenoids → Key Levers → Piano Sound
––––––––––––––––––––––––––––––––––––––––––––––––––––

Sensors → safety_logic → thermal derating / shutdown  
User Inputs → main loop → UI + mode handling

// midi_handler.cpp — MIDI parsing and scheduling module
//
// This file is reserved for the MIDI handling portion of the player piano firmware.
//
// In the current working version, this logic still lives inside main.cpp.
// It includes:
//
// - BLE-MIDI packet decoding
// - Note On / Note Off parsing
// - MIDI channel filtering for left-hand and right-hand playback
// - Octave shifting
// - Velocity scaling from the control panel
// - Predictive MIDI event buffering
// - Handling repeated notes and overlapping notes
// - Timing compensation for solenoid release and re-trigger behavior
//
// The goal of this module is to eventually separate MIDI input and scheduling
// from the main firmware loop so the system is easier to read, test, and expand.
//
// Current data flow:
//
// BLE MIDI input
//      ↓
// MIDI parser
//      ↓
// note / velocity / channel extraction
//      ↓
// predictive event buffer
//      ↓
// PWM controller
//      ↓
// PCA9685 boards + MOSFET drivers
//      ↓
// solenoids press piano keys
//
// This file is currently a documentation placeholder while the firmware is
// being refactored from the original working main.cpp implementation.

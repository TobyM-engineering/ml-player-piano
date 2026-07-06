// pwm_controller.cpp — Velocity mapping and solenoid PWM control
//
// This file is reserved for the PWM output portion of the player piano firmware.
//
// In the current working version, this logic still lives inside main.cpp.
// It includes:
//
// - MIDI velocity to PWM conversion
// - Solenoid pulse-width control
// - Per-note timing and recovery windows
// - Volume scaling from the left/right control knobs
// - Compensation for different key ranges
// - PCA9685 output writing
// - Active-low MOSFET output logic
// - Repeated writes to improve output reliability
//
// The purpose of this module is to convert musical intent into physical motion.
//
// MIDI velocity alone does not directly equal piano loudness, so the firmware
// uses tuned velocity mapping to decide how strongly each solenoid should strike.
//
// Simplified concept:
//
// MIDI velocity
//      ↓
// volume scaling
//      ↓
// tuned velocity mapping
//      ↓
// PWM duty cycle + pulse duration
//      ↓
// PCA9685 output
//      ↓
// MOSFET driver
//      ↓
// solenoid strike
//
// This file is currently a documentation placeholder while the working code
// remains inside main.cpp.

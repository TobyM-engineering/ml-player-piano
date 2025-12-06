// pwm_controller.cpp — Solenoid PWM & pulse controller
//
// Placeholder module describing how the system drives the 88 piano keys.
//
// Planned responsibilities:
//  - Map MIDI velocity → PWM duty cycle and pulse width
//  - Maintain per-key NoteState (isOn, pulseEnd, readyAt, etc.)
//  - Convert logical key indices → { PCA9685 board, hardware channel }
//  - Handle timing for fast retriggers and back-to-back notes
//  - Provide a small API such as:
//
//      void pwm_init();
//      void pwm_noteOn(uint8_t logicalKey, uint8_t velocity, float volumeFactor);
//      void pwm_update();   // called each loop to end pulses and force-off notes
//
// Currently this logic lives in main.cpp (`midi_velocityToPWM`, `noteStates[]`,
// `pcaWriteLogical`, and `midi_updatePCA`).

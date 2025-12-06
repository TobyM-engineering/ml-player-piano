// midi_handler.cpp — BLE-MIDI parsing & event buffering
//
// This file is a placeholder for a future refactor. In the current hardware
// build, all of the MIDI logic still lives in main.cpp.
//
// Planned responsibilities:
//  - Own the BLE-MIDI characteristic callbacks
//  - Decode raw BLE packets into Note On / Note Off events
//  - Apply octave shift and per-hand volume scaling
//  - Push events into a ring buffer for timing-accurate processing
//
// The idea is that main.cpp will call something like:
//
//    void midi_init();
//    void midi_onBlePacket(...);
//    void midi_processBufferedEvents();
//
// once this code is split out of main.cpp.

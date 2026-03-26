#pragma once
#include <Servo.h>
#include <stdint.h>
#include "SensorData.h"
#include "Controller.h"

// ── ServoMirror ────────────────────────────────────────────────────────────────
// Manages two RC servo channels.  Each channel reads a PWM input via interrupt,
// applies a 3-sample median filter, and drives an output servo.
//
// Passthrough (RC relay):
//   gMirror.passthrough();
//
// Autopilot (direct write to one channel):
//   gMirror.write(ServoMirror::SERVO_A, pwmUs);
//   gMirror.write(ServoMirror::SERVO_B, pwmUs);
// ─────────────────────────────────────────────────────────────────────────────

class ServoMirror {
public:
    enum Channel { SERVO_A = 0, SERVO_B = 1 };

    static constexpr uint32_t PULSE_CENTER_US = 1500;

    // Attach ISRs and output servos.
    void begin(uint8_t inPinA, uint8_t outPinA,
               uint8_t inPinB, uint8_t outPinB);

    // Wire up the controller whose updateAxisA/B will be called when engaged.
    void setController(Controller &c) { _ctrl = &c; }

    // Read median-filtered RC inputs and write to both output servos.
    void passthrough();

    // Send a specific pulse width (µs) to one output servo.
    void write(Channel ch, uint32_t us);

    // Return the latest raw (unfiltered) RC pulse width for a channel (µs).
    uint32_t getRawInput(Channel ch) const { return _pulseWidthUs[ch]; }

    // ── Per-axis control functions ─────────────────────────────────────────────
    // Called every loop while engaged.  Put your control logic for each axis
    // here and call write(SERVO_A/B, us) to drive the output.
    void controlAxisA(const SensorData &snap);   // e.g. elevator / pitch
    void controlAxisB(const SensorData &snap);   // e.g. aileron  / roll

private:
    Controller *_ctrl = nullptr;

    uint8_t _inPin[2]       = {0, 0};
    Servo   _out[2];

    volatile uint32_t _pulseStart[2]   = {0, 0};
    volatile uint32_t _pulseWidthUs[2] = {1500, 1500};

    uint32_t _med[2][3] = {{1500, 1500, 1500}, {1500, 1500, 1500}};

    void     onEdge(uint8_t ch);
    uint32_t medianOf(uint8_t ch);

    // Singleton pointer — only one ServoMirror exists; ISRs need a static target.
    static ServoMirror *_instance;
    static void isrA();
    static void isrB();
};

#include "ServoMirror.h"
#include <Arduino.h>

ServoMirror *ServoMirror::_instance = nullptr;

// ── begin ─────────────────────────────────────────────────────────────────────
void ServoMirror::begin(uint8_t inPinA, uint8_t outPinA,
                        uint8_t inPinB, uint8_t outPinB) {
    _instance = this;

    _inPin[0] = inPinA;
    _inPin[1] = inPinB;

    pinMode(inPinA, INPUT);
    attachInterrupt(digitalPinToInterrupt(inPinA), isrA, CHANGE);
    _out[0].attach(outPinA, 800, 2200);

    pinMode(inPinB, INPUT);
    attachInterrupt(digitalPinToInterrupt(inPinB), isrB, CHANGE);
    _out[1].attach(outPinB, 800, 2200);
}

// ── passthrough ───────────────────────────────────────────────────────────────
void ServoMirror::passthrough() {
    _out[0].writeMicroseconds((int)medianOf(0));
    _out[1].writeMicroseconds((int)medianOf(1));
}

// ── write ─────────────────────────────────────────────────────────────────────
void ServoMirror::write(Channel ch, uint32_t us) {
    _out[ch].writeMicroseconds((int)us);
}

// ── controlAxisA ──────────────────────────────────────────────────────────────
// Calls Controller::updateAxisA() for the pitch/elevator PD law, then drives
// the output servo.  Control logic lives in Controller.cpp — edit it there.
void ServoMirror::controlAxisA(const SensorData &snap) {
    if (!_ctrl) return;
    write(SERVO_A, _ctrl->updateAxisA(snap));
}

// ── controlAxisB ──────────────────────────────────────────────────────────────
// Calls Controller::updateAxisB() for the roll/aileron PD law, then drives
// the output servo.  Control logic lives in Controller.cpp — edit it there.
void ServoMirror::controlAxisB(const SensorData &snap) {
    if (!_ctrl) return;
    write(SERVO_B, _ctrl->updateAxisB(snap));
}

// ── onEdge (called from ISR) ──────────────────────────────────────────────────
void ServoMirror::onEdge(uint8_t ch) {
    if (digitalRead(_inPin[ch])) {
        _pulseStart[ch] = micros();
    } else {
        uint32_t w = micros() - _pulseStart[ch];
        if (w >= 800 && w <= 2200) _pulseWidthUs[ch] = w;
    }
}

// ── medianOf ──────────────────────────────────────────────────────────────────
uint32_t ServoMirror::medianOf(uint8_t ch) {
    uint32_t *m = _med[ch];
    m[0] = m[1]; m[1] = m[2]; m[2] = _pulseWidthUs[ch];
    uint32_t a = m[0], b = m[1], c = m[2];
    if (a > b) { uint32_t t = a; a = b; b = t; }
    if (b > c) { uint32_t t = b; b = c; c = t; }
    if (a > b) { uint32_t t = a; a = b; b = t; }
    return b;
}

// ── ISR trampolines ───────────────────────────────────────────────────────────
void ServoMirror::isrA() { _instance->onEdge(0); }
void ServoMirror::isrB() { _instance->onEdge(1); }

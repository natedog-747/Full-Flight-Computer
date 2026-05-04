#pragma once
#include <Servo.h>

static constexpr int   PIN_PITCH_SERVO        = 11;
static constexpr int   PIN_YAW_SERVO          = 9;
static constexpr float LAUNCH_ACCEL_THRESHOLD = 5.0f;  // m/s², mirrors FlightStateMachine

// Pitch and yaw PID attitude hold with rate damping.
//
// Pitch holds pitchTarget (default -2°, slight nose-down).
// Yaw holds the arithmetic-mean heading recorded while accelX >= LAUNCH_ACCEL_THRESHOLD
// before entry into FLIGHT, capturing the average throw direction.
//
// PID output = Kp*error + Ki*integral(error) - Kd*rate
// Servo center = 90°. Output clamped to ±maxDeg from center.
class Control {
public:
    bool  rateOnlyMode = false;   // true = rate damper only (Kd); false = full PID

    // ── Pitch PID ─────────────────────────────────────────────────────
    float pitchKp      =  5.0f;   // servo-deg per deg of error
    float pitchKi      =  0.1f;  // servo-deg per (deg·s) of accumulated error
    float pitchKd      =  0.5f;   // servo-deg per deg/s of pitch rate
    float pitchTarget  = 5.0f;   // hold target, degrees (negative = nose-down)
    bool  pitchReverse = false;
    float pitchMaxDeg  = 85.0f;

    // ── Yaw PID ───────────────────────────────────────────────────────
    float yawKp        =  5.0f;
    float yawKi        =  0.1f;
    float yawKd        =  0.5f;
    bool  yawReverse   = true;
    float yawMaxDeg    = 85.0f;

    int   pitchOut = 90;  // last value written to pitch servo (0–180)
    int   yawOut   = 90;  // last value written to yaw servo (0–180)

    // Diagnostic: each PID term as computed last tick (deg)
    float dbgPitchErr = 0.0f;
    float dbgPitchP   = 0.0f;
    float dbgPitchI   = 0.0f;
    float dbgPitchD   = 0.0f;

    void begin();
    // Call every loop tick. Pass current angles (deg), body rates (deg/s),
    // body-X acceleration (m/s²), and whether the FSM is in FLIGHT.
    void update(float pitch, float pitchRate,
                float yaw,   float yawRate,
                float accelX, bool inFlight);

private:
    Servo    _pitchServo;
    Servo    _yawServo;

    float    _pitchIntegral = 0.0f;
    float    _yawIntegral   = 0.0f;

    // Arithmetic-mean accumulator for throw-direction yaw averaging
    float    _yawSum     = 0.0f;
    int      _yawSamples = 0;
    float    _yawTarget  = 0.0f;

    bool     _prevInFlight = false;
    uint32_t _lastMicros   = 0;
};

#include "Control.h"
#include <Arduino.h>
#include <math.h>

static constexpr float DEG2RAD = M_PI / 180.0f;
static constexpr float RAD2DEG = 180.0f / M_PI;

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void Control::begin() {
    _pitchServo.attach(PIN_PITCH_SERVO);
    _yawServo.attach(PIN_YAW_SERVO);
    _pitchServo.write(90); pitchOut = 90;
    _yawServo.write(90);   yawOut   = 90;
    _lastMicros = micros();
}

void Control::update(float pitch, float pitchRate,
                     float yaw,   float yawRate,
                     float accelX, bool inFlight) {
    uint32_t now = micros();
    float dt = (float)(now - _lastMicros) * 1e-6f;
    _lastMicros = now;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.05f;  // guard first-call / micros() wrap

    // ── Throw-direction averaging ──────────────────────────────────────────────
    // While still pre-launch, record yaw as a circular sum whenever accelX is
    // above the launch threshold. atan2(sumSin, sumCos) gives the mean heading
    // without wrap-around error.
    if (!inFlight && accelX >= LAUNCH_ACCEL_THRESHOLD) {
        _yawSumSin += sinf(yaw * DEG2RAD);
        _yawSumCos += cosf(yaw * DEG2RAD);
        _yawSamples++;
    }

    // ── State-transition handling ──────────────────────────────────────────────
    bool justLaunched = !_prevInFlight && inFlight;
    bool justReset    =  _prevInFlight && !inFlight;
    _prevInFlight = inFlight;

    if (justLaunched) {
        // Lock yaw hold target to the circular mean of throw samples, or fall
        // back to the current heading if no samples were collected.
        _yawTarget = (_yawSamples > 0)
            ? atan2f(_yawSumSin, _yawSumCos) * RAD2DEG
            : yaw;
        // Clear integrators so there's no accumulated error from before launch.
        _pitchIntegral = 0.0f;
        _yawIntegral   = 0.0f;
    }

    if (justReset) {
        // Returning to STANDBY/CALIBRATION — clear everything so the next
        // throw starts fresh.
        _pitchIntegral = 0.0f;
        _yawIntegral   = 0.0f;
        _yawSumSin  = 0.0f;
        _yawSumCos  = 0.0f;
        _yawSamples = 0;
    }

    if (!inFlight) {
        _pitchServo.write(90); pitchOut = 90;
        _yawServo.write(90);   yawOut   = 90;
        return;
    }

    // ── Pitch ─────────────────────────────────────────────────────────────────
    float pitchError = pitchTarget - pitch;
    float pP = 0.0f, pI = 0.0f;
    float pD = -pitchKd * pitchRate;

    if (!rateOnlyMode) {
        pP = pitchKp * pitchError;
        float pCmd0 = pitchReverse ? -(pP + pitchKi * _pitchIntegral + pD)
                                   :  (pP + pitchKi * _pitchIntegral + pD);
        bool pitchSat = (pCmd0 >= pitchMaxDeg && pitchError < 0.0f)
                     || (pCmd0 <= -pitchMaxDeg && pitchError > 0.0f);
        if (!pitchSat)
            _pitchIntegral += pitchError * dt;
        if (pitchKi != 0.0f)
            _pitchIntegral = clampf(_pitchIntegral, -pitchMaxDeg / pitchKi, pitchMaxDeg / pitchKi);
        pI = pitchKi * _pitchIntegral;
    }

    float pCmd = clampf(pitchReverse ? -(pP + pI + pD) : (pP + pI + pD), -pitchMaxDeg, pitchMaxDeg);

    dbgPitchErr = pitchError;
    dbgPitchP   = pP;
    dbgPitchI   = pI;
    dbgPitchD   = pD;

    // ── Yaw ───────────────────────────────────────────────────────────────────
    float yawError = _yawTarget - yaw;
    if (yawError >  180.0f) yawError -= 360.0f;
    if (yawError < -180.0f) yawError += 360.0f;

    float yP = 0.0f, yI = 0.0f;
    float yD = -yawKd * yawRate;

    if (!rateOnlyMode) {
        yP = yawKp * yawError;
        float yCmd0 = yawReverse ? -(yP + yawKi * _yawIntegral + yD)
                                 :  (yP + yawKi * _yawIntegral + yD);
        bool yawSat = (yCmd0 >= yawMaxDeg && yawError < 0.0f)
                   || (yCmd0 <= -yawMaxDeg && yawError > 0.0f);
        if (!yawSat)
            _yawIntegral += yawError * dt;
        if (yawKi != 0.0f)
            _yawIntegral = clampf(_yawIntegral, -yawMaxDeg / yawKi, yawMaxDeg / yawKi);
        yI = yawKi * _yawIntegral;
    }

    float yCmd = clampf(yawReverse ? -(yP + yI + yD) : (yP + yI + yD), -yawMaxDeg, yawMaxDeg);

    pitchOut = (int)(90.0f + pCmd); _pitchServo.write(pitchOut);
    yawOut   = (int)(90.0f + yCmd); _yawServo.write(yawOut);
}

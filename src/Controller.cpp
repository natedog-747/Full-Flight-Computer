#include "Controller.h"
#include <math.h>

// ── engage ────────────────────────────────────────────────────────────────────
// Captures the KF LLA position as the NED reference origin.
// Latches current attitude as initial setpoints so the first update() produces
// zero control error (bumpless transfer — no servo jerk on engage).
void Controller::engage(const SensorData &snap) {
    // ── NED origin ────────────────────────────────────────────────────────
    _refLat = snap.kfLat;
    _refLon = snap.kfLon;
    _refAlt = snap.kfAlt;
    computeEarthRadii(_refLat, _Rn, _Re);

    // ── Bumpless setpoint initialisation ─────────────────────────────────
    // Start holding whatever attitude the plane has at engage so there is
    // no sudden deflection.  Guidance can update these via setTarget*().
    _targetPitch = snap.pitch;
    _targetRoll  = snap.roll;

    // ── Home heading ──────────────────────────────────────────────────────
    _homeHeading = snap.yaw;   // deg, 0 = North, CW positive
}

// ── updateState (shared between both axes) ────────────────────────────────────
// Refreshes NED position, velocity, attitude, and quaternion from the KF snapshot.
// Called at the top of each axis update so either method can be used standalone.
void Controller::updateState(const SensorData &snap) {
    _posN = (snap.kfLat - _refLat) * _Rn;
    _posE = (snap.kfLon - _refLon) * _Re;
    _posD = _refAlt - snap.kfAlt;

    _velN  = snap.kfVelN;
    _velE  = snap.kfVelE;
    _velD  = snap.kfVelD;
    _roll  = snap.roll;
    _pitch = snap.pitch;
    _yaw   = snap.yaw;
    _qw    = snap.qw;
    _qx    = snap.qx;
    _qy    = snap.qy;
    _qz    = snap.qz;
}

// ── updateAxisA — pitch / elevator ────────────────────────────────────────────
//   u = center + Kp*(targetPitch - pitch) - Kd*pitchRate
//
// IMU rate mapping:  pitch rate NED = -snap.gy  (sensor y = left-wing, NED y = right-wing)
// Sign convention:   servo > 1500 µs → nose UP.  Flip KP_A / KD_A signs if reversed.
uint16_t Controller::updateAxisA(const SensorData &snap) {
    updateState(snap);

    float yawRate = snap.gz;   // NED yaw rate (CW positive
    float errYaw  = snap.yaw - _homeHeading;

    float uA = SERVO_CENTER_US + KP_A * errYaw - KD_A * yawRate;
    return clampUs(uA);
}

// ── updateAxisB — roll / aileron ──────────────────────────────────────────────
//   u = center + Kp*(targetRoll - roll) - Kd*rollRate
//
// IMU rate mapping:  roll rate NED = snap.gx  (sensor x = nose = NED x, no flip)
// Sign convention:   servo > 1500 µs → right-wing DOWN.  Flip KP_B / KD_B if reversed.
uint16_t Controller::updateAxisB(const SensorData &snap) {
    updateState(snap);

    float rollRate = snap.gx;   // NED roll rate (right-wing-down positive)
    float errRoll  = _targetRoll - _roll;

    float uB = SERVO_CENTER_US + KP_B * errRoll - KD_B * rollRate;
    return clampUs(uB);
}

// ── clampUs ───────────────────────────────────────────────────────────────────
uint16_t Controller::clampUs(float us) {
    if (us < (float)SERVO_MIN_US) return SERVO_MIN_US;
    if (us > (float)SERVO_MAX_US) return SERVO_MAX_US;
    return (uint16_t)us;
}

// ── computeEarthRadii ─────────────────────────────────────────────────────────
// WGS84 meridional radius Rn and east-arc radius Re at latitude latRad (radians).
// Used to convert lat/lon differences to metres for the NED position computation.
void Controller::computeEarthRadii(float latRad, float &Rn, float &Re) {
    static constexpr float kAe = 6378137.0f;
    static constexpr float ke2 = 0.00669437999014f;
    float sinLat = sinf(latRad);
    float denom  = 1.0f - ke2 * sinLat * sinLat;
    Rn = kAe * (1.0f - ke2) / (denom * sqrtf(denom));
    Re = kAe / sqrtf(denom) * cosf(latRad);
    if (Rn < 1.0f) Rn = 1.0f;
    if (Re < 1.0f) Re = 1.0f;
}

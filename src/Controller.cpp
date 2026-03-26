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
}

// ── update ────────────────────────────────────────────────────────────────────
// 1. Refresh all internal state from the KF snapshot.
// 2. Compute NED position relative to the engage origin.
// 3. Run PD control on pitch (Axis A) and roll (Axis B).
// 4. Return servo pulse widths in microseconds.
//
// IMU angular-rate mapping (see header for full convention):
//   pitch rate NED = -snap.gy  [deg/s]   (sensor y = left-wing, NED y = right-wing)
//   roll  rate NED =  snap.gx  [deg/s]   (sensor x = nose = NED x, no sign flip)
void Controller::update(const SensorData &snap, uint16_t &pwmA, uint16_t &pwmB) {
    // ── State update ──────────────────────────────────────────────────────

    // NED position (metres) relative to the engage-point origin
    _posN = (snap.kfLat - _refLat) * _Rn;
    _posE = (snap.kfLon - _refLon) * _Re;
    _posD = _refAlt - snap.kfAlt;   // positive-down: climbs give negative posD

    // NED velocity and attitude directly from KF
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

    // ── Angular rates (deg/s, NED frame) ─────────────────────────────────
    // These come straight from the IMU — no differentiation noise.
    float pitchRate =  -snap.gy;   // NED pitch rate (nose-up positive)
    float rollRate  =   snap.gx;   // NED roll  rate (right-wing-down positive)

    // ── PD control laws ───────────────────────────────────────────────────
    //
    // Axis A — pitch (elevator / ServoA):
    //   u = center + Kp*(target - pitch) - Kd*pitchRate
    //
    // Axis B — roll (aileron / ServoB):
    //   u = center + Kp*(target - roll) - Kd*rollRate
    //
    // KP and KD signs assume: servo > 1500 µs deflects nose-UP / right-wing-DOWN.
    // Flip the sign of KP_A / KP_B in the header if your linkage is reversed.

    float errPitch = _targetPitch - _pitch;
    float errRoll  = _targetRoll  - _roll;

    float uA = SERVO_CENTER_US + KP_A * errPitch - KD_A * pitchRate;
    float uB = SERVO_CENTER_US + KP_B * errRoll  - KD_B * rollRate;

    pwmA = clampUs(uA);
    pwmB = clampUs(uB);
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

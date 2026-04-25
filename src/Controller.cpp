#include "Controller.h"
#include <math.h>

// ── engage ────────────────────────────────────────────────────────────────────
// Captures the KF LLA position as the NED reference origin.
// Latches current attitude as initial setpoints so the first update() produces
// zero control error (bumpless transfer — no servo jerk on engage).
void Controller::engage(const SensorData &snap, uint16_t rcInputB) {
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

    // ── Axis-B RC centre — bumpless transfer for altitude servo ──────────
    // Output is relative to this value so the servo does not jump on engage.
    // Clamp to a sane range in case of a stale / noisy RC reading.
    if (rcInputB < SERVO_MIN_US) rcInputB = SERVO_MIN_US;
    if (rcInputB > SERVO_MAX_US) rcInputB = SERVO_MAX_US;
    _rcCenterB = rcInputB;

    _integA    = 0.0f;   // clear integrators on every engage for bumpless transfer
    _integB    = 0.0f;
    _isEngaged = true;
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

// ── updateAxisA — yaw / heading hold ─────────────────────────────────────────
//   u = center - ( Kp*err + Ki*∫err·dt - Kd*yawRate )
//
// The correction is negated so that a positive heading error (plane rotated CW
// from home) commands servo < 1500 µs.  Flip all signs if your airframe is
// reversed relative to this convention.
//
// Angle wrap:  error is normalised to [-180, 180] so the controller always
// takes the short way around (no jump at the ±180° boundary).
//
// Anti-windup: the integrator contribution is clamped to ±INTEG_LIMIT_A µs
// before being added to the output.  The integrator itself accumulates without
// limit internally; only its effect on the servo is bounded.
//
// dt: taken from snap.dtMs (IMU cycle time, ~10 ms), clamped to [1, 50] ms to
// prevent runaway accumulation if the snapshot is stale.
uint16_t Controller::updateAxisA(const SensorData &snap) {
    if (!_isEngaged) return SERVO_CENTER_US;
    updateState(snap);

    // ── Heading error, wrapped to [-180, 180] ─────────────────────────────
    float errYaw = snap.yaw - _homeHeading;
    while (errYaw >  180.0f) errYaw -= 360.0f;
    while (errYaw < -180.0f) errYaw += 360.0f;
    _errYaw = errYaw;   // store for logging

    // ── Integrator accumulation ───────────────────────────────────────────
    float dt = snap.dtMs * 0.001f;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.050f) dt = 0.050f;
    _integA += errYaw * dt;

    // Anti-windup: clamp integrator contribution (not the state itself, so
    // it keeps tracking but can't saturate the output on its own).
    float integContrib = KI_A * _integA;
    if (integContrib >  INTEG_LIMIT_A) integContrib =  INTEG_LIMIT_A;
    if (integContrib < -INTEG_LIMIT_A) integContrib = -INTEG_LIMIT_A;

    // ── PID output (negated — servo is physically reversed) ──────────────
    float yawRate = snap.gz;   // NED yaw rate, deg/s, CW positive
    float uA = SERVO_CENTER_US - (KP_A * errYaw + integContrib - KD_A * yawRate);
    return clampCtrl(uA);
}

// ── updateAxisB — altitude hold (PI) ─────────────────────────────────────────
//   u = center + Kp*errAlt + Ki*∫errAlt·dt
//
// Reference altitude (_refAlt) is the KF altitude (m) captured at engage time.
// Error convention:  errAlt > 0 → plane is BELOW target → needs to climb.
// Servo convention:  servo > 1500 µs should produce nose-UP / climb.
//                    If your airframe is reversed, negate KP_B and KI_B.
//
// No derivative term — altitude is a slow-enough state that a D term on raw
// KF altitude adds noise more than it helps.  Add if desired.
//
// Anti-windup: same clamped-contribution approach as axis A.
uint16_t Controller::updateAxisB(const SensorData &snap) {
    if (!_isEngaged) return SERVO_CENTER_US;
    updateState(snap);

    // ── Altitude error (m): positive = below engage altitude ─────────────
    float errAlt = _refAlt - snap.kfAlt;
    _errAlt = errAlt;   // store for logging

    // ── Integrator accumulation ───────────────────────────────────────────
    float dt = snap.dtMs * 0.001f;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.050f) dt = 0.050f;
    _integB += errAlt * dt;

    // Anti-windup: clamp contribution, not the state
    float integContrib = KI_B * _integB;
    if (integContrib >  INTEG_LIMIT_B) integContrib =  INTEG_LIMIT_B;
    if (integContrib < -INTEG_LIMIT_B) integContrib = -INTEG_LIMIT_B;

    // ── PI output (negated — servo is physically reversed) ────────────────
    // Output is offset from _rcCenterB (RC input captured at engage) so there
    // is no servo jump on takeover.  clampCtrl still limits ±CTRL_LIMIT_US
    // around SERVO_CENTER_US for absolute authority bounds.
    float uB = (float)_rcCenterB - (KP_B * errAlt + integContrib);
    return clampCtrl(uB);
}

// ── clampUs ───────────────────────────────────────────────────────────────────
uint16_t Controller::clampUs(float us) {
    if (us < (float)SERVO_MIN_US) return SERVO_MIN_US;
    if (us > (float)SERVO_MAX_US) return SERVO_MAX_US;
    return (uint16_t)us;
}

// ── clampCtrl ─────────────────────────────────────────────────────────────────
// Limits controller output to ±CTRL_LIMIT_US (±20°) around centre.
// Prevents the autopilot from commanding more than 20° of deflection regardless
// of how large the error or integrator grow.
uint16_t Controller::clampCtrl(float us) {
    const float lo = (float)(SERVO_CENTER_US - CTRL_LIMIT_US);
    const float hi = (float)(SERVO_CENTER_US + CTRL_LIMIT_US);
    if (us < lo) return (uint16_t)lo;
    if (us > hi) return (uint16_t)hi;
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

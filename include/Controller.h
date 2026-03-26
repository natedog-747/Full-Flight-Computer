#pragma once
#include <stdint.h>
#include "SensorData.h"

// ── Autopilot Controller ──────────────────────────────────────────────────────
//
// Activated when the engage switch fires.  On the rising edge, engage() must be
// called once to snapshot the reference NED origin (from the KF LLA state) and
// latch the current attitude as the initial setpoint.
//
// update() is then called every loop.  It:
//   1. Converts KF LLA → NED metres relative to the engage origin
//   2. Runs independent PD loops on Axis A (pitch/elevator) and Axis B (roll/aileron)
//   3. Returns servo pulse widths ready to pass to writeMicroseconds()
//
// Axis A → ServoA (GPIO 10)   — pitch channel (elevator)
// Axis B → ServoB (GPIO 9)    — roll  channel (aileron)
//
// IMU axis convention (from main.cpp):
//   Sensor frame:  x = nose,  y = left-wing,  z = up
//   NED frame:     x = nose,  y = right-wing, z = down
//   roll  rate NED =  snap.gx [deg/s]
//   pitch rate NED = -snap.gy [deg/s]
//
// Servo sign convention (positive error → positive µs correction):
//   If elevator deflects nose-UP for servo > 1500, set KP_A  positive.
//   If elevator deflects nose-UP for servo < 1500, set KP_A  negative.
//   Mirror logic applies to KD and Axis B.
// ─────────────────────────────────────────────────────────────────────────────

class Controller {
public:
    // ── Tuning ────────────────────────────────────────────────────────────────
    static constexpr float KP_A =  8.0f;    // [µs/deg]      pitch proportional
    static constexpr float KD_A =  3.0f;    // [µs/(deg/s)]  pitch derivative
    static constexpr float KP_B =  8.0f;    // [µs/deg]      roll  proportional
    static constexpr float KD_B =  3.0f;    // [µs/(deg/s)]  roll  derivative

    static constexpr uint16_t SERVO_CENTER_US = 1500;
    static constexpr uint16_t SERVO_MIN_US    = 1000;
    static constexpr uint16_t SERVO_MAX_US    = 2000;

    Controller() = default;

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    // Call exactly once on the engage rising edge.
    // Captures KF LLA as NED origin, WGS84 Earth radii, and latches current
    // pitch/roll as the initial setpoints (smooth takeover — no sudden jerk).
    void engage(const SensorData &snap);

    // Call every loop while engaged.
    // Each method refreshes internal state and runs the PD law for its axis.
    // Returns the servo pulse width in microseconds.
    uint16_t updateAxisA(const SensorData &snap);   // pitch / elevator
    uint16_t updateAxisB(const SensorData &snap);   // roll  / aileron

    // ── Setpoints (update from a guidance layer as needed) ────────────────────
    void setTargetPitch(float deg) { _targetPitch = deg; }
    void setTargetRoll (float deg) { _targetRoll  = deg; }
    float getTargetPitch() const   { return _targetPitch; }
    float getTargetRoll()  const   { return _targetRoll;  }

    // ── NED state relative to engage origin ──────────────────────────────────
    // Position (m): North/East/Down from the point where engage was pressed.
    float getPosN()  const { return _posN;  }
    float getPosE()  const { return _posE;  }
    float getPosD()  const { return _posD;  }

    // Velocity (m/s, NED frame)
    float getVelN()  const { return _velN;  }
    float getVelE()  const { return _velE;  }
    float getVelD()  const { return _velD;  }

    // Home altitude — KF altitude (m) captured at the engage instant.
    float getHomeAlt()     const { return _refAlt;      }
    // Home heading — yaw (deg) captured at the engage instant.
    float getHomeHeading() const { return _homeHeading; }

    // Attitude
    float getRoll()  const { return _roll;  }   // deg, positive = right-wing down
    float getPitch() const { return _pitch; }   // deg, positive = nose up
    float getYaw()   const { return _yaw;   }   // deg, 0 = North, CW positive

    // Quaternion (body-to-NED, Hamilton [w x y z])
    float getQw() const { return _qw; }
    float getQx() const { return _qx; }
    float getQy() const { return _qy; }
    float getQz() const { return _qz; }

private:
    // ── Reference NED origin (set at engage) ─────────────────────────────────
    float _refLat = 0.0f, _refLon = 0.0f, _refAlt = 0.0f;
    float _homeHeading = 0.0f;   // yaw at engage (deg, 0 = North, CW positive)
    float _Rn = 6.371e6f, _Re = 6.371e6f;   // WGS84 radii at origin

    // ── Setpoints ─────────────────────────────────────────────────────────────
    float _targetPitch = 0.0f;
    float _targetRoll  = 0.0f;

    // ── Derivative state (for PD — uses gyro rates directly) ─────────────────
    // No extra state needed: angular rates come straight from snap.gx / snap.gy.

    // ── Current state (populated by update()) ────────────────────────────────
    float _posN = 0.0f, _posE = 0.0f, _posD = 0.0f;
    float _velN = 0.0f, _velE = 0.0f, _velD = 0.0f;
    float _roll  = 0.0f, _pitch = 0.0f, _yaw = 0.0f;
    float _qw = 1.0f, _qx = 0.0f, _qy = 0.0f, _qz = 0.0f;

    // Refresh NED position, velocity, attitude, and quaternion from a snapshot.
    void updateState(const SensorData &snap);

    // Clamp a float µs value to [SERVO_MIN_US, SERVO_MAX_US]
    static uint16_t clampUs(float us);

    // Compute WGS84 meridional (Rn) and east-arc (Re) radii for a given latitude
    static void computeEarthRadii(float latRad, float &Rn, float &Re);
};

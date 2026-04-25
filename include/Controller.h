#pragma once
#include <stdint.h>
#include "SensorData.h"

// ── Autopilot Controller ──────────────────────────────────────────────────────
//
// Activated when the engage switch fires.  On the rising edge, engage() must be
// called once to snapshot the reference NED origin (from the KF LLA state) and
// latch the current attitude as the initial setpoint.
//
// updateAxisA/B are called every loop.  Each returns a servo pulse width (µs)
// ready to pass to writeMicroseconds().  Controller authority is limited to
// ±CTRL_LIMIT_DEG (30°) from centre on both axes.
//
// Axis A → ServoA (GPIO 10)   — yaw / heading hold (PID, negated output)
// Axis B → ServoB (GPIO 9)    — altitude hold (PI, ref = kfAlt at engage)
//
// IMU axis convention (from main.cpp):
//   Sensor frame:  x = nose,  y = left-wing,  z = up
//   NED frame:     x = nose,  y = right-wing, z = down
//   yaw  rate NED =  snap.gz [deg/s]
//   roll rate NED =  snap.gx [deg/s]
// ─────────────────────────────────────────────────────────────────────────────

class Controller {
public:
    // ── Tuning ────────────────────────────────────────────────────────────────
    // Axis A (yaw/heading hold) — output is negated so positive error → servo < 1500
    static constexpr float KP_A          =  8.0f;    // [µs/deg]        proportional
    static constexpr float KI_A          =  0.5f;    // [µs/(deg·s)]    integral
    static constexpr float KD_A          =  3.0f;    // [µs/(deg/s)]    derivative
    static constexpr float INTEG_LIMIT_A = 150.0f;   // [µs]  anti-windup clamp on integrator contribution

    // Axis B (altitude hold) — units are metres, not degrees
    static constexpr float KP_B          =  100.0f;    // [µs/m]      altitude proportional
    static constexpr float KI_B          =  100.0f;    // [µs/(m·s)]  altitude integral
    static constexpr float INTEG_LIMIT_B = 150.0f;   // [µs]  anti-windup clamp on integrator contribution

    static constexpr uint16_t SERVO_CENTER_US = 1500;
    static constexpr uint16_t SERVO_MIN_US    = 800;
    static constexpr uint16_t SERVO_MAX_US    = 2200;

    // Maximum controller authority: ±20° from centre.
    // Derived from the standard RC servo mapping: 500 µs = 90°, so 1° ≈ 5.56 µs.
    // Controller output is clamped to [CENTER - CTRL_LIMIT_US, CENTER + CTRL_LIMIT_US]
    // regardless of how large the error or integrator grow.
    static constexpr float    CTRL_LIMIT_DEG = 40.0f;
    static constexpr uint16_t CTRL_LIMIT_US  =
        static_cast<uint16_t>(CTRL_LIMIT_DEG * 500.0f / 90.0f);  // = 222 µs

    Controller() = default;

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    // Call exactly once on the engage rising edge.
    // Captures KF LLA as NED origin, WGS84 Earth radii, and latches current
    // pitch/roll as the initial setpoints (smooth takeover — no sudden jerk).
    // rcInputB is the raw RC servo-B pulse width (µs) at the moment of engagement;
    // it becomes the PI output center so the servo does not jump on takeover.
    void engage(const SensorData &snap, uint16_t rcInputB);

    // Call when the engage switch turns off.  Clears the internal engaged flag
    // so updateAxisA/B cannot output control even if called accidentally.
    // Also zeroes integrators so they don't wind up across engage cycles.
    void disengage() { _isEngaged = false; _integA = 0.0f; _integB = 0.0f; }

    bool isEngaged() const { return _isEngaged; }

    // Call every loop while engaged.
    // Each method refreshes internal state and runs the PD law for its axis.
    // Returns SERVO_CENTER_US (safe neutral) if engage() has not been called.
    uint16_t updateAxisA(const SensorData &snap);   // yaw   / heading hold (PID)
    uint16_t updateAxisB(const SensorData &snap);   // altitude hold (PI, ref = kfAlt at engage)

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

    // PID diagnostics — valid after the first updateAxisA/B call while engaged.
    float getErrYaw()  const { return _errYaw;  }   // [deg]   last yaw error (axis A)
    float getIntegA()  const { return _integA;  }   // [deg·s] integrator state (axis A)
    float getErrAlt()  const { return _errAlt;  }   // [m]     last altitude error (axis B)
    float getIntegB()  const { return _integB;  }   // [m·s]   integrator state (axis B)

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
    bool _isEngaged = false;   // set true only by engage(), cleared by disengage()

    // ── Reference NED origin (set at engage) ─────────────────────────────────
    float    _refLat = 0.0f, _refLon = 0.0f, _refAlt = 0.0f;
    float    _homeHeading = 0.0f;   // yaw at engage (deg, 0 = North, CW positive)
    uint16_t _rcCenterB   = SERVO_CENTER_US;  // RC input on axis B captured at engage
    float _Rn = 6.371e6f, _Re = 6.371e6f;   // WGS84 radii at origin

    // ── Setpoints ─────────────────────────────────────────────────────────────
    float _targetPitch = 0.0f;
    float _targetRoll  = 0.0f;

    // ── Integrator state ──────────────────────────────────────────────────────
    float _integA = 0.0f;   // [deg·s]  accumulated yaw error integral (axis A)
    float _integB = 0.0f;   // [m·s]    accumulated altitude error integral (axis B)

    // ── Last computed errors (for logging / diagnostics) ──────────────────────
    float _errYaw = 0.0f;   // [deg]  yaw error from last updateAxisA call
    float _errAlt = 0.0f;   // [m]    altitude error from last updateAxisB call

    // ── Current state (populated by update()) ────────────────────────────────
    float _posN = 0.0f, _posE = 0.0f, _posD = 0.0f;
    float _velN = 0.0f, _velE = 0.0f, _velD = 0.0f;
    float _roll  = 0.0f, _pitch = 0.0f, _yaw = 0.0f;
    float _qw = 1.0f, _qx = 0.0f, _qy = 0.0f, _qz = 0.0f;

    // Refresh NED position, velocity, attitude, and quaternion from a snapshot.
    void updateState(const SensorData &snap);

    // Clamp a float µs value to [SERVO_MIN_US, SERVO_MAX_US]
    static uint16_t clampUs(float us);

    // Clamp a controller output to [CENTER - CTRL_LIMIT_US, CENTER + CTRL_LIMIT_US]
    static uint16_t clampCtrl(float us);

    // Compute WGS84 meridional (Rn) and east-arc (Re) radii for a given latitude
    static void computeEarthRadii(float latRad, float &Rn, float &Re);
};

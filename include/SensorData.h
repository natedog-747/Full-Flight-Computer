#pragma once
#include <stdint.h>

// Barometer calibration phase
enum class BmpPhase : uint8_t { WARMUP = 0, SETTLING = 1, LOCKED = 2 };

// Plain data transfer object shared between Core 0 (sensor task) and
// Core 1 (logger task). Always access under gDataMutex in main.cpp.
// No FreeRTOS types here — sensor class files don't need FreeRTOS.
struct SensorData {
    uint32_t timestampMs = 0;

    // IMU — BNO055 (m/s², deg/s)
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;
    uint8_t calSys = 0, calGyro = 0, calAccel = 0, calMag = 0;

    // Barometer — BMP390
    float    relAltM         = 0;
    BmpPhase bmpPhase        = BmpPhase::WARMUP;
    uint32_t bmpSettleRemSec = 0;

    // GPS — geodetic (LLA)
    bool     gpsFix        = false;
    float    gpsLat        = 0;   // radians
    float    gpsLon        = 0;   // radians
    float    gpsAlt        = 0;   // metres above ellipsoid
    // NED velocity derived from speed/course over ground (velD always 0 — not in NMEA)
    float    velN_ms = 0, velE_ms = 0, velD_ms = 0;
    float    gpsSpeedMs    = 0;   // speed over ground (m/s)
    float    gpsHeadingDeg = 0;
    uint8_t  gpsSats       = 0;
    float    gpsHDOP       = 0;

    // Attitude — gyro-rate quaternion integration (NED frame, ZYX Euler)
    // kfInitPhase: 0=ACCEL_AVG, 1=AWAIT_YAW, 2=READY
    uint8_t kfInitPhase = 0;
    float qw = 1, qx = 0, qy = 0, qz = 0;  // body-to-NED quaternion [w,x,y,z]
    float roll = 0, pitch = 0, yaw = 0;     // deg — roll(x), pitch(y), yaw(z)

    // KF dead-reckoning navigation (geodetic LLA + NED velocity)
    float kfLat = 0, kfLon = 0, kfAlt = 0;   // rad, rad, m
    float kfVelN = 0, kfVelE = 0, kfVelD = 0;
    float kfBaroBias = 0;  // barometer bias estimate (m)

    // Flags — set by sensor drivers each cycle; cleared by caller after use
    bool gpsNewData  = false;  // true for one cycle when GpsSensor has a fresh fix

    // Diagnostics
    float dtMs = 0;   // true sensor loop period (ms), measured start-to-start

    // ── Flight control state (written by loop1 into local snap before logging) ─
    // These are NOT in gShared — only the local copy in loop1() carries them.
    bool     ctrlEngaged   = false;  // true when autopilot is fully engaged & controlling
    uint16_t ctrlServoA_us = 1500;   // axis A output µs (controller cmd, or 1500 if passthrough)
    uint16_t ctrlServoB_us = 1500;   // axis B output µs
    uint32_t rcA_us        = 1500;   // RC receiver input channel A (µs)
    uint32_t rcB_us        = 1500;   // RC receiver input channel B (µs)
    uint32_t engagePulseUs = 0;      // engage/reset switch pulse width (µs)
    float    ctrlErrYaw    = 0.0f;   // yaw error fed into axis A PID (deg)
    float    ctrlIntegA    = 0.0f;   // axis A integrator state (deg·s)
    float    ctrlErrAlt    = 0.0f;   // altitude error fed into axis B PI (m)
    float    ctrlIntegB    = 0.0f;   // axis B integrator state (m·s)
    float    ctrlRefAlt    = 0.0f;   // altitude reference latched at engage (m)
};

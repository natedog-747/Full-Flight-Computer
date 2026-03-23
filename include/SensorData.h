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

    // GPS — NED frame (metres from averaged origin)
    bool     gpsFix        = false;
    bool     gpsOrigin     = false;  // true once 10-s origin average is locked
    uint32_t gpsAvgRemSec  = 0;      // seconds left in origin-averaging window
    float    nedN = 0, nedE = 0, nedD = 0;
    // NED velocity components derived from speed/course over ground.
    // velD is always 0 — vertical velocity is not in standard NMEA.
    float    velN_ms = 0, velE_ms = 0, velD_ms = 0;
    float    gpsSpeedMs    = 0;   // overall speed over ground (m/s)
    float    gpsHeadingDeg = 0;
    uint8_t  gpsSats       = 0;
    float    gpsHDOP       = 0;

    // Attitude — gyro-rate quaternion integration (NED frame, ZYX Euler)
    // kfInitPhase: 0=ACCEL_AVG, 1=AWAIT_YAW, 2=READY
    uint8_t kfInitPhase = 0;
    float qw = 1, qx = 0, qy = 0, qz = 0;  // body-to-NED quaternion [w,x,y,z]
    float roll = 0, pitch = 0, yaw = 0;     // deg — roll(x), pitch(y), yaw(z)

    // KF dead-reckoning navigation (NED frame, metres and m/s)
    float kfPosN = 0, kfPosE = 0, kfPosD = 0;
    float kfVelN = 0, kfVelE = 0, kfVelD = 0;
    float kfBaroBias = 0;  // barometer bias estimate (m)

    // Flags — set by sensor drivers each cycle; cleared by caller after use
    bool gpsNewData  = false;  // true for one cycle when GpsSensor has a fresh fix

    // Diagnostics
    float dtMs = 0;   // true sensor loop period (ms), measured start-to-start
};

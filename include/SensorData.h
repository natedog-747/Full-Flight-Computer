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
    bool    gpsFix        = false;
    bool    gpsOrigin     = false;
    float   nedN = 0, nedE = 0, nedD = 0;
    float   gpsSpeedMs    = 0;
    float   gpsHeadingDeg = 0;
    uint8_t gpsSats       = 0;
    float   gpsHDOP       = 0;

    // Diagnostics
    float dtMs = 0;   // true sensor loop period (ms), measured start-to-start
};

#pragma once
#include <Adafruit_GPS.h>
#include "SensorData.h"

class GpsSensor {
public:
    explicit GpsSensor(HardwareSerial &serial);

    void begin();

    // Non-blocking parse; writes into out when a valid fix is available.
    // Must be called every loop iteration on Core 0.
    void update(SensorData &out);

    bool isOriginSet() const { return _originSet; }

private:
    Adafruit_GPS _gps;

    bool     _originSet   = false;
    bool     _averaging   = false;
    uint32_t _avgStart    = 0;
    uint32_t _sampleCount = 0;
    double   _sumLat = 0, _sumLon = 0, _sumAlt = 0;
    double   _originLat = 0, _originLon = 0;
    float    _originAlt = 0;

    static constexpr uint8_t  MIN_FIX_QUALITY = 1;
    static constexpr float    MAX_HDOP        = 2.0f;
    static constexpr uint32_t AVG_DURATION_MS = 10000;
    static constexpr double   R_EARTH         = 6371000.0;

    void _computeNED(SensorData &out) const;
};

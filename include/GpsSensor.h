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

private:
    Adafruit_GPS _gps;

    static constexpr uint8_t MIN_FIX_QUALITY = 1;
};

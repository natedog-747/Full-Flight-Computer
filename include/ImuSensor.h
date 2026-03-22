#pragma once
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include "SensorData.h"

class ImuSensor {
public:
    explicit ImuSensor(TwoWire &wire = Wire, uint8_t addr = BNO055_ADDRESS_A);

    // Call once before first read(); Wire must be initialised beforehand
    bool begin();

    // Single burst I2C read: 18 bytes from 0x08 (accel + mag skip + gyro).
    // ~450 µs at 400 kHz. Writes ax/ay/az/gx/gy/gz + timestamp into out.
    void read(SensorData &out);

    // Cheap register read; call at ~1 Hz to avoid I2C overhead
    void getCalibration(uint8_t &sys, uint8_t &gyro,
                        uint8_t &accel, uint8_t &mag);

private:
    Adafruit_BNO055 _bno;
    TwoWire        &_wire;
    uint8_t         _addr;
};

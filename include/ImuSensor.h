#pragma once
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <math.h>

class ImuSensor {
public:
    float roll      = 0.0f;
    float pitch     = 0.0f;
    float yaw       = 0.0f;
    bool  calibrating = true;

    ImuSensor();
    bool begin();
    void update();

private:
    Adafruit_BNO055 _bno;

    float    _qw, _qx, _qy, _qz;
    double   _accelSumX, _accelSumY, _accelSumZ;
    int      _calibN;
    uint32_t _calibStartMs;
    uint32_t _lastMicros;
    uint32_t _lastTick;

    void _initFromAccel(float ax, float ay, float az);
    void _integrateGyro(float wx, float wy, float wz, float dt);
    void _quatToEuler();
};

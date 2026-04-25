#include "ImuSensor.h"
#include <Arduino.h>

// ── Coordinate frame transformation ──────────────────────────────────────────
// Sensor frame (BNO055 as mounted): X = nose, Y = up, Z = right wing
// Body frame   (desired):           X = nose, Y = right wing, Z = down
//
// Axis remapping:  body X = +sensor X
//                  body Y = +sensor Z   (sensor Z → right wing)
//                  body Z = -sensor Y   (sensor Y is up; body Z is down → negate)
//
// Flip any SIGN_BODY_* to -1 if that body axis still reads backwards on the bench.
#define SIGN_BODY_X   1    // body X = SIGN_BODY_X * sensor_X
#define SIGN_BODY_Y   1    // body Y = SIGN_BODY_Y * sensor_Z
#define SIGN_BODY_Z  -1    // body Z = SIGN_BODY_Z * sensor_Y  (−1 = flip up→down)

static const uint32_t CALIB_MS = 10000;

ImuSensor::ImuSensor()
    : _bno(55, 0x28, &Wire),
      _qw(1.0f), _qx(0.0f), _qy(0.0f), _qz(0.0f),
      _accelSumX(0), _accelSumY(0), _accelSumZ(0),
      _calibN(0), _calibStartMs(0), _lastMicros(0), _lastTick(0)
{}

bool ImuSensor::begin() {
    if (!_bno.begin()) {
        Serial.println("BNO055 not detected — check wiring!");
        return false;
    }
    _bno.setExtCrystalUse(true);
    _calibStartMs = millis();
    _lastMicros   = micros();
    return true;
}

void ImuSensor::update() {
    // Direct burst read: 18 bytes from reg 0x08
    // buf[0..5]  = ACC (1 LSB = 0.01 m/s²)
    // buf[6..11] = MAG (skipped)
    // buf[12..17]= GYR (1 LSB = 1/16 deg/s)
    uint8_t buf[18] = {};
    Wire.beginTransmission(0x28);
    Wire.write(0x08);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)0x28, (uint8_t)18);
    for (int i = 0; i < 18 && Wire.available(); i++) buf[i] = Wire.read();

    // Apply frame transformation: remap sensor axes to body axes
    float ax = SIGN_BODY_X * (int16_t)((buf[1]  << 8) | buf[0])  / 100.0f; // sensor X → body X
    float ay = SIGN_BODY_Y * (int16_t)((buf[5]  << 8) | buf[4])  / 100.0f; // sensor Z → body Y (right wing)
    float az = SIGN_BODY_Z * (int16_t)((buf[3]  << 8) | buf[2])  / 100.0f; // sensor Y → body Z (down)

    accelX    = ax;
    accelNorm = sqrtf(ax*ax + ay*ay + az*az);

    // Gyro: same axis remapping, convert deg/s → rad/s for integration
    static const float DEG2RAD = M_PI / 180.0f;
    float wx = SIGN_BODY_X * (int16_t)((buf[13] << 8) | buf[12]) / 16.0f * DEG2RAD; // sensor Wx → body Wx
    float wy = SIGN_BODY_Y * (int16_t)((buf[17] << 8) | buf[16]) / 16.0f * DEG2RAD; // sensor Wz → body Wy
    float wz = SIGN_BODY_Z * (int16_t)((buf[15] << 8) | buf[14]) / 16.0f * DEG2RAD; // sensor Wy → body Wz

    if (calibrating) {
        _accelSumX += ax; _accelSumY += ay; _accelSumZ += az;
        _calibN++;

        uint32_t elapsed = millis() - _calibStartMs;

        if (elapsed >= CALIB_MS) {
            float avgAx = (float)(_accelSumX / _calibN);
            float avgAy = (float)(_accelSumY / _calibN);
            float avgAz = (float)(_accelSumZ / _calibN);

            _initFromAccel(avgAx, avgAy, avgAz);
            calibrating = false;
            _lastMicros = micros();

            Serial.print("Calibration done (");
            Serial.print(_calibN);
            Serial.println(" samples)");
        } else {
            if (elapsed / 1000 != _lastTick) {
                _lastTick = elapsed / 1000;
                Serial.print("  Calibrating... ");
                Serial.print(CALIB_MS / 1000 - _lastTick);
                Serial.println(" s remaining");
            }
        }
    } else {
        uint32_t now = micros();
        float dt = (float)(now - _lastMicros) * 1e-6f;
        _lastMicros = now;

        _integrateGyro(wx, wy, wz, dt);
        _quatToEuler();
    }
}

void ImuSensor::_initFromAccel(float ax, float ay, float az) {
    // Body frame: X=nose, Y=right wing, Z=down.
    // Accel reads specific force (reaction to gravity):
    //   level → (0, 0, -g);  right-wing-down φ → ay<0, az≈-g;  nose-up θ → ax>0
    //
    // roll  φ: atan2(-ay, -az)  → 0° when level, +° when right wing down
    // pitch θ: atan2( ax, √(ay²+az²)) → 0° when level, +° when nose up
    float r = atan2f(-ay, -az);
    float p = atan2f( ax, sqrtf(ay*ay + az*az));

    float cr = cosf(r * 0.5f), sr = sinf(r * 0.5f);
    float cp = cosf(p * 0.5f), sp = sinf(p * 0.5f);
    // yaw = 0 → cy = 1, sy = 0
    _qw =  cr*cp;
    _qx =  sr*cp;
    _qy =  cr*sp;
    _qz = -sr*sp;
}

void ImuSensor::_integrateGyro(float wx, float wy, float wz, float dt) {
    float dqw = 0.5f * (-_qx*wx - _qy*wy - _qz*wz) * dt;
    float dqx = 0.5f * ( _qw*wx + _qy*wz - _qz*wy) * dt;
    float dqy = 0.5f * ( _qw*wy - _qx*wz + _qz*wx) * dt;
    float dqz = 0.5f * ( _qw*wz + _qx*wy - _qy*wx) * dt;

    _qw += dqw; _qx += dqx; _qy += dqy; _qz += dqz;

    float norm = sqrtf(_qw*_qw + _qx*_qx + _qy*_qy + _qz*_qz);
    if (norm > 1e-6f) { _qw /= norm; _qx /= norm; _qy /= norm; _qz /= norm; }
}

void ImuSensor::restartCalibration() {
    _accelSumX    = 0; _accelSumY = 0; _accelSumZ = 0;
    _calibN       = 0;
    _calibStartMs = millis();
    _lastTick     = 0;
    calibrating   = true;
}

void ImuSensor::_quatToEuler() {
    roll  = atan2f(2.0f*(_qw*_qx + _qy*_qz), 1.0f - 2.0f*(_qx*_qx + _qy*_qy));
    float sinp = 2.0f*(_qw*_qy - _qz*_qx);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    pitch = asinf(sinp);
    yaw   = atan2f(2.0f*(_qw*_qz + _qx*_qy), 1.0f - 2.0f*(_qy*_qy + _qz*_qz));

    roll  *= (180.0f / M_PI);
    pitch *= (180.0f / M_PI);
    yaw   *= (180.0f / M_PI);
}

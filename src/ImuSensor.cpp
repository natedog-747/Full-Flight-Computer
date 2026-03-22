#include "ImuSensor.h"

ImuSensor::ImuSensor(TwoWire &wire, uint8_t addr)
    : _bno(55, addr), _wire(wire), _addr(addr) {}

bool ImuSensor::begin() {
    if (!_bno.begin()) return false;
    _bno.setExtCrystalUse(true);
    return true;
}

void ImuSensor::read(SensorData &out) {
    // Single burst read: 18 bytes from 0x08
    //   buf[0..5]   = ACC_DATA  (X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB)
    //   buf[6..11]  = MAG_DATA  (skipped)
    //   buf[12..17] = GYR_DATA  (X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB)
    // Scales per BNO055 datasheet in NDOF mode:
    //   accel: 1 LSB = 0.01 m/s²   gyro: 1 LSB = 1/16 deg/s
    uint8_t buf[18] = {};
    _wire.beginTransmission(_addr);
    _wire.write(0x08);
    _wire.endTransmission(false);
    _wire.requestFrom(_addr, (uint8_t)18);
    for (int i = 0; i < 18 && _wire.available(); i++) buf[i] = _wire.read();

    out.timestampMs = millis();
    out.ax = (int16_t)((buf[1]  << 8) | buf[0])  / 100.0f;
    out.ay = (int16_t)((buf[3]  << 8) | buf[2])  / 100.0f;
    out.az = (int16_t)((buf[5]  << 8) | buf[4])  / 100.0f;
    out.gx = (int16_t)((buf[13] << 8) | buf[12]) / 16.0f;
    out.gy = (int16_t)((buf[15] << 8) | buf[14]) / 16.0f;
    out.gz = (int16_t)((buf[17] << 8) | buf[16]) / 16.0f;
}

void ImuSensor::getCalibration(uint8_t &sys, uint8_t &gyro,
                                uint8_t &accel, uint8_t &mag) {
    _bno.getCalibration(&sys, &gyro, &accel, &mag);
}

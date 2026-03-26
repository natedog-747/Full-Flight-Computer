#include "BaroSensor.h"

BaroSensor::BaroSensor(uint8_t i2cAddr) : _addr(i2cAddr) {}

bool BaroSensor::begin() {
    if (!_bmp.begin_I2C(_addr)) return false;
    _bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
    _bmp.setPressureOversampling(BMP3_NO_OVERSAMPLING);
    _bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    _bootMs = millis();
    return true;
}

void BaroSensor::resetCalibration() {
    // Skip WARMUP (hardware already at temperature); jump straight to SETTLING.
    // _ema carries the last locked pressure value — it converges fully within
    // the 10-second SETTLING window regardless of its starting value.
    _phaseStart = millis();
    _phase      = BmpPhase::SETTLING;
}

void BaroSensor::update(SensorData &out) {
    uint32_t now = millis();

    // Rate-limit to 50 Hz (20 ms) — forced mode read blocks ~2 ms,
    // so gating prevents hammering the I2C bus unnecessarily
    if (now - _lastPoll < 20) return;
    _lastPoll = now;

    switch (_phase) {
        case BmpPhase::WARMUP:
            // Discard readings while sensor die temperature stabilises
            if (now - _bootMs >= WARMUP_MS) {
                if (_bmp.performReading()) {
                    _ema        = _bmp.pressure;
                    _phaseStart = millis();
                    _phase      = BmpPhase::SETTLING;
                }
            }
            break;

        case BmpPhase::SETTLING:
            if (_bmp.performReading()) {
                _ema = EMA_ALPHA * _bmp.pressure + (1.0f - EMA_ALPHA) * _ema;
                uint32_t elapsed = millis() - _phaseStart;
                if (elapsed >= SETTLE_MS) {
                    _groundPa = _ema;
                    _phase    = BmpPhase::LOCKED;
                }
                out.bmpSettleRemSec = (elapsed < SETTLE_MS)
                                      ? (SETTLE_MS - elapsed) / 1000
                                      : 0;
            }
            break;

        case BmpPhase::LOCKED:
            if (_bmp.performReading()) {
                out.relAltM = _bmp.readAltitude(_groundPa / 100.0f);
            }
            break;
    }

    out.bmpPhase = _phase;
}

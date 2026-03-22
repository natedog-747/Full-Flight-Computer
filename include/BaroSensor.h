#pragma once
#include <Adafruit_BMP3XX.h>
#include "SensorData.h"

class BaroSensor {
public:
    explicit BaroSensor(uint8_t i2cAddr = 0x77);

    bool begin();

    // Call every loop iteration; internally rate-limits to 50 Hz.
    // Writes relAltM, bmpPhase, bmpSettleRemSec into out.
    void update(SensorData &out);

    BmpPhase phase()    const { return _phase; }
    float    groundPa() const { return _groundPa; }

private:
    Adafruit_BMP3XX _bmp;
    uint8_t         _addr;

    BmpPhase  _phase      = BmpPhase::WARMUP;
    uint32_t  _bootMs     = 0;
    uint32_t  _phaseStart = 0;
    uint32_t  _lastPoll   = 0;
    float     _ema        = 0;
    float     _groundPa   = 0;

    static constexpr uint32_t WARMUP_MS = 2000;
    static constexpr uint32_t SETTLE_MS = 10000;
    static constexpr float    EMA_ALPHA = 0.01f;
};

#pragma once

#include <SPI.h>
#include "SdFat.h"
#include "SensorData.h"

#define SD_CS_PIN 23  // PIN_SD_DAT3_CS on Adafruit Feather RP2040 Adalogger

class SdLogger {
public:
    bool begin();
    void log(const SensorData &data);

    // Write a one-shot event line (EVENT,<ms>,<msg>) to SD and Serial.
    void logEvent(const char *msg);

private:
    SdFat    _sd;
    File32   _file;
    SdSpiConfig _config{SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1};
    char     _fname[16]   = {};
    bool     _ready       = false;
    uint32_t _lastPrintMs = 0;
    uint32_t _rowCount    = 0;
};

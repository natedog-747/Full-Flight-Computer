#include <SPI.h>
#include "SdFat.h"
#include "SdLogger.h"
#include <Arduino.h>

bool SdLogger::begin() {
    Serial.print("Initializing SD card...");
    for (int attempt = 1; attempt <= 3; attempt++) {
        if (_sd.begin(_config)) break;
        Serial.printf("initialization failed (attempt %d/3)\n", attempt);
        if (attempt == 3) return false;
        delay(500);
    }
    Serial.println("initialization done.");

    // Pick next unused filename
    for (int i = 1; i <= 9999; i++) {
        snprintf(_fname, sizeof(_fname), "FLT%04d.CSV", i);
        if (!_sd.exists(_fname)) break;
    }

    _file = _sd.open(_fname, FILE_WRITE);
    if (_file) {
        Serial.printf("Logging to %s\n", _fname);
        _file.println("ms,ax,ay,az,gx,gy,gz,calSys,calGyro,calAccel,calMag,"
                      "relAltM,bmpPhase,bmpSettleRemSec,"
                      "gpsFix,gpsOrigin,gpsAvgRemSec,nedN,nedE,nedD,"
                      "velN_ms,velE_ms,velD_ms,gpsSpeedMs,gpsHeadingDeg,gpsSats,gpsHDOP,"
                      "dtMs");
        _file.close();
        _ready = true;
    } else {
        Serial.println("error opening file.");
    }

    return _ready;
}

void SdLogger::log(const SensorData &data) {
    static const char *phases[] = { "WARMUP", "SETTLING", "LOCKED" };
    uint32_t now = millis();

    if (_ready) {
        _file = _sd.open(_fname, FILE_WRITE);
        if (_file) {
            _file.printf(
                "%lu,"
                "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                "%u,%u,%u,%u,"
                "%.3f,%s,%lu,"
                "%u,%u,%lu,%.3f,%.3f,%.3f,"
                "%.3f,%.3f,%.3f,%.3f,%.3f,%u,%.2f,"
                "%.3f\n",
                data.timestampMs,
                data.ax,  data.ay,  data.az,
                data.gx,  data.gy,  data.gz,
                data.calSys, data.calGyro, data.calAccel, data.calMag,
                data.relAltM, phases[(uint8_t)data.bmpPhase], data.bmpSettleRemSec,
                (uint8_t)data.gpsFix, (uint8_t)data.gpsOrigin, data.gpsAvgRemSec,
                data.nedN, data.nedE, data.nedD,
                data.velN_ms, data.velE_ms, data.velD_ms,
                data.gpsSpeedMs, data.gpsHeadingDeg, data.gpsSats, data.gpsHDOP,
                data.dtMs);
            _file.close();
            _rowCount++;
        }
    }

    if (now - _lastPrintMs >= 250) {
        _lastPrintMs = now;
        const char *tag = _ready ? "[SD+SER]" : "[SER]   ";

        // IMU + baro line
        Serial.printf(
            "%s ax=%6.2f ay=%6.2f az=%6.2f | gx=%7.2f gy=%7.2f gz=%7.2f | "
            "alt=%+7.3fm [%s] | dt=%.2fms | rows=%lu\n",
            tag,
            data.ax,  data.ay,  data.az,
            data.gx,  data.gy,  data.gz,
            data.relAltM, phases[(uint8_t)data.bmpPhase],
            data.dtMs, _rowCount);

        // GPS line
        if (!data.gpsFix) {
            Serial.printf("%s GPS: NO FIX\n", tag);
        } else if (!data.gpsOrigin) {
            Serial.printf(
                "%s GPS: FIX %usats HDOP=%.2f | AVERAGING ORIGIN (%lus rem) | "
                "vN=%+6.2f vE=%+6.2f m/s spd=%5.2fm/s hdg=%5.1fdeg\n",
                tag, data.gpsSats, data.gpsHDOP, data.gpsAvgRemSec,
                data.velN_ms, data.velE_ms,
                data.gpsSpeedMs, data.gpsHeadingDeg);
        } else {
            Serial.printf(
                "%s GPS: N=%+8.3fm E=%+8.3fm D=%+7.3fm | "
                "vN=%+6.2f vE=%+6.2f vD=%+6.2f m/s | "
                "spd=%5.2fm/s hdg=%5.1fdeg | %usats HDOP=%.2f\n",
                tag,
                data.nedN, data.nedE, data.nedD,
                data.velN_ms, data.velE_ms, data.velD_ms,
                data.gpsSpeedMs, data.gpsHeadingDeg,
                data.gpsSats, data.gpsHDOP);
        }
    }
}

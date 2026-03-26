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
                      "gpsFix,gpsLat_rad,gpsLon_rad,gpsAlt_m,"
                      "velN_ms,velE_ms,velD_ms,gpsSpeedMs,gpsHeadingDeg,gpsSats,gpsHDOP,"
                      "dtMs,qw,qx,qy,qz,roll,pitch,yaw,"
                      "kfLat_rad,kfLon_rad,kfAlt_m,kfVelN,kfVelE,kfVelD,kfBaroBias,"
                      "ctrlEngaged,engagePulse_us,rcA_us,rcB_us,servoA_us,servoB_us,"
                      "errYaw_deg,integA_degs,errAlt_m,integB_ms,refAlt_m");
        _file.close();
        _ready = true;
    } else {
        Serial.println("error opening file.");
    }

    return _ready;
}

void SdLogger::logEvent(const char *msg) {
    uint32_t now = millis();
    Serial.printf("[EVENT] %lu ms — %s\n", now, msg);
    if (_ready) {
        _file = _sd.open(_fname, FILE_WRITE);
        if (_file) {
            _file.printf("EVENT,%lu,%s\n", now, msg);
            _file.close();
        }
    }
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
                "%u,%.8f,%.8f,%.3f,"
                "%.3f,%.3f,%.3f,%.3f,%.3f,%u,%.2f,"
                "%.3f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,"
                "%.8f,%.8f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                data.timestampMs,
                data.ax,  data.ay,  data.az,
                data.gx,  data.gy,  data.gz,
                data.calSys, data.calGyro, data.calAccel, data.calMag,
                data.relAltM, phases[(uint8_t)data.bmpPhase], data.bmpSettleRemSec,
                (uint8_t)data.gpsFix,
                data.gpsLat, data.gpsLon, data.gpsAlt,
                data.velN_ms, data.velE_ms, data.velD_ms,
                data.gpsSpeedMs, data.gpsHeadingDeg, data.gpsSats, data.gpsHDOP,
                data.dtMs,
                data.qw, data.qx, data.qy, data.qz,
                data.roll, data.pitch, data.yaw,
                data.kfLat, data.kfLon, data.kfAlt,
                data.kfVelN, data.kfVelE, data.kfVelD,
                data.kfBaroBias);
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

        // Attitude line
        static const char *kfPhase[] = { "ACCEL_AVG", "AWAIT_YAW", "READY" };
        const char *phase = (data.kfInitPhase <= 2) ? kfPhase[data.kfInitPhase] : "?";
        Serial.printf(
            "%s [KF:%s] roll=%+7.2f pitch=%+7.2f yaw=%+7.2f deg | "
            "q=[%+.4f %+.4f %+.4f %+.4f]\n",
            tag, phase,
            data.roll, data.pitch, data.yaw,
            data.qw, data.qx, data.qy, data.qz);

        // KF navigation line
        Serial.printf(
            "%s [KF:NAV] lat=%.6f lon=%.6f alt=%.1fm | "
            "vN=%+6.2f vE=%+6.2f vD=%+6.2f m/s | baroBias=%+.3fm\n",
            tag,
            data.kfLat * (180.0f / 3.14159265f),
            data.kfLon * (180.0f / 3.14159265f),
            data.kfAlt,
            data.kfVelN, data.kfVelE, data.kfVelD,
            data.kfBaroBias);

        // GPS line
        if (!data.gpsFix) {
            Serial.printf("%s GPS: NO FIX\n", tag);
        } else {
            Serial.printf(
                "%s GPS: lat=%.6f lon=%.6f alt=%.1fm | "
                "vN=%+6.2f vE=%+6.2f vD=%+6.2f m/s | "
                "spd=%5.2fm/s hdg=%5.1fdeg | %usats HDOP=%.2f\n",
                tag,
                data.gpsLat * (180.0f / 3.14159265f),
                data.gpsLon * (180.0f / 3.14159265f),
                data.gpsAlt,
                data.velN_ms, data.velE_ms, data.velD_ms,
                data.gpsSpeedMs, data.gpsHeadingDeg,
                data.gpsSats, data.gpsHDOP);
        }
    }
}

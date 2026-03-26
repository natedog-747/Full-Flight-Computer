#include "GpsSensor.h"
#include <math.h>

GpsSensor::GpsSensor(HardwareSerial &serial)
    : _gps(&serial) {}

void GpsSensor::begin() {
    _gps.begin(9600);
    _gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    _gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    _gps.sendCommand(PGCMD_ANTENNA);
}

void GpsSensor::update(SensorData &out) {
    _gps.read();
    if (!_gps.newNMEAreceived()) return;
    if (!_gps.parse(_gps.lastNMEA())) return;

    out.gpsFix    = _gps.fix;
    out.gpsSats   = _gps.satellites;
    // HDOP is only present in GGA sentences; RMC parses leave _gps.HDOP at 0.
    // Only overwrite when the library gives a positive value so the field holds
    // its last valid reading rather than glitching to 0 on every RMC sentence.
    if (_gps.HDOP > 0.0f) out.gpsHDOP = _gps.HDOP;
    out.gpsSpeedMs    = _gps.speed * 0.514444f;
    out.gpsHeadingDeg = _gps.angle;

    if (!_gps.fix || _gps.fixquality < MIN_FIX_QUALITY) return;

    // ── NED velocity from speed/course over ground ────────────────────────────
    float cogRad  = _gps.angle * (float)(M_PI / 180.0);
    out.velN_ms   = out.gpsSpeedMs * cosf(cogRad);
    out.velE_ms   = out.gpsSpeedMs * sinf(cogRad);
    out.velD_ms   = 0.0f;

    // ── Geodetic position (radians) ───────────────────────────────────────────
    out.gpsLat     = _gps.latitudeDegrees  * (float)(M_PI / 180.0);
    out.gpsLon     = _gps.longitudeDegrees * (float)(M_PI / 180.0);
    out.gpsAlt     = _gps.altitude;
    out.gpsNewData = true;
}

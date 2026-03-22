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

    out.gpsFix        = _gps.fix;
    out.gpsSats       = _gps.satellites;
    // HDOP is only present in GGA sentences; RMC parses leave _gps.HDOP at 0.
    // Only overwrite when the library gives a positive value so the field holds
    // its last valid reading rather than glitching to 0 on every RMC sentence.
    if (_gps.HDOP > 0.0f) out.gpsHDOP = _gps.HDOP;
    out.gpsSpeedMs    = _gps.speed * 0.514444f;
    out.gpsHeadingDeg = _gps.angle;

    // Need at least a basic fix to do anything position-related
    if (!_gps.fix || _gps.fixquality < MIN_FIX_QUALITY) return;

    // ── NED velocity from speed/course over ground ────────────────────────────
    // The MTK3339 outputs SOG (knots) and COG (degrees true) in the RMC sentence.
    // North and East components are simple trig; Down is not in standard NMEA.
    {
        float cogRad  = _gps.angle * (float)(M_PI / 180.0);
        out.velN_ms   = out.gpsSpeedMs * cosf(cogRad);
        out.velE_ms   = out.gpsSpeedMs * sinf(cogRad);
        out.velD_ms   = 0.0f;
    }

    // ── Origin averaging ─────────────────────────────────────────────────────
    // The HDOP gate was intentionally removed from this block.
    // The original code gated the elapsed-time check on HDOP <= MAX_HDOP,
    // so any fluctuation above 2.0 at the 10-second mark prevented the origin
    // from ever locking.  We accumulate all basic-fix samples, let the 10-s
    // average wash out noise, and lock unconditionally once the window closes.
    if (!_originSet) {
        if (!_averaging) {
            _averaging   = true;
            _avgStart    = millis();
            _sumLat = _sumLon = _sumAlt = 0;
            _sampleCount = 0;
        }

        _sumLat += _gps.latitudeDegrees;
        _sumLon += _gps.longitudeDegrees;
        _sumAlt += _gps.altitude;
        _sampleCount++;

        uint32_t elapsed = millis() - _avgStart;
        out.gpsAvgRemSec = (elapsed < AVG_DURATION_MS)
                           ? (AVG_DURATION_MS - elapsed) / 1000
                           : 0;

        if (elapsed < AVG_DURATION_MS) {
            out.gpsOrigin = false;
            return;
        }

        // 10 s elapsed — lock origin and fall through to compute NED now
        _originLat = _sumLat / _sampleCount;
        _originLon = _sumLon / _sampleCount;
        _originAlt = (float)(_sumAlt / _sampleCount);
        _originSet = true;
    }

    // ── NED displacement ─────────────────────────────────────────────────────
    out.gpsOrigin = true;
    _computeNED(out);
}

void GpsSensor::_computeNED(SensorData &out) const {
    double dLat = (_gps.latitudeDegrees  - _originLat) * (M_PI / 180.0);
    double dLon = (_gps.longitudeDegrees - _originLon) * (M_PI / 180.0);
    out.nedN = (float)(dLat * R_EARTH);
    out.nedE = (float)(dLon * R_EARTH * cos(_originLat * (M_PI / 180.0)));
    out.nedD = -(_gps.altitude - _originAlt);
}

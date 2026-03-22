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

    out.gpsFix      = _gps.fix;
    out.gpsSats     = _gps.satellites;
    out.gpsHDOP     = _gps.HDOP;
    out.gpsSpeedMs  = _gps.speed * 0.514444f;
    out.gpsHeadingDeg = _gps.angle;

    if (!_gps.fix || _gps.fixquality < MIN_FIX_QUALITY || _gps.HDOP > MAX_HDOP) return;

    // Average 10 s of good-fix readings to establish the NED origin
    if (!_originSet) {
        if (!_averaging) {
            _averaging    = true;
            _avgStart     = millis();
            _sumLat = _sumLon = _sumAlt = 0;
            _sampleCount  = 0;
        }
        _sumLat += _gps.latitudeDegrees;
        _sumLon += _gps.longitudeDegrees;
        _sumAlt += _gps.altitude;
        _sampleCount++;

        if (millis() - _avgStart >= AVG_DURATION_MS) {
            _originLat  = _sumLat / _sampleCount;
            _originLon  = _sumLon / _sampleCount;
            _originAlt  = (float)(_sumAlt / _sampleCount);
            _originSet  = true;
        }
        out.gpsOrigin = false;
        return;
    }

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

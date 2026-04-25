#include "FlightStateMachine.h"
#include <Arduino.h>  // millis()

void FlightStateMachine::update(bool imuCalibrating, float roll, float pitch, float accelNorm, float accelX) {
    // Always recompute so getLedColor() reflects current attitude even on the
    // transition frame from CALIBRATION → STANDBY.
    _withinAttitudeLimits = (fabsf(roll) <= MAX_ROLL_DEG && fabsf(pitch) <= MAX_PITCH_DEG);

    switch (state) {
        case FlightState::CALIBRATION:
            if (!imuCalibrating) {
                state = FlightState::STANDBY;
            }
            break;

        case FlightState::STANDBY:
            if (_withinAttitudeLimits && accelX >= LAUNCH_ACCEL_X_MS2) {
                _flightEntryMs    = millis();
                _resetTriggeredMs = 0;
                state = FlightState::FLIGHT;
            }
            break;

        case FlightState::FLIGHT: {
            uint32_t now = millis();
            if (_resetTriggeredMs == 0 &&
                accelNorm >= RESET_ACCEL_MS2 &&
                (now - _flightEntryMs) >= FLIGHT_COOLDOWN_MS) {
                _resetTriggeredMs = now;   // start the settling wait
            }
            if (_resetTriggeredMs != 0 && (now - _resetTriggeredMs) >= RESET_SETTLE_MS) {
                _resetTriggeredMs = 0;
                state = FlightState::CALIBRATION;
            }
            break;
        }
    }
}

uint32_t FlightStateMachine::getLedColor() const {
    switch (state) {
        case FlightState::CALIBRATION:
            return 0xFF0000u;  // red
        case FlightState::STANDBY:
            return _withinAttitudeLimits ? 0x00FF00u : 0xFF8000u;  // green : orange
        case FlightState::FLIGHT:
            return 0x0000FFu;  // blue
        default:
            return 0x000000u;
    }
}

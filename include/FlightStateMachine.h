#pragma once
#include <stdint.h>
#include <math.h>

enum class FlightState { CALIBRATION, STANDBY, FLIGHT };

class FlightStateMachine {
public:
    // Thresholds (all in SI units / degrees)
    static constexpr float LAUNCH_ACCEL_X_MS2 = 5.0f;  // body-X (nose) accel to trigger STANDBY → FLIGHT
    static constexpr float RESET_ACCEL_MS2  = 20.0f;  // norm to trigger FLIGHT → CALIBRATION
    static constexpr float MAX_ROLL_DEG     = 20.0f;  // ±limit for launch-ready window
    static constexpr float    MAX_PITCH_DEG      = 20.0f;
    static constexpr uint32_t FLIGHT_COOLDOWN_MS = 2000;   // ms after entering FLIGHT before high-G reset is armed
    static constexpr uint32_t RESET_SETTLE_MS    = 2000;   // ms to wait after high-G trigger before transitioning to CALIBRATION

    FlightState state = FlightState::CALIBRATION;

    // Call once per loop.  imuCalibrating comes from ImuSensor::calibrating.
    void update(bool imuCalibrating, float roll, float pitch, float accelNorm, float accelX);

    // Returns a packed 0x00RRGGBB color for the NeoPixel.
    uint32_t getLedColor() const;

private:
    bool     _withinAttitudeLimits = false;
    uint32_t _flightEntryMs        = 0;
    uint32_t _resetTriggeredMs     = 0;   // 0 = no pending reset
};

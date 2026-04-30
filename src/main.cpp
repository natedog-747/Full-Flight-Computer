#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include "ImuSensor.h"
#include "FlightStateMachine.h"
#include "Control.h"

#ifndef PIN_NEOPIXEL
#define PIN_NEOPIXEL 8
#endif

static const uint32_t LOOP_HZ = 20;
static const uint32_t LOOP_MS = 50 / LOOP_HZ;

Adafruit_NeoPixel   pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
ImuSensor           imuSensor;
FlightStateMachine  stateMachine;
Control             control;

static TickType_t xLastWakeTime;

// ── Core 0 ───────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(3000);
    Serial.println("=== BOOT Core 0 ===");

#ifdef NEOPIXEL_POWER
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
#endif

    pixel.begin();
    pixel.setBrightness(50);
    pixel.show();

    analogReadResolution(12);
    Wire.begin();
    if (!imuSensor.begin()) {
        while (1) vTaskDelay(portMAX_DELAY);
    }
    control.begin();
    Serial.println("BNO055 ready");
    Serial.println("Hold still — calibrating 10 s...");

    xLastWakeTime = xTaskGetTickCount();
}

void loop() {
    FlightState prevState = stateMachine.state;

    imuSensor.update();
    stateMachine.update(imuSensor.calibrating, imuSensor.roll, imuSensor.pitch, imuSensor.accelNorm, imuSensor.accelX);
    control.update(imuSensor.pitch, imuSensor.pitchRate,
                   imuSensor.yaw,   imuSensor.yawRate,
                   imuSensor.accelX, stateMachine.state == FlightState::FLIGHT);

    // On FLIGHT → CALIBRATION transition, restart the IMU calibration routine
    if (prevState == FlightState::FLIGHT && stateMachine.state == FlightState::CALIBRATION) {
        imuSensor.restartCalibration();
        Serial.println("High-G event — recalibrating...");
    }

    // NeoPixel reflects flight state
    pixel.setPixelColor(0, pixel.gamma32(stateMachine.getLedColor()));
    pixel.show();

    // Serial orientation output while not calibrating
    if (stateMachine.state != FlightState::CALIBRATION) {
        static uint32_t lastPrintMs = 0;
        uint32_t now = millis();
        if (now - lastPrintMs >= 100) {
            lastPrintMs = now;
            Serial.print("State: ");
            Serial.print(stateMachine.state == FlightState::STANDBY ? "STANDBY" : "FLIGHT");
            Serial.print("  Roll: ");    Serial.print(imuSensor.roll,  2);
            Serial.print("  Pitch: ");  Serial.print(imuSensor.pitch, 2);
            Serial.print("  Yaw: ");    Serial.print(imuSensor.yaw,   2);
            Serial.print("  |a|: ");      Serial.print(imuSensor.accelNorm, 2);
            Serial.print("  PRate: ");    Serial.print(imuSensor.pitchRate, 1);
            Serial.print("  PErr: ");     Serial.print(control.dbgPitchErr, 2);
            Serial.print("  P: ");        Serial.print(control.dbgPitchP, 2);
            Serial.print("  I: ");        Serial.print(control.dbgPitchI, 2);
            Serial.print("  D: ");        Serial.print(control.dbgPitchD, 2);
            float battV = (analogRead(A3) / 4095.0f) * 3.3f * 2.0f;
            Serial.print("  PSrv: ");     Serial.print(control.pitchOut);
            Serial.print("  YSrv: ");     Serial.print(control.yawOut);
            Serial.print("  Batt: ");     Serial.print(battV, 2);
            Serial.println("V");
        }
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(LOOP_MS));
}

// ── Core 1 ───────────────────────────────────────────────────────────────────

void setup1() {
    Serial.println("=== BOOT Core 1 ===");
}

void loop1() {
    vTaskDelay(portMAX_DELAY);
}

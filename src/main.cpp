#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "SensorData.h"
#include "GpsSensor.h"
#include "ImuSensor.h"
#include "BaroSensor.h"
#include "SdLogger.h"

static GpsSensor  gGps(Serial1);
static ImuSensor  gImu(Wire);
static BaroSensor gBaro;
static SdLogger   gLogger;

static SensorData gData;

static uint32_t lastImuMs  = 0;
static uint32_t lastBaroMs = 0;
static uint32_t lastCalMs  = 0;
static uint32_t prevLoopUs = 0;

void setup() {
    Serial.begin(9600);
    delay(3000);
    Serial.println("=== BOOT ===");

    Wire.begin();
    Wire.setClock(400000);

    gGps.begin();

    if (!gImu.begin())  { Serial.println("BNO055 FAIL"); while (1); }
    if (!gBaro.begin()) { Serial.println("BMP390 FAIL"); while (1); }
    if (!gLogger.begin()) { Serial.println("SD FAIL — continuing without SD"); }

    prevLoopUs = micros();
    Serial.println("=== RUNNING ===");
}

void loop() {
    Serial.println("=== Running ===");
    uint32_t loopStart = micros();
    gData.dtMs = (loopStart - prevLoopUs) / 1000.0f;
    prevLoopUs = loopStart;

    uint32_t now = millis();
    gData.timestampMs = now;

    // GPS — non-blocking, every iteration
    gGps.update(gData);

    // IMU — 100 Hz
    if (now - lastImuMs >= 10) {
        lastImuMs = now;
        gImu.read(gData);
    }

    // Calibration — 1 Hz
    if (now - lastCalMs >= 1000) {
        lastCalMs = now;
        gImu.getCalibration(gData.calSys, gData.calGyro, gData.calAccel, gData.calMag);
    }

    // Barometer — 50 Hz
    if (now - lastBaroMs >= 20) {
        lastBaroMs = now;
        gBaro.update(gData);
    }

    // Log to SD + serial
    gLogger.log(gData);
}

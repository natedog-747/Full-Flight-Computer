#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <Servo.h>

#include "SensorData.h"
#include "GpsSensor.h"
#include "ImuSensor.h"
#include "BaroSensor.h"
#include "SdLogger.h"
#include "KalmanFilter.h"

// ── Configurable update / log rates (milliseconds) ───────────────────────────
#define IMU_RATE_MS     10   // 100 Hz — also sets how often the GPS UART is drained
#define BARO_RATE_MS   100   //  10 Hz
#define LOG_RATE_MS     10   // 100 Hz

// ── Peripherals ───────────────────────────────────────────────────────────────
// Core 0 owns: Wire (I2C → IMU + Baro) and Serial1 (UART → GPS)
static GpsSensor  gGps(Serial1);
static ImuSensor  gImu(Wire);
static BaroSensor gBaro;
// Core 1 owns: SPI1 (SD card), servo passthrough
static SdLogger   gLogger;

// ── Servo passthrough (Core 1) ────────────────────────────────────────────────
#define SERVO_IN_PIN   12
#define SERVO_OUT_PIN  10

static Servo            gServoOut;
static volatile uint32_t gPulseStart   = 0;
static volatile uint32_t gPulseWidthUs = 1500;  // safe default: centre

static void onServoIn() {
    if (digitalRead(SERVO_IN_PIN)) {
        gPulseStart = micros();
    } else {
        uint32_t w = micros() - gPulseStart;
        if (w >= 800 && w <= 2200) gPulseWidthUs = w;
    }
}
// Kalman filter — owned exclusively by Core 0 sensor task (no mutex needed)
static KalmanFilter gKF;

// ── Shared sensor snapshot (Core 0 writes, Core 1 reads) ─────────────────────
static SensorData        gShared;
static SemaphoreHandle_t gDataMutex = nullptr;

// ── Core 0 sensor task ────────────────────────────────────────────────────────
// All Core 0 buses are handled in a single task:
//   - GPS UART is drained first each cycle (cheap: ~10 bytes at 9600 baud / 1 Hz)
//   - IMU and Baro follow on I2C
// This eliminates the separate GPS task that was prone to stack-overflow hangs
// (Adafruit GPS parse() calls atof/atoi, which blew the old 512-word stack).
static void taskSensors(void *) {
    TickType_t wake      = xTaskGetTickCount();
    SensorData gpsLocal  = {};
    SensorData imuLocal  = {};
    SensorData baroLocal = {};
    uint32_t lastCalMs   = 0;
    uint32_t lastBaroMs  = 0;
    bool     baroFresh   = false;
    uint32_t prevUs      = micros();

    for (;;) {
        uint32_t nowMs = millis();
        uint32_t nowUs = micros();

        // ── GPS: drain UART buffer ──────────────────────────────────────────
        // At 9600 baud / 1 Hz, at most ~10 bytes arrive per 10 ms cycle.
        // Looping on Serial1.available() consumes exactly what is there.
        while (Serial1.available()) {
            gGps.update(gpsLocal);
        }

        // GPS Kalman measurement update — only when a fresh NED fix is available
        if (gpsLocal.gpsNewData && gpsLocal.gpsOrigin) {
            gKF.updateFromGPS(gpsLocal.nedN, gpsLocal.nedE, gpsLocal.nedD,
                              gpsLocal.velN_ms, gpsLocal.velE_ms,
                              gpsLocal.gpsHDOP,
                              gpsLocal.gpsHeadingDeg, gpsLocal.gpsSpeedMs);
            gpsLocal.gpsNewData = false;
        }

        // ── IMU: 100 Hz ─────────────────────────────────────────────────────
        gImu.read(imuLocal);
        imuLocal.timestampMs = nowMs;
        imuLocal.dtMs        = (nowUs - prevUs) / 1000.0f;
        prevUs               = nowUs;

        // ── NED axis flip + KF update ────────────────────────────────────────
        // Sensor frame: x=nose, y=left wing, z=up
        // NED frame:    x=nose, y=right wing, z=down  →  negate y and z
        {
            const float kDeg2Rad = 3.14159265f / 180.0f;
            float gxNed =  imuLocal.gx * kDeg2Rad;
            float gyNed = -imuLocal.gy * kDeg2Rad;
            float gzNed = -imuLocal.gz * kDeg2Rad;
            float axNed =  imuLocal.ax;
            float ayNed = -imuLocal.ay;
            float azNed = -imuLocal.az;

            // Gyro + accel averaging for bias/roll/pitch init (phase-guarded inside)
            gKF.feedImu(gxNed, gyNed, gzNed, axNed, ayNed, azNed);

            // GPS heading for yaw initialisation (phase-guarded inside)
            if (gpsLocal.gpsFix) {
                gKF.feedGpsHeading(gpsLocal.gpsSpeedMs, gpsLocal.gpsHeadingDeg);
            }

            // Gyro integration — no-op during ACCEL_AVG phase
            gKF.predict(gxNed, gyNed, gzNed, axNed, ayNed, azNed,
                        imuLocal.dtMs * 0.001f);

            gKF.getQuaternion(imuLocal.qw, imuLocal.qx, imuLocal.qy, imuLocal.qz);
            gKF.getEulerDeg(imuLocal.roll, imuLocal.pitch, imuLocal.yaw);
            gKF.getPosition(imuLocal.kfPosN, imuLocal.kfPosE, imuLocal.kfPosD);
            gKF.getVelocity(imuLocal.kfVelN, imuLocal.kfVelE, imuLocal.kfVelD);
            imuLocal.kfInitPhase = (uint8_t)gKF.getInitPhase();
            imuLocal.kfBaroBias  = gKF.getBaroBias();
        }

        if (nowMs - lastCalMs >= 1000) {
            lastCalMs = nowMs;
            gImu.getCalibration(imuLocal.calSys, imuLocal.calGyro,
                                imuLocal.calAccel, imuLocal.calMag);
        }

        // ── Baro: BARO_RATE_MS ───────────────────────────────────────────────
        baroFresh = false;
        if (nowMs - lastBaroMs >= BARO_RATE_MS) {
            lastBaroMs = nowMs;
            gBaro.update(baroLocal);
            baroFresh = true;
        }

        // Baro Kalman measurement update — only when locked and fresh
        if (baroFresh && baroLocal.bmpPhase == BmpPhase::LOCKED) {
            gKF.updateFromBarometer(baroLocal.relAltM);
        }

        // ── Publish to shared snapshot ───────────────────────────────────────
        if (xSemaphoreTake(gDataMutex, 0) == pdTRUE) {
            gShared.gpsFix        = gpsLocal.gpsFix;
            gShared.gpsOrigin     = gpsLocal.gpsOrigin;
            gShared.gpsAvgRemSec  = gpsLocal.gpsAvgRemSec;
            gShared.nedN          = gpsLocal.nedN;
            gShared.nedE          = gpsLocal.nedE;
            gShared.nedD          = gpsLocal.nedD;
            gShared.velN_ms       = gpsLocal.velN_ms;
            gShared.velE_ms       = gpsLocal.velE_ms;
            gShared.velD_ms       = gpsLocal.velD_ms;
            gShared.gpsSpeedMs    = gpsLocal.gpsSpeedMs;
            gShared.gpsHeadingDeg = gpsLocal.gpsHeadingDeg;
            gShared.gpsSats       = gpsLocal.gpsSats;
            gShared.gpsHDOP       = gpsLocal.gpsHDOP;
            gShared.ax            = imuLocal.ax;
            gShared.ay            = imuLocal.ay;
            gShared.az            = imuLocal.az;
            gShared.gx            = imuLocal.gx;
            gShared.gy            = imuLocal.gy;
            gShared.gz            = imuLocal.gz;
            gShared.calSys        = imuLocal.calSys;
            gShared.calGyro       = imuLocal.calGyro;
            gShared.calAccel      = imuLocal.calAccel;
            gShared.calMag        = imuLocal.calMag;
            gShared.timestampMs   = imuLocal.timestampMs;
            gShared.dtMs          = imuLocal.dtMs;
            gShared.kfInitPhase   = imuLocal.kfInitPhase;
            gShared.qw            = imuLocal.qw;
            gShared.qx            = imuLocal.qx;
            gShared.qy            = imuLocal.qy;
            gShared.qz            = imuLocal.qz;
            gShared.roll          = imuLocal.roll;
            gShared.pitch         = imuLocal.pitch;
            gShared.yaw           = imuLocal.yaw;
            gShared.kfPosN        = imuLocal.kfPosN;
            gShared.kfPosE        = imuLocal.kfPosE;
            gShared.kfPosD        = imuLocal.kfPosD;
            gShared.kfVelN        = imuLocal.kfVelN;
            gShared.kfVelE        = imuLocal.kfVelE;
            gShared.kfVelD        = imuLocal.kfVelD;
            gShared.kfBaroBias    = imuLocal.kfBaroBias;
            gShared.relAltM         = baroLocal.relAltM;
            gShared.bmpPhase        = baroLocal.bmpPhase;
            gShared.bmpSettleRemSec = baroLocal.bmpSettleRemSec;
            xSemaphoreGive(gDataMutex);
        }

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(IMU_RATE_MS));
    }
}

// ── Core 0 ───────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(9600);
    delay(3000);
    Serial.println("=== BOOT Core 0 ===");

    gDataMutex = xSemaphoreCreateMutex();

    Wire.begin();
    Wire.setClock(400000);

    gGps.begin();
    if (!gImu.begin())  { Serial.println("BNO055 FAIL"); while (1); }
    if (!gBaro.begin()) { Serial.println("BMP390 FAIL"); while (1); }

    TaskHandle_t h;
    xTaskCreate(taskSensors, "SENS", 2048, nullptr, 4, &h);
    vTaskCoreAffinitySet(h, (1 << 0));

    Serial.println("Core 0 ready");
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}

// ── Core 1 ───────────────────────────────────────────────────────────────────

void setup1() {
    while (gDataMutex == nullptr) delay(1);

    if (!gLogger.begin()) {
        Serial.println("SD FAIL — logging to serial only");
    }

    // Servo passthrough: capture input on GPIO 13, drive output on GPIO 10
    pinMode(SERVO_IN_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(SERVO_IN_PIN), onServoIn, CHANGE);
    gServoOut.attach(SERVO_OUT_PIN, 800, 2200);

    Serial.println("Core 1 ready");
}

void loop1() {
    static TickType_t wake = xTaskGetTickCount();

    // Mirror servo input → output with a 3-sample median filter.
    // Rejects single-sample glitches with no lag on real signal changes.
    static uint32_t med[3] = {1500, 1500, 1500};
    med[0] = med[1]; med[1] = med[2]; med[2] = gPulseWidthUs;
    uint32_t a = med[0], b = med[1], c = med[2];
    if (a > b) { uint32_t t = a; a = b; b = t; }
    if (b > c) { uint32_t t = b; b = c; c = t; }
    if (a > b) { uint32_t t = a; a = b; b = t; }
    gServoOut.writeMicroseconds((int)b);  // b is the median

    SensorData snap;
    if (xSemaphoreTake(gDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        snap = gShared;
        xSemaphoreGive(gDataMutex);
        gLogger.log(snap);
    }

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOG_RATE_MS));
}

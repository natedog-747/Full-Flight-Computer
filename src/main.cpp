#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "SensorData.h"
#include "GpsSensor.h"
#include "ImuSensor.h"
#include "BaroSensor.h"
#include "SdLogger.h"

// ── Configurable update / log rates (milliseconds) ───────────────────────────
#define IMU_RATE_MS     10   // 100 Hz — also sets how often the GPS UART is drained
#define BARO_RATE_MS   100   //  10 Hz
#define LOG_RATE_MS     10   // 100 Hz

// ── Peripherals ───────────────────────────────────────────────────────────────
// Core 0 owns: Wire (I2C → IMU + Baro) and Serial1 (UART → GPS)
static GpsSensor  gGps(Serial1);
static ImuSensor  gImu(Wire);
static BaroSensor gBaro;
// Core 1 owns: SPI1 (SD card)
static SdLogger   gLogger;

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

        // ── IMU: 100 Hz ─────────────────────────────────────────────────────
        gImu.read(imuLocal);
        imuLocal.timestampMs = nowMs;
        imuLocal.dtMs        = (nowUs - prevUs) / 1000.0f;
        prevUs               = nowUs;

        if (nowMs - lastCalMs >= 1000) {
            lastCalMs = nowMs;
            gImu.getCalibration(imuLocal.calSys, imuLocal.calGyro,
                                imuLocal.calAccel, imuLocal.calMag);
        }

        // ── Baro: BARO_RATE_MS ───────────────────────────────────────────────
        if (nowMs - lastBaroMs >= BARO_RATE_MS) {
            lastBaroMs = nowMs;
            gBaro.update(baroLocal);
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
    xTaskCreate(taskSensors, "SENS", 1024, nullptr, 4, &h);
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
    Serial.println("Core 1 ready");
}

void loop1() {
    static TickType_t wake = xTaskGetTickCount();

    SensorData snap;
    if (xSemaphoreTake(gDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        snap = gShared;
        xSemaphoreGive(gDataMutex);
        gLogger.log(snap);
    }

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOG_RATE_MS));
}

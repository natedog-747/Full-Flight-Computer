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
#define GPS_RATE_MS     20   //  50 Hz  (UART buffer drained in bursts each cycle)
#define IMU_RATE_MS     10   // 100 Hz
#define BARO_RATE_MS    100   //  10 Hz  (BaroSensor also internally rate-limits)
#define LOG_RATE_MS     10   // 100 Hz

// ── Peripherals — each bus is owned by exactly one core ──────────────────────
// Core 0: Wire (I2C → IMU + Baro), Serial1 (GPS)
static GpsSensor  gGps(Serial1);
static ImuSensor  gImu(Wire);
static BaroSensor gBaro;
// Core 1: SPI1 (SD card)
static SdLogger   gLogger;

// ── Shared sensor snapshot (written by Core 0 tasks, read by Core 1) ─────────
static SensorData        gShared;
static SemaphoreHandle_t gDataMutex = nullptr;

// ── Core 0 sensor tasks ───────────────────────────────────────────────────────

// GPS — drains the UART buffer in a burst then sleeps for GPS_RATE_MS.
// GpsSensor::update() calls _gps.read() once (1 byte); at 9600 baud we
// accumulate ~GPS_RATE_MS bytes between wakeups, so we iterate 128 times to
// guarantee the buffer is fully drained before sleeping.
static void taskGps(void *) {
    TickType_t wake  = xTaskGetTickCount();
    SensorData local = {};

    for (;;) {
        for (int i = 0; i < 128; i++) {
            gGps.update(local);
        }

        if (xSemaphoreTake(gDataMutex, 0) == pdTRUE) {
            gShared.gpsFix        = local.gpsFix;
            gShared.gpsOrigin     = local.gpsOrigin;
            gShared.gpsAvgRemSec  = local.gpsAvgRemSec;
            gShared.nedN          = local.nedN;
            gShared.nedE          = local.nedE;
            gShared.nedD          = local.nedD;
            gShared.velN_ms       = local.velN_ms;
            gShared.velE_ms       = local.velE_ms;
            gShared.velD_ms       = local.velD_ms;
            gShared.gpsSpeedMs    = local.gpsSpeedMs;
            gShared.gpsHeadingDeg = local.gpsHeadingDeg;
            gShared.gpsSats       = local.gpsSats;
            gShared.gpsHDOP       = local.gpsHDOP;
            xSemaphoreGive(gDataMutex);
        }

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(GPS_RATE_MS));
    }
}

// I2C task — handles both IMU and Baro in a single task so they never
// contend for the Wire bus. FreeRTOS is preemptive; running IMU (higher
// priority) and Baro (lower priority) as separate tasks allowed taskImu to
// interrupt taskBaro mid-I2C transaction, corrupting BMP390 pressure reads
// during SETTLING and producing a wrong ground reference pressure.
static void taskI2C(void *) {
    TickType_t wake     = xTaskGetTickCount();
    SensorData imuLocal = {};
    SensorData baroLocal = {};
    uint32_t lastCalMs  = 0;
    uint32_t lastBaroMs = 0;
    uint32_t prevUs     = micros();

    for (;;) {
        uint32_t nowMs = millis();
        uint32_t nowUs = micros();

        // IMU — every cycle (IMU_RATE_MS)
        gImu.read(imuLocal);
        imuLocal.timestampMs = nowMs;
        imuLocal.dtMs        = (nowUs - prevUs) / 1000.0f;
        prevUs               = nowUs;

        if (nowMs - lastCalMs >= 1000) {
            lastCalMs = nowMs;
            gImu.getCalibration(imuLocal.calSys, imuLocal.calGyro,
                                imuLocal.calAccel, imuLocal.calMag);
        }

        // Baro — every BARO_RATE_MS, sequentially after IMU (no bus conflict)
        if (nowMs - lastBaroMs >= BARO_RATE_MS) {
            lastBaroMs = nowMs;
            gBaro.update(baroLocal);
        }

        if (xSemaphoreTake(gDataMutex, 0) == pdTRUE) {
            gShared.ax              = imuLocal.ax;
            gShared.ay              = imuLocal.ay;
            gShared.az              = imuLocal.az;
            gShared.gx              = imuLocal.gx;
            gShared.gy              = imuLocal.gy;
            gShared.gz              = imuLocal.gz;
            gShared.calSys          = imuLocal.calSys;
            gShared.calGyro         = imuLocal.calGyro;
            gShared.calAccel        = imuLocal.calAccel;
            gShared.calMag          = imuLocal.calMag;
            gShared.timestampMs     = imuLocal.timestampMs;
            gShared.dtMs            = imuLocal.dtMs;
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

    // Create sensor tasks and pin each to Core 0
    TaskHandle_t h;
    xTaskCreate(taskGps,  "GPS",  512, nullptr, 2, &h);
    vTaskCoreAffinitySet(h, (1 << 0));

    xTaskCreate(taskI2C, "I2C", 512, nullptr, 4, &h);
    vTaskCoreAffinitySet(h, (1 << 0));

    Serial.println("Core 0 tasks started");
}

// Sensor tasks handle everything; loop() yields forever so the scheduler
// can run the higher-priority tasks on this core.
void loop() {
    vTaskDelay(portMAX_DELAY);
}

// ── Core 1 ───────────────────────────────────────────────────────────────────

void setup1() {
    while (gDataMutex == nullptr) delay(1);

    if (!gLogger.begin()) {
        Serial.println("SD FAIL — logging to serial only");
    }
    Serial.println("Core 1 logger ready");
}

// Runs at LOG_RATE_MS using vTaskDelayUntil for precise timing.
// The static wake variable is initialised on the first call and updated
// by vTaskDelayUntil on every subsequent call.
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

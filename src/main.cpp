#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>

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
// Core 0 owns: Wire (I2C → IMU + Baro + PCA9685) and Serial1 (UART → GPS)
static GpsSensor  gGps(Serial1);
static ImuSensor  gImu(Wire);
static BaroSensor gBaro;
// Core 1 owns: SPI1 (SD card), servo passthrough
static SdLogger   gLogger;

// ── Servo passthrough (Core 1) ────────────────────────────────────────────────
#define SERVO_A_IN_PIN   12
#define SERVO_A_OUT_PIN  10
#define SERVO_B_IN_PIN   11
#define SERVO_B_OUT_PIN   9

static Servo             gServoOutA;
static volatile uint32_t gPulseStartA   = 0;
static volatile uint32_t gPulseWidthUsA = 1500;

static Servo             gServoOutB;
static volatile uint32_t gPulseStartB   = 0;
static volatile uint32_t gPulseWidthUsB = 1500;

static void onServoInA() {
    if (digitalRead(SERVO_A_IN_PIN)) {
        gPulseStartA = micros();
    } else {
        uint32_t w = micros() - gPulseStartA;
        if (w >= 800 && w <= 2200) gPulseWidthUsA = w;
    }
}

static void onServoInB() {
    if (digitalRead(SERVO_B_IN_PIN)) {
        gPulseStartB = micros();
    } else {
        uint32_t w = micros() - gPulseStartB;
        if (w >= 800 && w <= 2200) gPulseWidthUsB = w;
    }
}

// ── PCA9685 engage logic ──────────────────────────────────────────────────────
// GPIO 5 PWM duty cycle is measured on Core 1 (interrupt-based).
// When duty exceeds ENGAGE_THRESHOLD, Core 1 sets gEngageActive = true.
// Core 0's sensor task reads that flag and drives the PCA9685 via Wire.
// ALL I2C stays on Core 0 — cross-core Wire calls crash the RP2040.
#define ENGAGE_IN_PIN  5

// Servo pulse width threshold in microseconds.
// Engage when the input channel is above ~1700 µs (stick past 2/3 high).
// Standard RC range is 1000–2000 µs; 1700 µs is a reliable high-side threshold.
static constexpr uint32_t ENGAGE_THRESHOLD_US = 1700;
static constexpr uint8_t  PCA9685_FREQ_HZ     = 50;    // standard servo PWM freq

// Centre pulse = 1500 µs. At 50 Hz the PCA9685 period is 20 000 µs / 4096 counts.
// 1500 µs × 4096 / 20 000 µs = 307.2 → 307 counts.
// This is the universally accepted centre position — safe for all standard RC servos.
static constexpr uint16_t SERVO_CENTER_TICKS =
    (uint16_t)(1500UL * 4096UL / (1000000UL / PCA9685_FREQ_HZ));

// PCA9685 lives on Wire (I2C0) alongside BNO055 (0x28) and BMP390 (0x76) — no address clash.
static Adafruit_PWMServoDriver gPwmDriver(0x40, Wire);

// Cross-core engage flag: Core 1 writes, Core 0 reads.
// Volatile is sufficient — single-bit, 32-bit-aligned on Cortex-M0+.
static volatile bool gEngageActive = false;

// Duty-cycle measurement for the engage pin (Core 1 ISR).
// gEngageValid is false until we have seen at least one complete high pulse,
// preventing a false trigger if the first edge is a falling edge.
static volatile bool     gEngageValid    = false;
static volatile uint32_t gEngageRiseUs   = 0;
static volatile uint32_t gEngageHighUs   = 0;
static volatile uint32_t gEngagePeriodUs = 20000;  // safe default (50 Hz)

static void onEngagePin() {
    uint32_t now = micros();
    if (digitalRead(ENGAGE_IN_PIN)) {
        // Rising edge — update period estimate from last rise
        if (gEngageValid) {
            uint32_t period = now - gEngageRiseUs;
            if (period > 1000 && period < 100000)   // accept 10 Hz – 1 kHz
                gEngagePeriodUs = period;
        }
        gEngageRiseUs = now;
    } else {
        // Falling edge — capture high time (only after a rising edge has been seen)
        if (gEngageRiseUs != 0) {
            gEngageHighUs = now - gEngageRiseUs;
            gEngageValid  = true;
        }
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
    bool     lastEngaged = false;

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

        // ── PCA9685 engage — runs on Core 0 so Wire is never touched cross-core
        {
            bool engaged = gEngageActive;
            if (engaged != lastEngaged) {
                Serial.println(engaged ? "ENGAGE: ON" : "ENGAGE: OFF");
                if (engaged) {
                    for (uint8_t ch = 0; ch < 16; ch++)
                        gPwmDriver.setPWM(ch, 0, SERVO_CENTER_TICKS);
                }
                lastEngaged = engaged;
            }
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

    // PCA9685 shares Wire (I2C0) with IMU/baro — address 0x40 does not clash
    gPwmDriver.begin();
    gPwmDriver.setPWMFreq(PCA9685_FREQ_HZ);
    // Safe starting position: all channels to 90° centre (1500 µs)
    for (uint8_t ch = 0; ch < 16; ch++)
        gPwmDriver.setPWM(ch, 0, SERVO_CENTER_TICKS);

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

    // Servo passthrough: GPIO 12→10 and GPIO 11→9
    pinMode(SERVO_A_IN_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(SERVO_A_IN_PIN), onServoInA, CHANGE);
    gServoOutA.attach(SERVO_A_OUT_PIN, 800, 2200);

    pinMode(SERVO_B_IN_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(SERVO_B_IN_PIN), onServoInB, CHANGE);
    gServoOutB.attach(SERVO_B_OUT_PIN, 800, 2200);

    // Engage duty-cycle monitor on GPIO 5
    // ISR only sets gEngageActive; PCA9685 writes happen on Core 0 via Wire.
    pinMode(ENGAGE_IN_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENGAGE_IN_PIN), onEngagePin, CHANGE);

    Serial.println("Core 1 ready");
}

void loop1() {
    static TickType_t wake = xTaskGetTickCount();

    if (!gEngageActive) {
        // Mirror both servo inputs → outputs with a 3-sample median filter each.
        {
            static uint32_t med[3] = {1500, 1500, 1500};
            med[0] = med[1]; med[1] = med[2]; med[2] = gPulseWidthUsA;
            uint32_t a = med[0], b = med[1], c = med[2];
            if (a > b) { uint32_t t = a; a = b; b = t; }
            if (b > c) { uint32_t t = b; b = c; c = t; }
            if (a > b) { uint32_t t = a; a = b; b = t; }
            gServoOutA.writeMicroseconds((int)b);
        }
        {
            static uint32_t med[3] = {1500, 1500, 1500};
            med[0] = med[1]; med[1] = med[2]; med[2] = gPulseWidthUsB;
            uint32_t a = med[0], b = med[1], c = med[2];
            if (a > b) { uint32_t t = a; a = b; b = t; }
            if (b > c) { uint32_t t = b; b = c; c = t; }
            if (a > b) { uint32_t t = a; a = b; b = t; }
            gServoOutB.writeMicroseconds((int)b);
        }
    } else {
        // Engaged: follow internal commands (90° / 1500 µs centre)
        gServoOutA.writeMicroseconds(1500);
        gServoOutB.writeMicroseconds(1500);
    }

    // Compute engage state from duty cycle and publish to Core 0 via volatile flag.
    // No I2C here — Core 0 owns Wire and handles all PCA9685 transactions.
    if (gEngageValid) {
        gEngageActive = (gEngageHighUs >= ENGAGE_THRESHOLD_US);
    }

    SensorData snap;
    if (xSemaphoreTake(gDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        snap = gShared;
        xSemaphoreGive(gDataMutex);
        gLogger.log(snap);
    }

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOG_RATE_MS));
}

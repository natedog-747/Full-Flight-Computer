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
#include "Controller.h"
#include "ServoMirror.h"

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

static ServoMirror  gServoMirror;
static Controller   gController;

// ── Autopilot servo assignment ────────────────────────────────────────────────
// Map each control-loop axis to a physical servo channel.
// Swap these if your airframe's wiring is reversed.
static constexpr ServoMirror::Channel PITCH_SERVO = ServoMirror::SERVO_A;  // elevator
static constexpr ServoMirror::Channel ROLL_SERVO  = ServoMirror::SERVO_B;  // aileron

// ── PCA9685 engage logic ──────────────────────────────────────────────────────
// GPIO 5 PWM duty cycle is measured on Core 1 (interrupt-based).
// When duty exceeds ENGAGE_THRESHOLD, Core 1 sets gEngageActive = true.
// Core 0's sensor task reads that flag and drives the PCA9685 via Wire.
// ALL I2C stays on Core 0 — cross-core Wire calls crash the RP2040.
#define ENGAGE_IN_PIN  5

// Servo pulse-width thresholds (µs).  Standard RC range is 1000–2000 µs.
//
//  Zone               Pulse width       Switch position (3-pos switch)
//  ─────────────────  ────────────────  ────────────────────────────────
//  Passthrough        < 1250 µs         low  (~1000 µs)
//  Autopilot engaged  1250 – 1599 µs    middle (~1500 µs)
//  KF reset           ≥ 1600 µs         high (~2000 µs)
//
// RESET_THRESHOLD is set to 1600 µs rather than exactly 1500 µs so that a
// switch resting at the centre position (1500 µs) never accidentally fires
// a reset due to normal ±50 µs signal variation.
static constexpr uint32_t ENGAGE_THRESHOLD_US = 1250;   // 1/4 of 1000–2000 µs range
static constexpr uint32_t RESET_THRESHOLD_US  = 1900;   // above centre — high switch pos
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

// Cross-core KF reset request: Core 1 sets on rising edge into reset zone,
// Core 0 consumes (calls gKF.reset() + gBaro.resetCalibration()) and clears.
static volatile bool gKfResetPending = false;

// Cross-core ready-wag: Core 0 sets once when KF first reaches READY.
// gWagSatCount holds the satellite count to wag; Core 1 reads and clears both.
static volatile bool    gWagPending  = false;
static volatile uint8_t gWagSatCount = 0;

// Duty-cycle measurement for the engage pin (Core 1 ISR).
// gEngageValid is false until we have seen at least one complete high pulse,
// preventing a false trigger if the first edge is a falling edge.
static volatile bool     gEngageValid    = false;
static volatile uint32_t gEngageRiseUs   = 0;
static volatile uint32_t gEngageHighUs   = 0;
static volatile uint32_t gEngagePeriodUs = 20000;  // safe default (50 Hz)
// Timestamp of the last valid falling edge — used to detect signal loss.
static volatile uint32_t gEngageLastFallUs = 0;

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
            gEngageHighUs     = now - gEngageRiseUs;
            gEngageLastFallUs = now;
            gEngageValid      = true;
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

        // GPS Kalman measurement update — only when a fresh fix is available
        if (gpsLocal.gpsNewData && gpsLocal.gpsFix) {
            gKF.updateFromGPS(gpsLocal.gpsLat, gpsLocal.gpsLon, gpsLocal.gpsAlt,
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

        // ── KF reset (requested by Core 1 via switch high position) ─────────
        // Consume the flag here so reset() and resetCalibration() execute on
        // Core 0, which owns Wire (I2C).  Both calls are safe at this point
        // because the KF feeds/predict below will start fresh on this cycle.
        if (gKfResetPending) {
            gKfResetPending = false;
            gKF.reset();
            gBaro.resetCalibration();
            Serial.println("KF RESET: attitude + gyro bias re-initialising, baro re-baselining");
        }

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
            gKF.getPosition(imuLocal.kfLat, imuLocal.kfLon, imuLocal.kfAlt);
            gKF.getVelocity(imuLocal.kfVelN, imuLocal.kfVelE, imuLocal.kfVelD);
            imuLocal.kfInitPhase = (uint8_t)gKF.getInitPhase();
            imuLocal.kfBaroBias  = gKF.getBaroBias();

            // One-shot: wag the rudder when KF first reaches READY.
            // Satellite count is captured here (Core 0 owns GPS state).
            // Core 1 will consume gWagPending and execute the wag sequence.
            {
                static bool wagArmed = true;   // false after first READY transition
                if (wagArmed && gKF.isReady()) {
                    wagArmed     = false;
                    uint8_t sats = gpsLocal.gpsSats;
                    if (sats > 12) sats = 12;  // cap at 12 to keep wag under ~5 s
                    gWagSatCount = sats;
                    gWagPending  = true;
                    Serial.printf("WAG: KF ready — %u sats, wagging %u times\n", sats, sats);
                }
            }

            // ── One-shot: seed KF altitude from GPS on first READY + GPS fix + baro locked
            if (gKF.isReady() && gpsLocal.gpsFix && baroLocal.bmpPhase == BmpPhase::LOCKED) {
                gKF.seedAltitude(gpsLocal.gpsAlt, baroLocal.relAltM);
            }
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
            gShared.gpsLat        = gpsLocal.gpsLat;
            gShared.gpsLon        = gpsLocal.gpsLon;
            gShared.gpsAlt        = gpsLocal.gpsAlt;
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
            gShared.kfLat         = imuLocal.kfLat;
            gShared.kfLon         = imuLocal.kfLon;
            gShared.kfAlt         = imuLocal.kfAlt;
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

    // Servo passthrough: GPIO 12→10 (A) and GPIO 11→9 (B)
    gServoMirror.begin(SERVO_A_IN_PIN, SERVO_A_OUT_PIN,
                       SERVO_B_IN_PIN, SERVO_B_OUT_PIN);
    gServoMirror.setController(gController);

    // Engage duty-cycle monitor on GPIO 5
    // INPUT_PULLDOWN keeps the pin LOW when no RC signal is connected, preventing
    // spurious ISR triggers from a floating pin that could falsely set gEngageActive.
    // ISR only sets gEngageActive; PCA9685 writes happen on Core 0 via Wire.
    pinMode(ENGAGE_IN_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(ENGAGE_IN_PIN), onEngagePin, CHANGE);

    Serial.println("Core 1 ready");
}

void loop1() {
    static TickType_t wake         = xTaskGetTickCount();
    static bool       prevEngaged  = false;
    static bool       ctrlReady    = false;
    static uint8_t    engageCount  = 0;   // debounce counter

    // ── 1. Update engage / reset state ───────────────────────────────────────
    // Raw signal check: pulse is valid if a falling edge was seen within the
    // last 50 ms (2.5 RC frames at 50 Hz) AND the pulse width clears the
    // threshold.  gEngageHighUs starts at 0, so this defaults safely to false.
    bool fresh   = (micros() - gEngageLastFallUs) < 50000;
    bool rawHigh = fresh && (gEngageHighUs >= ENGAGE_THRESHOLD_US);
    bool inReset = fresh && (gEngageHighUs >= RESET_THRESHOLD_US);

    // Debounce: require 3 consecutive high reads (~30 ms) to go engaged,
    // but drop to disengaged immediately on the first non-high read.
    if (rawHigh) {
        if (engageCount < 3) engageCount++;
    } else {
        engageCount = 0;
    }
    gEngageActive = (engageCount >= 3);
    bool engaged = gEngageActive;

    // Rising edge into reset zone → schedule KF/baro reset on Core 0 (one-shot).
    // Also immediately disengage so the plane is in safe passthrough while the
    // KF re-initialises (~10 s accel avg + up to 30 s yaw init).
    {
        static bool prevInReset = false;
        if (inReset && !prevInReset) {
            gKfResetPending = true;
            Serial.println("KF RESET: scheduled — disengaging");
        }
        prevInReset = inReset;
    }

    // ── 2. Grab latest sensor snapshot ───────────────────────────────────────
    SensorData snap = {};   // zero-init so control never sees garbage if mutex times out
    if (xSemaphoreTake(gDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        snap = gShared;
        xSemaphoreGive(gDataMutex);
    }

    // ── 3. Engage / disengage logic ───────────────────────────────────────────
    if (!engaged) {
        // Switch low — disengage immediately.
        if (ctrlReady) {
            ctrlReady = false;
            gController.disengage();
            gServoMirror.passthrough();
            Serial.println("CTRL: disengaged — passthrough active");
        }
    } else if (inReset) {
        // Switch high (reset zone) — force disengaged while held here.
        // The KF reset was already scheduled on the rising edge in step 1.
        if (ctrlReady) {
            ctrlReady = false;
            gController.disengage();
            gServoMirror.passthrough();
        }
    } else if (!ctrlReady) {
        // Switch in engage zone — engage immediately (bumpless transfer).
        gController.engage(snap);
        ctrlReady = true;

        static constexpr float kR2D = 180.0f / 3.14159265f;
        char buf[160];
        snprintf(buf, sizeof(buf),
            "ENGAGE: kfPhase=%u gpsFix=%u roll=%.1fdeg pitch=%.1fdeg hdg=%.1fdeg",
            snap.kfInitPhase, (uint8_t)snap.gpsFix,
            snap.roll, snap.pitch, snap.yaw);
        Serial.println(buf);
        gLogger.logEvent(buf);
    }
    prevEngaged = engaged;

    // ── 4. Servo output ───────────────────────────────────────────────────────
    // Servo output — passthrough is the ONLY mode when not fully engaged.
    // Diagnostic line prints every 500 ms: mode, raw pin-5 pulse, servo values.

    // Wag state machine — executes only in passthrough mode.
    // Each "wag" is one full ±10° sweep (two 200 ms half-wags).
    // Number of wags = GPS satellite count captured when KF first reached READY.
    static constexpr uint32_t WAG_HALF_MS      = 200;  // ms per half-deflection
    static constexpr uint32_t WAG_AMPLITUDE_US = 56;   // 10° in µs (500µs/90° × 10°)
    static uint8_t  wagHalfRem = 0;   // half-wags remaining (2 per full wag)
    static bool     wagHigh    = true;
    static uint32_t wagNextMs  = 0;

    // Consume the pending flag from Core 0 (only start if not engaged)
    if (gWagPending && !engaged) {
        gWagPending = false;
        wagHalfRem  = (uint8_t)(gWagSatCount * 2);
        wagHigh     = true;
        wagNextMs   = millis();
    }

    // Hoist servo command outputs so step 4 and step 5 share the same values
    // without calling updateAxisA/B twice (which would double the integrators).
    uint16_t cmdA = 1500, cmdB = 1500;

    {
        static uint32_t lastDiagMs = 0;
        uint32_t nowMs = millis();
        bool printNow  = (nowMs - lastDiagMs >= 500);

        if (!engaged || !ctrlReady) {
            if (wagHalfRem > 0) {
                // Wag SERVO_A (rudder); let SERVO_B passthrough normally
                if (nowMs >= wagNextMs) {
                    uint32_t wagUs = wagHigh
                        ? (ServoMirror::PULSE_CENTER_US + WAG_AMPLITUDE_US)
                        : (ServoMirror::PULSE_CENTER_US - WAG_AMPLITUDE_US);
                    gServoMirror.write(ServoMirror::SERVO_A, wagUs);
                    wagHigh = !wagHigh;
                    wagHalfRem--;
                    wagNextMs = nowMs + WAG_HALF_MS;
                }
                gServoMirror.write(ServoMirror::SERVO_B,
                                   gServoMirror.getRawInput(ServoMirror::SERVO_B));
            } else {
                gServoMirror.passthrough();
            }
            if (printNow) {
                lastDiagMs = nowMs;
                const char *tag = (wagHalfRem > 0) ? "[WAG     ]"
                                : inReset           ? "[KF-RESET]"
                                :                     "[PASSTHRU]";
                Serial.printf("%s pin5=%luus cnt=%u rcA=%luus rcB=%luus\n",
                    tag, gEngageHighUs, engageCount,
                    gServoMirror.getRawInput(ServoMirror::SERVO_A),
                    gServoMirror.getRawInput(ServoMirror::SERVO_B));
            }
        } else {
            cmdA = gController.updateAxisA(snap);
            cmdB = gController.updateAxisB(snap);
            gServoMirror.write(ServoMirror::SERVO_A, cmdA);
            gServoMirror.write(ServoMirror::SERVO_B, cmdB);
            if (printNow) {
                lastDiagMs = nowMs;
                Serial.printf("[CONTROL ] pin5=%luus cnt=%u ctrlA=%uus ctrlB=%uus\n",
                    gEngageHighUs, engageCount, cmdA, cmdB);
            }
        }
    }

    // ── 5. Populate control-state fields in snap, then log ───────────────────
    // These are written into the LOCAL snap copy only — not back to gShared.
    snap.ctrlEngaged   = (engaged && ctrlReady);
    snap.engagePulseUs = gEngageHighUs;
    snap.rcA_us        = gServoMirror.getRawInput(ServoMirror::SERVO_A);
    snap.rcB_us        = gServoMirror.getRawInput(ServoMirror::SERVO_B);
    snap.ctrlServoA_us = cmdA;
    snap.ctrlServoB_us = cmdB;
    snap.ctrlErrYaw    = gController.getErrYaw();
    snap.ctrlIntegA    = gController.getIntegA();
    snap.ctrlErrAlt    = gController.getErrAlt();
    snap.ctrlIntegB    = gController.getIntegB();
    snap.ctrlRefAlt    = gController.getHomeAlt();

    gLogger.log(snap);

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOG_RATE_MS));
}

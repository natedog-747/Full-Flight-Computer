#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include "ImuSensor.h"

#ifndef PIN_NEOPIXEL
#define PIN_NEOPIXEL 8
#endif

static const uint32_t LOOP_HZ   = 20;
static const uint32_t LOOP_MS   = 50 / LOOP_HZ;

Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
ImuSensor imuSensor;

static uint16_t   hue           = 0;
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

    Wire.begin();
    if (!imuSensor.begin()) {
        while (1) vTaskDelay(portMAX_DELAY);
    }
    Serial.println("BNO055 ready");
    Serial.println("Hold still — calibrating 5 s...");

    xLastWakeTime = xTaskGetTickCount();
}

void loop() {
    // NeoPixel rainbow
    pixel.setPixelColor(0, pixel.gamma32(pixel.ColorHSV(hue)));
    pixel.show();
    hue += 256;

    imuSensor.update();

    if (!imuSensor.calibrating) {
        static uint32_t lastOrientPrintMs = 0;
        uint32_t now_ms = millis();
        if (now_ms - lastOrientPrintMs >= 100) {
            lastOrientPrintMs = now_ms;
            Serial.print("Roll: ");    Serial.print(imuSensor.roll,  2);
            Serial.print("  Pitch: "); Serial.print(imuSensor.pitch, 2);
            Serial.print("  Yaw: ");   Serial.println(imuSensor.yaw, 2);
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

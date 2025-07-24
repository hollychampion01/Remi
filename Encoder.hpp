#pragma once

#include <Arduino.h>
#include "global.hpp"

namespace mtrn3100 {

class Encoder {
public:
    Encoder(uint8_t enc1, uint8_t enc2) 
        : encoder1_pin(enc1), encoder2_pin(enc2) {

        instance = this;  // Store static instance pointer

        pinMode(encoder1_pin, INPUT_PULLUP);
        pinMode(encoder2_pin, INPUT_PULLUP);

        // Attach interrupt on encoder1_pin (must be interrupt-capable)
        // We assume enc1 is either pin 2 or pin 3 on Arduino Uno
        uint8_t interruptNum = digitalPinToInterrupt(encoder1_pin);
        if (interruptNum != NOT_AN_INTERRUPT) {
            attachInterrupt(interruptNum, readEncoderISR, RISING);
        }

        counts_per_revolution = 360; // ⚠️ Set according to your actual encoder
    }

    // This is called by the interrupt
    void readEncoder() {
        noInterrupts();

        // Determine direction using encoder2_pin
        if (digitalRead(encoder2_pin) == HIGH) {
            count++;
        } else {
            count--;
        }

        interrupts();
    }
    float getDistanceMM() {
        const float wheelDiameterMM = 32.0f;
        const float wheelCircumferenceMM = wheelDiameterMM * 3.1416;
        return getRotation() * wheelCircumferenceMM;
    }


    // Convert encoder count to radians
    float getRotation() {
        return (2.0f * PI * count) / counts_per_revolution;
    }

private:
    // Static ISR trampoline function
    static void readEncoderISR() {
        if (instance != nullptr) {
            instance->readEncoder();
        }
    }

public:
    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;
    volatile int8_t direction = 0;
    float position = 0;
    uint16_t counts_per_revolution = 360;
    volatile long count = 0;
    uint32_t prev_time = 0;
    bool read = false;

private:
    static Encoder* instance;
};

// Static member initialization
Encoder* Encoder::instance = nullptr;

}  // namespace mtrn3100

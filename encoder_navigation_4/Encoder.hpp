#pragma once
#include <Arduino.h>
#include "global.hpp"

namespace mtrn3100 {

class Encoder {
public:
  // encA must be an interrupt-capable pin (2 or 3 on Uno).
  // cpr = counts-per-wheel-rev (NOT motor shaft). Include gearbox * encoder PPR * x4 if using full quadrature.
  Encoder(uint8_t encA, uint8_t encB, uint16_t cpr = 360)
  : encoder1_pin(encA), encoder2_pin(encB), counts_per_revolution(cpr) {

    pinMode(encoder1_pin, INPUT_PULLUP);
    pinMode(encoder2_pin, INPUT_PULLUP);

    uint8_t intNum = digitalPinToInterrupt(encoder1_pin);
    // Map this instance to the correct interrupt and attach a dedicated ISR
    if (intNum == 0) { enc0 = this; attachInterrupt(0, isr0, CHANGE); }
    else if (intNum == 1) { enc1 = this; attachInterrupt(1, isr1, CHANGE); }
    // If you use other boards with more INTs, extend this mapping accordingly.
  }

  // --- Safe atomic read of count for use in loop/printing ---
  long getCount() const {
    noInterrupts();
    long c = count;
    interrupts();
    return c;
  }

  // Distance in mm (counts → rotations → distance). No extra 2π here.
  float getDistanceMM() const {
    const float wheelDiameterMM = 32.0f;
    const float wheelCircumferenceMM = wheelDiameterMM * 3.1416f;
    long c = getCount();
    return (static_cast<float>(c) * wheelCircumferenceMM) / counts_per_revolution;
  }

  // If you really want radians, compute from counts directly
  float getRotation() const { // radians of the wheel
    long c = getCount();
    return (2.0f * PI * static_cast<float>(c)) / counts_per_revolution;
  }

  void reset() {
    noInterrupts();
    count = 0;
    interrupts();
  }

  // Expose pins if you need for diagnostics
  const uint8_t encoder1_pin; // channel A (interrupt)
  const uint8_t encoder2_pin; // channel B (direction)
  volatile long count = 0;
  uint16_t counts_per_revolution;

private:
  // Called on any change of channel A for this instance
  inline void onAChange() {
    // Determine direction from THIS encoder's B channel
    // If your direction is reversed, swap ++/-- below.
    if (digitalRead(encoder2_pin) == HIGH) {
      count++;
    } else {
      count--;
    }
  }

  // Per-interrupt static trampolines (Uno: INT0=pin2, INT1=pin3)
  static void isr0() { if (enc0) enc0->onAChange(); }
  static void isr1() { if (enc1) enc1->onAChange(); }

  static Encoder* enc0;
  static Encoder* enc1;
};

// Static member definitions
Encoder* Encoder::enc0 = nullptr;
Encoder* Encoder::enc1 = nullptr;

} // namespace mtrn3100

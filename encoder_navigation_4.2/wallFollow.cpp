#include "wallFollow.hpp"
#include "movement.hpp"
#include <Arduino.h>  // for millis()

using namespace mtrn3100;

WallFollow::WallFollow(uint8_t leftPwm, uint8_t leftDir, uint8_t rightPwm, uint8_t rightDir,
                       float kp, float ki, float kd)
    : leftMotor(leftPwm, leftDir), rightMotor(rightPwm, rightDir), pid(kp, ki, kd) {}

void WallFollow::begin() {
    pid.zeroAndSetTarget(0, setpoint_mm);  // Set target distance to 100 mm
    startTime = millis();
    taskStarted = true;
}

void WallFollow::update() {
    if (!taskStarted) return;

    unsigned long elapsed = millis() - startTime;
    if (elapsed > max_time) {
        stopMove();
        taskStarted = false;
        return;
    }

    double currentDist = distFront;  // Use global variable from lidar update
    double error = setpoint_mm - currentDist;

    // If within ±5mm of setpoint, stop
    if (abs(error) <= tolerance_mm) {
        stopMove();
        return;
    }

    float pwm = pid.compute(currentDist);
    pwm = constrain(pwm, -200, 200);  // Optional: clamp motor PWM

    leftMotor.setPWM(pwm);
    rightMotor.setPWM(pwm);
}


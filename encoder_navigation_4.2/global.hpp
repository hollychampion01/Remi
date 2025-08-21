
#pragma once
#include <Wire.h>
#include <MPU6050_light.h>
#include "VL6180X.h"
#include <Arduino.h>

extern MPU6050 mpu;

extern float distFront;
extern float distLeft;
extern float distRight;

extern bool wallFront;
extern bool wallLeft;
extern bool wallRight;
extern bool wallSurround;

extern int pwm1;
extern int pwm2;

extern float currAngle;
extern float currPos;

extern String instructions[]; 

extern const String comm;
constexpr unsigned long commandDelay = 800;



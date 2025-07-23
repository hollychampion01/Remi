// global.cpp
#include "global.hpp"

MPU6050 mpu(Wire);
float distFront = 0;
float distLeft = 0;
float distRight = 0;

bool wallFront = false;
bool wallLeft = false;
bool wallRight = false;
bool wallSurround = false;

int pwm1 = 0;
int pwm2 = 0;

float currAngle = 0;
float currPos = 0;

const String comm = "lfrfflfr";

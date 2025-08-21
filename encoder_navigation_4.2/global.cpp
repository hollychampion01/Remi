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

String instructions[] = {
  "MOVE 150 75 60 0",    // Move forward 150mm at 75mm/s, heading 0°
  "TURN 45",             // Turn left 45°
  "MOVE 100 50 45 0",    // Move forward 100mm at 50mm/s, heading 45°
  "TURN -60",            // Turn right 60°
  "MOVE 120 60 345 0",   // Move forward 120mm at 60mm/s, heading 345°
  "TURN 30",             // Turn right 30°
  "MOVE 60 40 15 0",     // Move forward 60mm at 40mm/s, heading 15°
  "STOP"                 // Stop the robot
};

// Manually track the number of instructions
const size_t numInstructions = sizeof(instructions) / sizeof(instructions[0]);
const String comm = "";

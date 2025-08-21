#pragma once

#include "global.hpp"
#include "Motor.hpp"

// global.hpp or movement.hpp
#define MOTOR1_DIR(x) (x)
#define MOTOR2_DIR(x) (-(x))  // Reverse direction


extern mtrn3100::Motor motor1;
extern mtrn3100::Motor motor2;

inline void moveFront() {
motor1.setPWM(MOTOR1_DIR(100));
motor2.setPWM(MOTOR2_DIR(100));

}

inline void moveLeft() {
  motor1.setPWM((100));
  motor2.setPWM((100));
}

inline void moveRight() {
  motor1.setPWM(100);
  motor2.setPWM(100);
}

inline void stopMove() {
  motor1.setPWM(0);
  motor2.setPWM(0);
}

#ifndef INPUTTRANSFORMATION_HPP
#define INPUTTRANSFORMATION_HPP

#include "global.hpp"
#include "movement.hpp"
#include "mpu.hpp"
#include "Motor.hpp"
#include <Arduino.h>  // Required for String

namespace mtrn3100 {

class InputTrans {
public:
    void inputToCommand(const String& comm);

private:
    Motor leftMotor;
    Motor rightMotor;
};

} // namespace mtrn3100

#endif
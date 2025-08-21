#ifndef WALLFOLLOW_HPP
#define WALLFOLLOW_HPP

#include "Motor.hpp"
#include "PIDController.hpp"
#include "global.hpp"

namespace mtrn3100 {

class WallFollow {
public:
    WallFollow(uint8_t leftPwm, uint8_t leftDir, uint8_t rightPwm, uint8_t rightDir,
               float kp, float ki, float kd);

    void begin();
    void update();

private:
    Motor leftMotor;
    Motor rightMotor;
    PIDController pid;

    unsigned long startTime;
    bool taskStarted = false;

    const double setpoint_mm = 100.0;  // desired distance from wall
    const double tolerance_mm = 5.0;   // acceptable error
    const unsigned long max_time = 10000;   // 10 seconds

};

} // namespace mtrn3100

#endif

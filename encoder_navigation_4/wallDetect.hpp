#pragma once

#include "global.hpp"
#include "movement.hpp"
#include "mpu.hpp"

float normalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

inline void handleWallDetection() {
    static bool turning = false;
    static float targetAngle = 0.0f;

    float currYaw = mpu.getAngleZ();

    if (!turning) {
        if (wallSurround == 0) {
            moveFront();
        } 
        else if (wallSurround == 1) {
            if (wallLeft) {
                turning = true;
                targetAngle = normalizeAngle(currYaw + 90);
            } 
            else if (wallRight) {
                turning = true;
                targetAngle = normalizeAngle(currYaw - 90);
            } 
            else {
                stopMove();
            }
        } 
        else if (wallSurround == 2) {
            if (!wallLeft) {
                turning = true;
                targetAngle = normalizeAngle(currYaw - 90);
            } 
            else if (!wallRight) {
                turning = true;
                targetAngle = normalizeAngle(currYaw + 90);
            } 
            else {
                moveFront();
            }
        }
        else if (wallSurround == 3) {
            turning = true;
            static bool initialized = false;
            if (!initialized) {
                targetAngle = normalizeAngle(currYaw - 90);  // clockwise
                initialized = true;
            }
            
        }
    } 
    else {
        float currYaw = mpu.getAngleZ();
        float error = normalizeAngle(targetAngle - currYaw);

        if (abs(error) > 2.0f) {
            // Turn in the shortest direction
            if (error > 0) moveLeft();
            else moveRight();
        } else {
            stopMove();
            turning = false;
        }
    }
}



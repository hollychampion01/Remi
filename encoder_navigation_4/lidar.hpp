#pragma once

#include "global.hpp"

// Pins for shutdown control of each lidar sensor
#define XSHUT_FRONT A1
#define XSHUT_LEFT  A0
#define XSHUT_RIGHT A2

// Unique I2C addresses for each lidar sensor
#define ADDR_FRONT  0x33
#define ADDR_LEFT   0x34
#define ADDR_RIGHT  0x35


void initializeLidars();
void printLidarReadings();
void updateLidars();

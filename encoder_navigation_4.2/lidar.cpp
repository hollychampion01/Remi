#include "lidar.hpp"
#include <Arduino.h>

// Define the global lidar sensor objects
VL6180X lidarFront;
VL6180X lidarLeft;
VL6180X lidarRight;

// Define wall detection flags and count

void initializeLidars() {
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  // Step 0: power off all lidars
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(100);

  // Step 1: front
  digitalWrite(XSHUT_FRONT, HIGH);
  delay(10);
  lidarFront.init();
  lidarFront.setAddress(ADDR_FRONT);
  lidarFront.configureDefault();
  lidarFront.setTimeout(200);

  // Step 2: left
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(10);
  lidarLeft.init();
  lidarLeft.setAddress(ADDR_LEFT);
  lidarLeft.configureDefault();
  lidarLeft.setTimeout(200);

  // Step 3: right
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);
  lidarRight.init();
  lidarRight.setAddress(ADDR_RIGHT);
  lidarRight.configureDefault();
  lidarRight.setTimeout(200);

  Serial.println("Lidars initialized.");
}

void printLidarReadings() {
  distFront = lidarFront.readRangeSingle();
  distLeft = lidarLeft.readRangeSingle();
  distRight = lidarRight.readRangeSingle();

if (lidarFront.timeoutOccurred()) {
  Serial.println("LIDAR FRONT TIMEOUT!");
} else{
  Serial.print("Front: "); Serial.print(distFront); Serial.print(" mm | ");
}

if (lidarLeft.timeoutOccurred()) {
  Serial.println("LIDAR LEFT TIMEOUT!");
} else{
  Serial.print("Left: "); Serial.print(distLeft); Serial.print(" mm | ");
}

if (lidarRight.timeoutOccurred()) {
  Serial.println("LIDAR RIGHT TIMEOUT!");
} else{
  Serial.print("Right: "); Serial.println(distRight); Serial.println(" mm");
}
  
  wallSurround = 0;
  wallFront = distFront < 105;
  wallLeft = distLeft < 105;
  wallRight = distRight < 105;

  if (wallFront) wallSurround++;
  if (wallLeft)  wallSurround++;
  if (wallRight) wallSurround++;
}

void updateLidars() {
  distFront = lidarFront.readRangeSingle();
  distLeft = lidarLeft.readRangeSingle();
  distRight = lidarRight.readRangeSingle();
}



#include "mpu.hpp"

void setupMPU() {
  byte status = mpu.begin();
  if (status != 0) {
    Serial.println("MPU failed! Check wiring.");
    return;
  }

  Serial.print(F("MPU6050 status: "));
  Serial.println(status);

  while (status != 0) { } // wait if fail

  Serial.println(F("Calculating offsets..."));
  delay(3000);
  mpu.calcOffsets(true, true);
  Serial.println("Done!");
}

void printMPU() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    mpu.update();

    Serial.print(F("TEMPERATURE: ")); Serial.println(mpu.getTemp());
    Serial.print(F("ACCELERO  X: ")); Serial.print(mpu.getAccX());
    Serial.print("\tY: "); Serial.print(mpu.getAccY());
    Serial.print("\tZ: "); Serial.println(mpu.getAccZ());

    Serial.print(F("GYRO      X: ")); Serial.print(mpu.getGyroX());
    Serial.print("\tY: "); Serial.print(mpu.getGyroY());
    Serial.print("\tZ: "); Serial.println(mpu.getGyroZ());

    Serial.print(F("ACC ANGLE X: ")); Serial.print(mpu.getAccAngleX());
    Serial.print("\tY: "); Serial.println(mpu.getAccAngleY());

    Serial.print(F("ANGLE     X: ")); Serial.print(mpu.getAngleX());
    Serial.print("\tY: "); Serial.print(mpu.getAngleY());
    Serial.print("\tZ: "); Serial.println(mpu.getAngleZ());

    Serial.println(F("=====================================================\n"));
    lastPrint = millis();
  }
}

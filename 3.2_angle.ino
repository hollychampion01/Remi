#include "global.hpp"
#include "Motor.hpp"
#include "Encoder.hpp"
#include "PIDController.hpp"
#include "lidar.hpp"
#include "mpu.hpp"
#include "wallDetect.hpp"
#include "movement.hpp"
#include "wallFollow.hpp"

//screen
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// Motor 1 (left)
#define MOT1PWM 11
#define MOT1DIR 12
#define MOT1_ENCA 2
#define MOT1_ENCB 6

// Motor 2 (right)
#define MOT2PWM 9
#define MOT2DIR 10
#define MOT2_ENCA 3
#define MOT2_ENCB 7

#define SDA A4
#define SCL A5



// Motors and encoders
mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);
mtrn3100::Encoder encoder1(MOT1_ENCA, MOT1_ENCB);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);
mtrn3100::Encoder encoder2(MOT2_ENCA, MOT2_ENCB);

// Controllers (not used directly here, but kept for completeness)
mtrn3100::PIDController controller1(150.0, 10.0, 5.0);
mtrn3100::PIDController controller2(150.0, 10.0, 5.0);

// Distance PID Controller for front lidar
// mtrn3100::PIDController distanceController(2.0, 0.1, 0.5);  // Kp, Ki, Kd for distance control

// Angle PID Controller for rotation control (tuned for stability)
mtrn3100::PIDController angleController(4.0, 0.0, 0.0);  // Reduced gains to prevent oscillation

// Store initial angle
float initialAngle = 0.0;


// Lidar
// Make sure distFront is declared globally and updated in `updateLidars()`

// Wall follow controller
mtrn3100::WallFollow wallFollow(MOT1PWM, MOT1DIR, MOT2PWM, MOT2DIR, 20.0f, 5.0f, 2.0f);

void setup() {
    Serial.begin(9600);
    Wire.begin();
    while (!Serial);  // Wait for Serial (if needed)

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
    }

    // Clear the buffer
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println(F("IMU Ready"));
    display.println(F("Angle Display"));
    display.display();
    delay(2000);
  


    Serial.println("Setup starting...");

    setupMPU();           // Initializes MPU
    initializeLidars();   // Initializes LIDARs
    controller1.zeroAndSetTarget(encoder1.getRotation(), 2.0);
    controller2.zeroAndSetTarget(encoder2.getRotation(), 2.0);
    
    // Capture initial angle after MPU setup
    delay(500);  // Let MPU stabilize
    mpu.update();
    float startAngle = mpu.getAngleZ();
    // initialAngle = startAngle + 90.0;  // Target angle is 90 degrees from start
    initialAngle = 90.0;  // Target angle is 90 degrees from start
    Serial.print("Start angle: ");
    Serial.print(startAngle);
    Serial.print(" | Target angle: ");
    Serial.println(initialAngle);
    
    // Initialize angle PID controller to target 90 degrees from start
    angleController.zeroAndSetTarget(startAngle, initialAngle);  // Target: turn 90 degrees

    wallFollow.begin();   // Start the wall follow behavior

    Serial.println("Setup done.");
}

// Add at top of file
bool shouldMoveForward = false;



void loop() {
  mpu.update();
  updateLidars();

  // --- PID ANGLE CONTROL ---
  float currentAngle = mpu.getAngleZ();
  float angleError = angleController.compute(currentAngle);
  float turnSpeed = constrain(angleError, -80.0f, 80.0f);  // Reduced max speed for stability
  
  if (abs(angleError) > 3.0f) {
    // Apply PID-controlled rotation to return to initial angle
    // if (abs(turnSpeed) > 15.0f) {  // Larger dead zone to prevent oscillation
      // Differential steering: positive turnSpeed = turn right, negative = turn left
      motor1.setPWM(turnSpeed);   // Left motor
      motor2.setPWM(turnSpeed);   // Right motor (same direction for rotation)
    // } else {
    //   motor1.setPWM(0);
    //   motor2.setPWM(0);
    // }
} else {
    motor1.setPWM(0);
    motor2.setPWM(0);
}

  // Update PID
  float pos1 = encoder1.getRotation();
  float pos2 = encoder2.getRotation();
  pwm1 = constrain(controller1.compute(pos1), -255.0f, 255.0f);
  pwm2 = constrain(controller2.compute(pos2), -255.0f, 255.0f);

  // Optional: Debug output
  Serial.print("Current: "); Serial.print(currentAngle, 2);
  Serial.print(" | Target: "); Serial.print(initialAngle, 2);
  Serial.print(" | Error: "); Serial.print(currentAngle - initialAngle, 2);
  Serial.print(" | Turn Speed: "); Serial.println(turnSpeed, 1);

  // Wall following logic
  //handleWallDetection();
  // If this delay is too long, getZ may not be update frequently enough as controller is doing nothing as CPU is doing nothing
  // Should we use timing control aka millis
  delay(30);  // Slower update rate for stability (10Hz instead of 20Hz)
  screen();
}

/// for OLED display
void screen() {
  // Read current Z angle from MPU
  float currentAngle = mpu.getAngleZ();
  float error = currentAngle - initialAngle;

  // Clear display
  display.clearDisplay();

  //setting position of text
  display.setCursor(0, 0);

  //print Z angle
  display.setTextSize(1);
  display.print("Current: ");
  display.print(currentAngle, 1);
  display.println(" deg");
  
  // Display target angle
  display.print("Target:  ");
  display.print(initialAngle, 1);
  display.println(" deg");
  
  // Display error
  display.print("Error:   ");
  display.print(error, 1);
  display.print(" deg");
  
  display.display();
}
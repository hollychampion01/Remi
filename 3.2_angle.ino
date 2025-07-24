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

// --- Custom Angle Tracking & Drift Correction ---
float ahrs_angle = 0.0;          // Our manually integrated angle, more robust to drift.
float gyro_z_bias = 0.0;         // Stores the dynamically calculated gyro bias.
unsigned long last_update_micros = 0; // For precise dt calculation.
bool is_calibrating = true;      // Flag to control initial calibration.
#define CALIBRATION_SAMPLES 1000 // Number of samples for calibration.
int sample_count = 0;            // Counter for calibration samples.
float gyro_z_sum = 0;            // Sum for averaging samples.
// ---

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
    
    // --- Initial Gyro Bias Calibration ---
    Serial.println("Calibrating Gyro... Keep robot stationary.");
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Calibrating Gyro");
    display.println("Please wait...");
    display.display();

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        mpu.update();
        gyro_z_sum += mpu.getGyroZ();
        delay(2); // Small delay between samples
    }
    gyro_z_bias = gyro_z_sum / CALIBRATION_SAMPLES;
    gyro_z_sum = 0; // Reset for later use
    sample_count = 0;

    Serial.print("Gyro Z bias calculated: ");
    Serial.println(gyro_z_bias);
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Calibration Done!");
    display.display();
    delay(1000);
    // ---

    initializeLidars();   // Initializes LIDARs
    controller1.zeroAndSetTarget(encoder1.getRotation(), 2.0);
    controller2.zeroAndSetTarget(encoder2.getRotation(), 2.0);
    
    // Set target angle to 90 degrees. Our internal angle starts at 0.
    initialAngle = 90.0;
    angleController.zeroAndSetTarget(0.0, initialAngle);

    wallFollow.begin();   // Start the wall follow behavior

    Serial.println("Setup done.");
    last_update_micros = micros(); // Initialize timer for angle calculation
}

// Add at top of file
bool shouldMoveForward = false;



void loop() {
  mpu.update();

  // --- Custom Angle Calculation with Dynamic Bias Correction ---
  unsigned long now = micros();
  float dt = (now - last_update_micros) / 1000000.0f;
  last_update_micros = now;
  float gyro_z = mpu.getGyroZ();
  ahrs_angle += (gyro_z - gyro_z_bias) * dt;

  // --- PID ANGLE CONTROL ---
  float currentAngle = ahrs_angle; // Use our robust angle
  float angleError = initialAngle - currentAngle; // Error is Target - Current
  float turnSpeed = constrain(angleController.compute(currentAngle), -50.0f, 50.0f);

  // --- Dynamic Gyro Recalibration when Stationary ---
  // If the robot is supposed to be still (low error and low motor output),
  // any detected rotation is drift. We re-calculate the bias.
  if (abs(angleError) < 5.0f && abs(turnSpeed) < 1.0f) {
      // Robot is stationary, collect samples to refine bias
      gyro_z_sum += gyro_z;
      sample_count++;
      if (sample_count >= 200) { // Update bias every 200 samples
          gyro_z_bias = gyro_z_sum / sample_count;
          // Slowly correct ahrs_angle to prevent accumulated error while stationary
          ahrs_angle = ahrs_angle * 0.98 + initialAngle * 0.02;
          sample_count = 0; // Reset for next averaging cycle
          gyro_z_sum = 0;
      }
  } else {
      // Robot is moving, reset the calibration sample count
      sample_count = 0;
      gyro_z_sum = 0;
  }
  
  if (abs(angleError) > 3.0f) {
    // Apply PID-controlled rotation
    motor1.setPWM(turnSpeed);
    motor2.setPWM(turnSpeed);
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
  Serial.print(" | Error: "); Serial.print(angleError, 2);
  Serial.print(" | Turn Speed: "); Serial.print(turnSpeed, 1);
  Serial.print(" | Bias: "); Serial.println(gyro_z_bias, 4);

  // Wall following logic
  //handleWallDetection();
  // If this delay is too long, getZ may not be update frequently enough as controller is doing nothing as CPU is doing nothing
  // Should we use timing control aka millis
  delay(10);  // Can reduce delay as our loop is more controlled now
  screen();
}

/// for OLED display
void screen() {
  // Read current Z angle from our robust calculation
  float currentAngle = ahrs_angle;
  float error = initialAngle - currentAngle;

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

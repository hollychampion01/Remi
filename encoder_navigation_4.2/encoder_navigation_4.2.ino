#include "global.hpp"
#include "Motor.hpp"
#include "Encoder.hpp"
#include "PIDController.hpp"
#include "mpu.hpp"
#include "lidar.hpp"
#include <math.h>
#include <Wire.h>

// ==== Build-time switches (set to 1 to re-enable logs) ====
#define VERBOSE 0

// ================== Pinout ==================
#define MOT1PWM 11
#define MOT1DIR 12
#define MOT1_ENCA 2
#define MOT1_ENCB 6

#define MOT2PWM 9
#define MOT2DIR 10
#define MOT2_ENCA 3
#define MOT2_ENCB 7

// ================== Hardware ==================
mtrn3100::Motor   motor1(MOT1PWM, MOT1DIR);      // Left
mtrn3100::Encoder encoder1(MOT1_ENCA, MOT1_ENCB);
mtrn3100::Motor   motor2(MOT2PWM, MOT2DIR);      // Right
mtrn3100::Encoder encoder2(MOT2_ENCA, MOT2_ENCB);

// ================== Control ==================
mtrn3100::PIDController angleController(9.0, 0.05, 0.5); // Kp, Ki, Kd
mtrn3100::PIDController distanceController(1.0, 0.0, 0.0); // PID for controlling forward distance
mtrn3100::PIDController leftMotorHeadingController(0.0, 0.0, 0.0); // Left motor PID
mtrn3100::PIDController rightMotorHeadingController(0.0, 0.0, 0.0); // Right motor PID

// Heading hold (forward)
const float HEADING_KP           = 0.55f;
const float HEADING_MAX_ADJUST   = 10.0f;
const float CLOSE_THRESHOLD_MM   = 50.0f;
const float AVOIDANCE_KP         = 0.3f;
const float AVOIDANCE_MAX_ADJUST = 10.0f;

// Forward motion
const float FORWARD_DISTANCE_MM  = 330.0f;
const float forwardSpeedRight    = 120.0f;
const float forwardSpeedLeft     = 110.0f;

// Turn motion
const float TURN_DEG             = 45.0f;
const float TURN_MAX_PWM         = 60.5f;
const float TURN_COMPLETE_DEG    = 2.0f;
const unsigned long TURN_TIMEOUT_MS = 8000;

// Turn assist
const float TURN_MIN_PWM         = 18.0f;
const float TURN_START_KICK      = 35.0f;
const unsigned long TURN_KICK_MS = 120;


// ================== IMU state ==================
float yaw_unwrapped = 0.0f;    // continuous yaw (deg)
float ahrs_angle    = 0.0f;    // normalized [-180,180)
float gyro_z_bias   = 0.0f;
unsigned long last_update_micros = 0;



// ================== Command state ==================
size_t currentCommandIndex = 0;
bool commandInProgress = false;

// command string like "flll" (from your global.hpp)
extern const String comm;  // Instructions to process

// Forward helpers
float encoder1Start = 0.0f, encoder2Start = 0.0f;
float forwardHeadingTarget = 0.0f;

// Turn helpers
float targetYaw_unwrapped = 0.0f;

// ================== Prototypes ============
static void processInstructions();
static void executeMove(int distance, int speed, int heading);
static void executeTurn(int angle);
static void stopRobot();

static float normalizeAngle(float angle);
static void  updateIMU();
static bool  safeRecalibrateGyroBiasForTurn();
static void  updateRotationDisplay();


// ============ Setup ==================
void setup() {
#if VERBOSE
  Serial.begin(9600);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 1500)) {}
#endif

  Wire.begin();
  setupMPU();
  initializeLidars();

  // Gyro bias (robot still)
  float sum = 0;
  for (int i=0;i<200;i++){
    mpu.update();
    sum += mpu.getGyroZ();
    delay(2);
  }
  gyro_z_bias = sum / 200.0f;

  angleController.zeroAndSetTarget(0.0f, 0.0f);
  last_update_micros = micros();
}

// ============ Loop ==================
void loop() {
  // Process instructions from global.cpp
  if (currentCommandIndex < comm.length()) {
    char cmd = comm.charAt(currentCommandIndex);

    if (!commandInProgress) {
      if (cmd == 'f') {
        // Start a forward move
        encoder1Start = encoder1.getDistanceMM();
        encoder2Start = encoder2.getDistanceMM();
        updateIMU();
        // Set the forwardHeadingTarget only once (if it's not already set)
        if (forwardHeadingTarget == 0.0f) {
          forwardHeadingTarget = ahrs_angle; // Only set if it was never set before
        }

        commandInProgress = true;
      } else if (cmd == 'l' || cmd == 'r') {
        // Start a turn
        (void)safeRecalibrateGyroBiasForTurn();
        updateIMU();
        float delta = (cmd == 'l') ? +TURN_DEG : -TURN_DEG;
        targetYaw_unwrapped = yaw_unwrapped + delta;
        angleController.zeroAndSetTarget(yaw_unwrapped, targetYaw_unwrapped);
        commandInProgress = true;
      } else {
        currentCommandIndex++;
      }
    }

    if (commandInProgress) {
      bool done = false;
      if (cmd == 'f')      done = executeForward();
      else                 done = executeRotation();

      if (done) {
        commandInProgress = false;
        currentCommandIndex++;
        if (cmd == 'l' || cmd == 'r') {
          updateIMU();
          angleController.zeroAndSetTarget(yaw_unwrapped, yaw_unwrapped);
        }
      }
    }
  } else {
    motor1.setPWM(0);
    motor2.setPWM(0);
  }

  updateIMU();
  updateRotationDisplay();
  delay(10);
}

// ================== Process Instructions ==================
void processInstructions() {
  for (size_t i = 0; i < sizeof(instructions)/sizeof(instructions[0]); ++i) {
    String command = instructions[i];

    if (command.startsWith("MOVE")) {
      // Parse MOVE command
      int distance = command.substring(5, 8).toInt();  // Extract the distance
      int speed = command.substring(9, 12).toInt();    // Extract the speed
      int heading = command.substring(13, 16).toInt(); // Extract heading
      executeMove(distance, speed, heading);
    } 
    else if (command.startsWith("TURN")) {
      // Parse TURN command
      int angle = command.substring(5).toInt();  // Extract the angle
      executeTurn(angle);
    } 
    else if (command == "STOP") {
      // Execute STOP
      stopRobot();
    }
  }
}

// ============ Forward (encoders + heading hold) ============
bool executeForward() {
  float d1 = fabsf(encoder1.getDistanceMM() - encoder1Start);
  float d2 = fabsf(encoder2.getDistanceMM() - encoder2Start);
  float avg = 0.5f * (d1 + d2);

  if (avg < FORWARD_DISTANCE_MM) { // Continue forward until we reach the target distance
    updateIMU();
    updateLidars();

    // Compute forward distance error
    float distanceErr = FORWARD_DISTANCE_MM - avg;
    float forwardPWM = distanceController.compute(distanceErr); // Use PID for forward distance

    // Heading correction using separate PIDs for each wheel
    float headingErr = forwardHeadingTarget - ahrs_angle;

    while (headingErr > 180.0f) headingErr -= 360.0f;   // Normalize heading error
    while (headingErr < -180.0f) headingErr += 360.0f;

    // Compute heading adjustment for the left motor
    float leftHeadingErr = headingErr;
    float leftHeadingAdj = leftMotorHeadingController.compute(leftHeadingErr);
    leftHeadingAdj = constrain(leftHeadingAdj, -HEADING_MAX_ADJUST, HEADING_MAX_ADJUST);

    // Compute heading adjustment for the right motor
    float rightHeadingErr = headingErr;
    float rightHeadingAdj = rightMotorHeadingController.compute(rightHeadingErr);
    rightHeadingAdj = constrain(rightHeadingAdj, -HEADING_MAX_ADJUST, HEADING_MAX_ADJUST);

    // Combine both corrections (distance and heading) for the final motor speeds
    float leftPWM = forwardSpeedLeft - leftHeadingAdj;   // Adjust for left motor
    float rightPWM = forwardSpeedRight + rightHeadingAdj; // Adjust for right motor

    // NOTE: opposite sign on left motor for forward motion
    motor1.setPWM(rightPWM);  // Set PWM for right motor
    motor2.setPWM(-leftPWM);  // Set PWM for left motor (opposite direction)

    return false; // Keep moving forward
  } else {
    motor1.setPWM(0); // Stop the motors when the target distance is reached
    motor2.setPWM(0);
    return true; // Stop the forward motion
  }
}

// ============ Turn (IMU only; PID in unwrapped yaw) ============
bool executeRotation() {
  static bool started = false;
  static unsigned long startMs = 0;
  static float lastTurnPWM = 0.0f;

  updateIMU();
  float err = targetYaw_unwrapped - yaw_unwrapped;

  if (!started) {
    angleController.zeroAndSetTarget(yaw_unwrapped, targetYaw_unwrapped);
    startMs   = millis();
    started   = true;
    lastTurnPWM = 0.0f;

    float kickSign = (err >= 0.0f) ? +1.0f : -1.0f;
    motor1.setPWM(kickSign * TURN_START_KICK);
    motor2.setPWM(kickSign * TURN_START_KICK);
    delay(TURN_KICK_MS);
    motor1.setPWM(0);
    motor2.setPWM(0);
  }

  bool timeout = (millis() - startMs) > TURN_TIMEOUT_MS;

  if (fabsf(err) > TURN_COMPLETE_DEG && !timeout) {
    float out = angleController.compute(yaw_unwrapped);

    float desiredSign = (err >= 0.0f) ? +1.0f : -1.0f;
    if (out * desiredSign < 0.0f) out = -out;

    float mag = fabsf(out);
    if (mag < TURN_MIN_PWM) mag = TURN_MIN_PWM;
    if (mag > TURN_MAX_PWM) mag = TURN_MAX_PWM;
    float turnPWM = desiredSign * mag;

    float maxDeltaPWM = 5.0f;
    float deltaPWM = turnPWM - lastTurnPWM;
    if (deltaPWM >  maxDeltaPWM) deltaPWM =  maxDeltaPWM;
    if (deltaPWM < -maxDeltaPWM) deltaPWM = -maxDeltaPWM;
    turnPWM = lastTurnPWM + deltaPWM;
    lastTurnPWM = turnPWM;

    motor1.setPWM(turnPWM);
    motor2.setPWM(turnPWM);
    return false;
  } else {
    motor1.setPWM(0);
    motor2.setPWM(0);
    started = false;
    delay(200);
    return true;
  }
}


// ================== Move and Turn Functions ==================
void executeMove(int distance, int speed, int heading) {
  Serial.print("Moving forward: ");
  Serial.print(distance);
  Serial.print(" mm at ");
  Serial.print(speed);
  Serial.print(" mm/s with heading ");
  Serial.println(heading);
  
  // Logic to execute the movement
  // You would typically control motors here using distance and speed parameters
  // and adjust heading by rotating the robot as needed.
}

void executeTurn(int angle) {
  Serial.print("Turning: ");
  Serial.print(angle);
  Serial.println(" degrees");

  // Logic to turn the robot by the specified angle
  // You would rotate the robot using the angle
}

void stopRobot() {
  Serial.println("Stopping the robot");
  motor1.setPWM(0);
  motor2.setPWM(0);
  delay(500);
}
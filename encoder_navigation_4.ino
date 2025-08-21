#include "global.hpp"
#include "Motor.hpp"
#include "Encoder.hpp"
#include "PIDController.hpp"
#include "mpu.hpp"
#include "lidar.hpp"
#include <math.h>

/*
  Changes vs your original:
  - Turn assist: TURN_MIN_PWM floor + optional TURN_START_KICK to beat stiction.
  - Sign guard: force the commanded turn to match the error's sign.
  - Safer per-turn gyro bias: settle motors, sample longer, sanity-check mean/variance.
*/

// ================== Pinout ==================
#define MOT1PWM 11
#define MOT1DIR 12
#define MOT1_ENCA 2
#define MOT1_ENCB 6

#define MOT2PWM 9
#define MOT2DIR 10
#define MOT2_ENCA 3
#define MOT2_ENCB 7

#define SDA A4
#define SCL A5

// ================== Hardware ==================
mtrn3100::Motor   motor1(MOT1PWM, MOT1DIR);      // Left (see forward note below)
mtrn3100::Encoder encoder1(MOT1_ENCA, MOT1_ENCB);
mtrn3100::Motor   motor2(MOT2PWM, MOT2DIR);      // Right
mtrn3100::Encoder encoder2(MOT2_ENCA, MOT2_ENCB);

// ================== Control ==================
// PID for angle (turns). Tuned to be stable without D at the seam.
mtrn3100::PIDController angleController(9.0, 0.05, 0.5); // Kp, Ki, Kd

// Tiny heading hold (forward)
const float HEADING_KP           = 0.55f;
const float HEADING_MAX_ADJUST   = 12.0f;
const float CLOSE_THRESHOLD_MM   = 50.0f;
const float AVOIDANCE_KP         = 0.3f;
const float AVOIDANCE_MAX_ADJUST = 10.0f;

// Forward motion
const float FORWARD_DISTANCE_MM  = 330.0f; // one cell
const float forwardSpeedRight    = 120.0f; // base PWMs
const float forwardSpeedLeft     = 120.0f;

// Turn motion
const float TURN_DEG             = 90.0f;
const float TURN_MAX_PWM         = 60.5f;
const float TURN_COMPLETE_DEG    = 2.0f;   // slightly looser for robustness
const unsigned long TURN_TIMEOUT_MS = 8000;

// --- Turn assist (NEW) ---
const float TURN_MIN_PWM         = 18.0f;      // set to beat static friction; tune 15–25
const float TURN_START_KICK      = 35.0f;      // short initial kick to get moving
const unsigned long TURN_KICK_MS = 120;        // kick duration (ms)

// ================== IMU state ==================
// Keep a continuous yaw for control; normalize only for printing.
float yaw_unwrapped = 0.0f;      // continuous yaw (deg), never normalized
float ahrs_angle    = 0.0f;      // pretty print yaw in [-180,180)
float gyro_z_bias   = 0.0f;
unsigned long last_update_micros = 0;

// ================== Command state ==================
size_t currentCommandIndex = 0;
bool   commandInProgress   = false;

// from your codebase (e.g., in global.hpp) — string of commands like "flll"
extern const String comm;

// Forward helpers
float encoder1Start = 0.0f, encoder2Start = 0.0f;
float forwardHeadingTarget = 0.0f;

// Turn helpers
float targetYaw_unwrapped = 0.0f;

// ============ Prototypes ============
static inline float normalizeAngle(float angle);
static inline void  updateIMU();
static bool         safeRecalibrateGyroBiasForTurn();

// ============ Math helpers ============
static inline float normalizeAngle(float angle) {
  angle = fmodf(angle + 180.0f, 360.0f);
  if (angle < 0) angle += 360.0f;
  return angle - 180.0f;
}

// ============ IMU update ============
static inline void updateIMU() {
  mpu.update();
  unsigned long now = micros();
  float dt = (now - last_update_micros) / 1e6f;
  if (dt < 0) dt = 0; // safety if micros() wrapped
  last_update_micros = now;

  // integrate gyro (deg/s) -> deg (unwrapped)
  float gz = mpu.getGyroZ();
  yaw_unwrapped += (gz - gyro_z_bias) * dt;

  // pretty/diagnostic angle for logs and heading hold
  ahrs_angle = normalizeAngle(yaw_unwrapped);
}

// ============ Forward (encoders + heading hold) ============
bool executeForward() {
  // distances since start
  float d1 = fabsf(encoder1.getDistanceMM() - encoder1Start);
  float d2 = fabsf(encoder2.getDistanceMM() - encoder2Start);
  float avg = 0.5f * (d1 + d2);

  if (avg < FORWARD_DISTANCE_MM) {
    updateIMU();
    updateLidars();

    // Heading hold (based on normalized heading for convenience)
    float headingErr = forwardHeadingTarget - ahrs_angle;
    while (headingErr > 180.0f) headingErr -= 360.0f;
    while (headingErr < -180.0f) headingErr += 360.0f;

    float headingAdj = headingErr * HEADING_KP;

    // Simple wall avoidance nudge
    float avoidanceAdj = 0.0f;
    if (distLeft < CLOSE_THRESHOLD_MM) {
      avoidanceAdj -= (CLOSE_THRESHOLD_MM - distLeft) * AVOIDANCE_KP;
    }
    if (distRight < CLOSE_THRESHOLD_MM) {
      avoidanceAdj += (CLOSE_THRESHOLD_MM - distRight) * AVOIDANCE_KP;
    }
    avoidanceAdj = constrain(avoidanceAdj, -AVOIDANCE_MAX_ADJUST, AVOIDANCE_MAX_ADJUST);

    headingAdj += avoidanceAdj;
    headingAdj = constrain(headingAdj, -HEADING_MAX_ADJUST, HEADING_MAX_ADJUST);

    float leftPWM  = forwardSpeedLeft  - headingAdj;
    float rightPWM = forwardSpeedRight + headingAdj;

    // NOTE: platform uses opposite sign on left motor for forward
    motor1.setPWM(rightPWM);
    motor2.setPWM(-leftPWM);

    return false;
  } else {
    motor1.setPWM(0);
    motor2.setPWM(0);
    Serial.println("Forward done");
    delay(200);
    return true;
  }
}

// ============ Turn (IMU only; PID in unwrapped yaw) ============
bool executeRotation() {
  static bool started = false;
  static unsigned long startMs = 0;
  static float lastTurnPWM = 0.0f;  // for PWM ramping

  updateIMU();
  float err = targetYaw_unwrapped - yaw_unwrapped; // angle error (unwrapped)

  if (!started) {
    angleController.zeroAndSetTarget(yaw_unwrapped, targetYaw_unwrapped);
    startMs   = millis();
    started   = true;
    lastTurnPWM = 0.0f;

    // --- OPTIONAL: short kick to break static friction/backlash ---
    float kickSign = (err >= 0.0f) ? +1.0f : -1.0f;
    motor1.setPWM(kickSign * TURN_START_KICK);
    motor2.setPWM(kickSign * TURN_START_KICK);
    delay(TURN_KICK_MS);
    motor1.setPWM(0);
    motor2.setPWM(0);
    // --------------------------------------------------------------

    Serial.print("Rotate start: cur=");
    Serial.print(ahrs_angle, 1);
    Serial.print(" tgt=");
    Serial.println(normalizeAngle(targetYaw_unwrapped), 1);
  }

  bool timeout = (millis() - startMs) > TURN_TIMEOUT_MS;

  if (fabsf(err) > TURN_COMPLETE_DEG && !timeout) {
    float out = angleController.compute(yaw_unwrapped);

    // --- NEW: enforce correct turn direction + deadband floor
    float desiredSign = (err >= 0.0f) ? +1.0f : -1.0f;  // + = turn left, - = turn right
    if (out * desiredSign < 0.0f) {
      // PID produced opposite sign; flip it so we always drive the intended direction
      out = -out;
    }

    float mag = fabsf(out);
    if (mag < TURN_MIN_PWM) mag = TURN_MIN_PWM;
    if (mag > TURN_MAX_PWM) mag = TURN_MAX_PWM;
    float turnPWM = desiredSign * mag;
    // -------------------------------------------------------------

    // --- PWM ramping (unchanged) ---
    float maxDeltaPWM = 5.0f;            // maximum change per loop
    float deltaPWM = turnPWM - lastTurnPWM;
    if (deltaPWM >  maxDeltaPWM) deltaPWM =  maxDeltaPWM;
    if (deltaPWM < -maxDeltaPWM) deltaPWM = -maxDeltaPWM;
    turnPWM = lastTurnPWM + deltaPWM;
    lastTurnPWM = turnPWM;
    // -------------------------------

    // Spin in place with ramped PWM; same sign on both motors matches your wiring
    motor1.setPWM(turnPWM);
    motor2.setPWM(turnPWM);

    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500) {
      Serial.print("Rot cur="); Serial.print(ahrs_angle, 1);
      Serial.print(" err=");    Serial.print(err, 1);
      Serial.print(" pwm=");    Serial.println(turnPWM, 1);
      lastDebug = millis();
    }
    return false;
  } else {
    motor1.setPWM(0);
    motor2.setPWM(0);
    started = false;
    if (timeout) {
      Serial.println("Rotate timeout — stopping");
    } else {
      Serial.print("Rotate done. final err=");
      Serial.println(err, 1);
    }
    delay(300);
    return true;
  }
}

// ================== Safer per-turn gyro bias ==================
static bool safeRecalibrateGyroBiasForTurn() {
  // Ensure we're fully stopped & settled before biasing
  motor1.setPWM(0);
  motor2.setPWM(0);
  delay(250);  // let vibrations die down

  float sum = 0.0f, sum2 = 0.0f;
  const int N = 80; // slightly longer sample than before for stability
  for (int i = 0; i < N; i++) {
    mpu.update();
    float gz = mpu.getGyroZ();
    sum  += gz;
    sum2 += gz * gz;
    delay(2);
  }
  float mean = sum / N;
  float var  = (sum2 / N) - mean * mean;

  // Accept only if "still enough". Tune thresholds to your IMU noise.
  const float MEAN_THRESH = 1.0f; // deg/s
  const float VAR_THRESH  = 4.0f; // (deg/s)^2
  bool accepted = (fabsf(mean) < MEAN_THRESH) && (var < VAR_THRESH);

  if (accepted) {
    gyro_z_bias = mean;
    Serial.print("turn gyro_z_bias="); Serial.println(gyro_z_bias, 3);
  } else {
    Serial.print("turn gyro_z_bias REJECTED (mean="); Serial.print(mean, 3);
    Serial.print(", var="); Serial.print(var, 3);
    Serial.println(") — keeping previous bias");
  }

  // IMPORTANT: reset integration timer so next dt isn't huge
  last_update_micros = micros();
  return accepted;
}

// ================== Setup ==================
void setup() {
  Serial.begin(9600);
  Wire.begin();
  while (!Serial) {}

  Serial.println("KISS Maze Bot Boot");

  setupMPU();
  initializeLidars();

  // Gyro bias (robot still)
  Serial.println("Calibrating gyro...");
  float sum = 0;
  for (int i=0;i<200;i++){
    mpu.update();
    sum += mpu.getGyroZ();
    delay(2);
  }
  gyro_z_bias = sum / 200.0f;
  Serial.print("gyro_z_bias="); Serial.println(gyro_z_bias,4);

  // Init controller & timer
  angleController.zeroAndSetTarget(0.0f, 0.0f);
  last_update_micros = micros(); // start IMU integration timing fresh

  Serial.println("Ready.");
}

// ================== Main loop ==================
void loop() {
  // Process command string like "flll"
  if (currentCommandIndex < comm.length()) {
    char cmd = comm.charAt(currentCommandIndex);

    if (!commandInProgress) {
      Serial.print("Cmd: "); Serial.println(cmd);

      if (cmd == 'f') {
        // Start a straight drive using encoders, lock heading once
        encoder1Start = encoder1.getDistanceMM();
        encoder2Start = encoder2.getDistanceMM();
        updateIMU(); // get fresh yaw
        forwardHeadingTarget = ahrs_angle;  // lock current (normalized) heading
        commandInProgress = true;

      } else if (cmd == 'l' || cmd == 'r') {
        // Prepare a 90° IMU turn with safer gyro calibration
        Serial.println("Recalibrating gyro for turn...");

        // Safer re-bias; OK if it fails (we'll keep old bias)
        (void)safeRecalibrateGyroBiasForTurn();

        updateIMU();
        float delta = (cmd == 'l') ? +TURN_DEG : -TURN_DEG;
        targetYaw_unwrapped = yaw_unwrapped + delta;

        angleController.zeroAndSetTarget(yaw_unwrapped, targetYaw_unwrapped);
        commandInProgress = true;

      } else {
        Serial.println("Unknown cmd, skipping");
        currentCommandIndex++;
      }
    }

    if (commandInProgress) {
      bool done = false;

      if (cmd == 'f') {
        done = executeForward();
      } else if (cmd == 'l' || cmd == 'r') {
        done = executeRotation();
      }

      if (done) {
        commandInProgress = false;
        currentCommandIndex++;

        // After a turn, clear controller state & freeze current yaw
        if (cmd == 'l' || cmd == 'r') {
          updateIMU();
          angleController.zeroAndSetTarget(yaw_unwrapped, yaw_unwrapped);
          Serial.print("Turn completed, final angle: ");
          Serial.println(ahrs_angle, 1); // pretty print for humans
        }
      }
    }
  } else {
    // Idle — motors off, keep yaw integrated
    updateIMU();
    motor1.setPWM(0);
    motor2.setPWM(0);
    static unsigned long lastMsg = 0;
    if (millis() - lastMsg > 2000) {
      Serial.println("Idle (all commands done).");
      lastMsg = millis();
    }
  }

  delay(10);
}

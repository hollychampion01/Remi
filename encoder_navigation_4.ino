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

// ==== OLED (SSD1306) ====
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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

// Heading hold (forward)
const float HEADING_KP           = 0.55f;
const float HEADING_MAX_ADJUST   = 12.0f;
const float CLOSE_THRESHOLD_MM   = 50.0f;
const float AVOIDANCE_KP         = 0.3f;
const float AVOIDANCE_MAX_ADJUST = 10.0f;

// Forward motion
const float FORWARD_DISTANCE_MM  = 330.0f;
const float forwardSpeedRight    = 120.0f;
const float forwardSpeedLeft     = 120.0f;

// Turn motion
const float TURN_DEG             = 90.0f;
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

// ================== Rotation Display ==================
float rotation_start = 0.0f;
float current_rotation = 0.0f;
unsigned long last_rotation_display = 0;

// ================== Command state ==================
size_t currentCommandIndex = 0;
bool   commandInProgress   = false;

// command string like "flll" (from your global.hpp)
extern const String comm;

// Forward helpers
float encoder1Start = 0.0f, encoder2Start = 0.0f;
float forwardHeadingTarget = 0.0f;

// Turn helpers
float targetYaw_unwrapped = 0.0f;

// ============ Prototypes ============
static float normalizeAngle(float angle);
static void  updateIMU();
static bool  safeRecalibrateGyroBiasForTurn();
static void  updateRotationDisplay();
static void  oledInit();

// ============ Math helpers ============
static float normalizeAngle(float angle) {
  angle = fmodf(angle + 180.0f, 360.0f);
  if (angle < 0) angle += 360.0f;
  return angle - 180.0f;
}

// ============ IMU update ============
static void updateIMU() {
  mpu.update();
  unsigned long now = micros();
  float dt = (now - last_update_micros) / 1e6f;
  if (dt < 0) dt = 0;
  last_update_micros = now;

  float gz = mpu.getGyroZ();
  yaw_unwrapped += (gz - gyro_z_bias) * dt;
  ahrs_angle = normalizeAngle(yaw_unwrapped);
}

// ============ OLED ============
static void oledInit() {
  static bool inited = false;
  if (inited) return;
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(F("MazeBot"));
  display.display();
  inited = true;
}

// ============ Rotation Display ============
static void updateRotationDisplay() {
  current_rotation = yaw_unwrapped - rotation_start;

  if (millis() - last_rotation_display > 200) {
#if VERBOSE
    Serial.print(F("Angle: "));
    Serial.print(current_rotation, 1);
    Serial.println(F(" deg"));
#endif
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print(F("Ang "));
    display.print(current_rotation, 1);
    display.println(F("d"));
    // small normalized yaw below
    display.setTextSize(1);
    display.setCursor(0, 32);
    display.print(F("Yaw "));
    display.print(ahrs_angle, 1);
    display.display();

    last_rotation_display = millis();
  }
}

// ============ Forward (encoders + heading hold) ============
bool executeForward() {
  float d1 = fabsf(encoder1.getDistanceMM() - encoder1Start);
  float d2 = fabsf(encoder2.getDistanceMM() - encoder2Start);
  float avg = 0.5f * (d1 + d2);

  if (avg < FORWARD_DISTANCE_MM) {
    updateIMU();
    updateLidars();

    float headingErr = forwardHeadingTarget - ahrs_angle;
    while (headingErr > 180.0f) headingErr -= 360.0f;
    while (headingErr < -180.0f) headingErr += 360.0f;

    float headingAdj = headingErr * HEADING_KP;

    float avoidanceAdj = 0.0f;
    if (distLeft  < CLOSE_THRESHOLD_MM) avoidanceAdj -= (CLOSE_THRESHOLD_MM - distLeft)  * AVOIDANCE_KP;
    if (distRight < CLOSE_THRESHOLD_MM) avoidanceAdj += (CLOSE_THRESHOLD_MM - distRight) * AVOIDANCE_KP;
    avoidanceAdj = constrain(avoidanceAdj, -AVOIDANCE_MAX_ADJUST, AVOIDANCE_MAX_ADJUST);

    headingAdj += avoidanceAdj;
    headingAdj = constrain(headingAdj, -HEADING_MAX_ADJUST, HEADING_MAX_ADJUST);

    float leftPWM  = forwardSpeedLeft  - headingAdj;
    float rightPWM = forwardSpeedRight + headingAdj;

    // NOTE: opposite sign on left motor for forward
    motor1.setPWM(rightPWM);
    motor2.setPWM(-leftPWM);
    return false;
  } else {
    motor1.setPWM(0);
    motor2.setPWM(0);
    return true;
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

// ================== Safer per-turn gyro bias ==================
static bool safeRecalibrateGyroBiasForTurn() {
  motor1.setPWM(0);
  motor2.setPWM(0);
  delay(250);

  float sum = 0.0f, sum2 = 0.0f;
  const int N = 80;
  for (int i = 0; i < N; i++) {
    mpu.update();
    float gz = mpu.getGyroZ();
    sum  += gz;
    sum2 += gz * gz;
    delay(2);
  }
  float mean = sum / N;
  float var  = (sum2 / N) - mean * mean;

  const float MEAN_THRESH = 1.0f;
  const float VAR_THRESH  = 4.0f;
  bool accepted = (fabsf(mean) < MEAN_THRESH) && (var < VAR_THRESH);

  if (accepted) gyro_z_bias = mean;

  last_update_micros = micros();
  return accepted;
}

// ================== Setup ==================
void setup() {
#if VERBOSE
  Serial.begin(9600);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 1500)) {}
#endif

  Wire.begin();
  oledInit();

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

  updateIMU();
  rotation_start = yaw_unwrapped;
  current_rotation = 0.0f;
  last_rotation_display = millis() - 201; // force first update
  updateRotationDisplay();
}

// ================== Main loop ==================
void loop() {
  if (currentCommandIndex < comm.length()) {
    char cmd = comm.charAt(currentCommandIndex);

    if (!commandInProgress) {
      if (cmd == 'f') {
        encoder1Start = encoder1.getDistanceMM();
        encoder2Start = encoder2.getDistanceMM();
        updateIMU();
        forwardHeadingTarget = ahrs_angle;
        commandInProgress = true;
      } else if (cmd == 'l' || cmd == 'r') {
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

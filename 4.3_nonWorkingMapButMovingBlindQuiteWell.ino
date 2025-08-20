#include "global.hpp"
#include "Motor.hpp"
#include "Encoder.hpp"
#include "PIDController.hpp"
#include "lidar.hpp"
#include "mpu.hpp"
#include <Wire.h>
#include <math.h>
#include <EEPROM.h>

// ==================== Pins ====================
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

// ================= Motors / Encoders =================
mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);
mtrn3100::Encoder encoder1(MOT1_ENCA, MOT1_ENCB);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);
mtrn3100::Encoder encoder2(MOT2_ENCA, MOT2_ENCB);

// ================= PID (not used directly here) =================
mtrn3100::PIDController controller1(150.0, 10.0, 5.0);
mtrn3100::PIDController controller2(150.0, 10.0, 5.0);

// Distance / angle PIDs (unused here but kept)
mtrn3100::PIDController distanceController(2.0, 0.0, 0.0);
mtrn3100::PIDController angleController(2.0, 0.00, 0.0);

// =============== Movement globals (your old code) ===============
float initialAngle = 0.0;
size_t currentCommandIndex = 0;
bool commandInProgress = false;
unsigned long lastCommandTime = 0;

const float forwardSpeedRight = 100.0f;
const float forwardSpeedLeft  = 98.0f;

float encoder1Start = 0.0f;
float encoder2Start = 0.0f;
float LeftSpeed = 0.0f;
float RightSpeed = 0.0f;

void printLidarReadings(); // provided elsewhere

// --- turn latch ---
bool turning = false;
unsigned long turnReleaseMs = 0;
float turnTarget = 0.0f;

const unsigned long TURN_HOLD_MS = 300;
const float TURN_TOL = 3.0f;
const float TURN_PWM_SCALE = 0.25f;

// Geometry-aware thresholds
const float CELL_MM = 180.0f;
const float ROBOT_MM = 75.0f;
const float SIDE_CENTER_MM = (CELL_MM - ROBOT_MM) / 2.0f; // ~52.5 mm
const float FRONT_TURN_MM = 55.0f;
const float SIDE_WALL_PRESENT_MM = SIDE_CENTER_MM + 17.0f; // ~70 mm
const float SIDE_TOO_CLOSE_MM = 35.0f;
const float FRONT_TURN_MM_CORRIDOR_BONUS = 20.0f;

// ====== Scan pause at cell centers ======
bool cellScanLatched = false;
unsigned long cellScanReleaseMs = 0;
const unsigned long CELL_STOP_MS = 120; // 100–200 ms pause is good

// ===================== Mapping & Pose =====================
enum Dir : uint8_t { NORTH=0, EAST=1, SOUTH=2, WEST=3 };
inline Dir leftOf(Dir d)  { return Dir((uint8_t(d) + 3) & 3); }
inline Dir rightOf(Dir d) { return Dir((uint8_t(d) + 1) & 3); }
inline Dir backOf(Dir d)  { return Dir((uint8_t(d) + 2) & 3); }
inline Dir opposite(Dir d){ return backOf(d); }
inline uint8_t dirMask(Dir d) { return uint8_t(1) << (uint8_t(d) & 3); }

// Packed cell: 2 bytes
struct Cell {
  uint8_t walls = 0; // bit0=N, bit1=E, bit2=S, bit3=W
  uint8_t flags = 0; // bit0..3 known (N,E,S,W), bit4 = visited
};

static const int MAZE_N = 9;
Cell maze[MAZE_N][MAZE_N];

inline bool inBounds(int x, int y) { return x >= 0 && x < MAZE_N && y >= 0 && y < MAZE_N; }

// 12 unreachable corner cells
bool reachableCell(int x, int y) {
  if (!inBounds(x,y)) return false;
  if ((x==0 && y==0) || (x==1 && y==0) || (x==0 && y==1)) return false;
  if ((x==MAZE_N-1 && y==0) || (x==MAZE_N-2 && y==0) || (x==MAZE_N-1 && y==1)) return false;
  if ((x==0 && y==MAZE_N-1) || (x==0 && y==MAZE_N-2) || (x==1 && y==MAZE_N-1)) return false;
  if ((x==MAZE_N-1 && y==MAZE_N-1) || (x==MAZE_N-2 && y==MAZE_N-1) || (x==MAZE_N-1 && y==MAZE_N-2)) return false;
  return true;
}

// flags helpers
inline void  markVisited(int x,int y)     { maze[y][x].flags |= 0x10; }
inline bool  cellVisited(int x,int y)     { return (maze[y][x].flags & 0x10); }
inline void  setKnown(int x,int y, Dir d) { maze[y][x].flags |= dirMask(d); }
inline bool  isKnown (int x,int y, Dir d) { return (maze[y][x].flags & dirMask(d)); }
inline void  setWall (int x,int y, Dir d) { maze[y][x].walls |= dirMask(d); }
inline bool  hasWall (int x,int y, Dir d) { return (maze[y][x].walls & dirMask(d)); }

struct Pose {
  int8_t x = 1, y = MAZE_N-2;  // set your real start cell
  Dir heading = NORTH;         // IMU-quantized heading (only at turn end)
  Dir stepDir = NORTH;         // drive direction for grid stepping
  float mmInCell = 0.0f;
  float lastRevL = 0.0f, lastRevR = 0.0f;
  float yawAlignDeg = 0.0f;    // offset so IMU aligns to grid
} pose;

// === Odom config ===
// Put your real wheel circumference here:
const float WHEEL_CIRC_MM     = 210.0f; // e.g. Ø ≈ 67 mm → ~210 mm
// Step exactly one cell:
const float ENTER_NEW_CELL_MM = CELL_MM;

// Quantize aligned yaw to N/E/S/W
Dir quantizeHeadingFromYaw(float yawDegAligned) {
  float a = fmodf(yawDegAligned, 360.0f); if (a < 0) a += 360.0f;
  int k = int((a + 45.0f) / 90.0f) & 3;
  return Dir(k);
}
inline float gridYawDeg() { return mpu.getAngleZ() + pose.yawAlignDeg; }

// Encoders → wheel revolutions
inline float leftWheelRevs()  { return encoder1.getRotation() / 360.0f; } // adjust if needed
inline float rightWheelRevs() { return encoder2.getRotation() / 360.0f; } // adjust if needed

// Avg forward delta in mm since last call
inline float deltaForwardMM() {
  float rL = leftWheelRevs();
  float rR = rightWheelRevs();
  float dL = (rL - pose.lastRevL) * WHEEL_CIRC_MM;
  float dR = (rR - pose.lastRevR) * WHEEL_CIRC_MM;
  pose.lastRevL = rL; pose.lastRevR = rR;
  return 0.5f * (dL + dR);
}

// ===== Serial ASCII map (no OLED) =====
char headingChar(Dir h) {
  switch(h){ case NORTH: return 'N'; case EAST: return 'E';
             case SOUTH: return 'S'; case WEST: return 'W'; default: return '?'; }
}

float mappingPercent() {
  int total=0, seen=0;
  for (int y=0; y<MAZE_N; ++y) for (int x=0; x<MAZE_N; ++x) {
    if (reachableCell(x,y)) { total++; if (cellVisited(x,y)) seen++; }
  }
  return total ? (100.0f*seen/total) : 0.0f;
}

void printAsciiMap() {
  Serial.println(F("\n+----- MAP (N up) -----+"));
  for (int y = MAZE_N-1; y >= 0; --y) {
    Serial.print(' ');
    for (int x = 0; x < MAZE_N; ++x) {
      char c;
      if (!reachableCell(x,y))             c = '#';
      else if (pose.x == x && pose.y == y) c = headingChar(pose.heading);
      else if (cellVisited(x,y))           c = '.';
      else                                 c = ' ';
      Serial.print(c);
      if (x < MAZE_N-1) Serial.print(' ');
    }
    Serial.println();
  }
  Serial.print(F("Pose: (")); Serial.print(pose.x); Serial.print(',');
  Serial.print(pose.y); Serial.print(F(") "));
  Serial.print(headingChar(pose.heading));
  Serial.print(F("   Mapped: ")); Serial.print(mappingPercent(),1); Serial.println(F("%"));
}

unsigned long lastPrintMs = 0;
const unsigned long PRINT_PERIOD_MS = 300;
inline void maybePrintStatus() {
  unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_PERIOD_MS) { printAsciiMap(); lastPrintMs = now; }
}

// ====== Mapping functions ======
void initMaze() {
  for (int y=0; y<MAZE_N; ++y) for (int x=0; x<MAZE_N; ++x) {
    maze[y][x] = Cell{};
    if (!reachableCell(x,y)) { maze[y][x].walls = 0x0F; maze[y][x].flags = 0x1F; }
  }
}

void setStartPose(int sx, int sy, Dir shead) {
  pose.x = sx; pose.y = sy; pose.heading = shead; pose.stepDir = shead;
  pose.mmInCell = 0.0f;
  pose.lastRevL = leftWheelRevs();
  pose.lastRevR = rightWheelRevs();
  // Align IMU so current yaw == shead*90°
  float want = float(shead) * 90.0f;
  pose.yawAlignDeg = want - mpu.getAngleZ();
  if (reachableCell(pose.x, pose.y)) markVisited(pose.x, pose.y);
}

void senseAndUpdateWalls() {
  if (!reachableCell(pose.x, pose.y)) return;

  struct SideSense { Dir d; float dist; float thr; } S[3] = {
    { pose.heading,         distFront, FRONT_TURN_MM },
    { leftOf(pose.heading), distLeft,  SIDE_WALL_PRESENT_MM },
    { rightOf(pose.heading),distRight, SIDE_WALL_PRESENT_MM }
  };

  for (auto &s : S) {
    setKnown(pose.x, pose.y, s.d);
    if (s.dist <= 0) continue; // ignore invalid
    bool wall = (s.dist < s.thr);

    int nx = pose.x, ny = pose.y;
    if (s.d == NORTH) ny++;
    else if (s.d == EAST) nx++;
    else if (s.d == SOUTH) ny--;
    else if (s.d == WEST) nx--;

    if (wall) {
      setWall(pose.x, pose.y, s.d);
      if (inBounds(nx,ny) && reachableCell(nx,ny)) {
        setKnown(nx,ny,opposite(s.d));
        setWall (nx,ny,opposite(s.d));
      }
    } else {
      if (inBounds(nx,ny) && reachableCell(nx,ny)) {
        setKnown(nx,ny,opposite(s.d));
      }
    }
  }

  markVisited(pose.x, pose.y);
}

void saveMazeToEEPROM() {
  int addr=0;
  for (int y=0;y<MAZE_N;y++) for (int x=0;x<MAZE_N;x++) {
    EEPROM.update(addr++, maze[y][x].walls);
    EEPROM.update(addr++, maze[y][x].flags);
  }
}

// === cell stepping: exactly 180 mm, then pause to scan ===
void updatePoseFromMotion() {
  float dmm = deltaForwardMM();
  pose.mmInCell += dmm;
  if (pose.mmInCell < 0) pose.mmInCell = 0;

  if (pose.mmInCell >= ENTER_NEW_CELL_MM) {
    // reached the next cell center
    switch (pose.stepDir) {
      case NORTH: pose.y++; break;
      case EAST:  pose.x++; break;
      case SOUTH: pose.y--; break;
      case WEST:  pose.x--; break;
    }

    pose.mmInCell = 0.0f;

    // guard illegal
    if (!inBounds(pose.x, pose.y) || !reachableCell(pose.x, pose.y)) {
      switch (pose.stepDir) {
        case NORTH: pose.y--; break;
        case EAST:  pose.x--; break;
        case SOUTH: pose.y++; break;
        case WEST:  pose.x++; break;
      }
      return;
    }

    // brief stop to scan cleanly
    motor1.setPWM(0);
    motor2.setPWM(0);
    cellScanLatched   = true;
    cellScanReleaseMs = millis() + CELL_STOP_MS;
  }
}

// =============== Angle error helper ===============
float angleError(float current, float target) {
  float error = target - current;
  while (error > 180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;
  return error;
}

// ===== P-hold forward mixer =====
inline float clampf(float v, float lo, float hi){ return v < lo ? lo : (v > hi ? hi : v); }
void setForwardWithTwist(float pr_base, float pl_base, float twist){
  float pr = clampf(pr_base + twist, -255, 255);
  float pl = clampf(pl_base - twist, -255, 255);
  motor1.setPWM(pr);       // right motor forward = +
  motor2.setPWM(-pl);      // left  motor forward = +
}

// ===== Simple exploration plan (left/right/front unvisited; else straight) =====
int8_t plannedTurnRel = 0; // -1=right, 0=straight, +1=left

bool openEdge(int x,int y, Dir d){
  return isKnown(x,y,d) && !hasWall(x,y,d);
}
bool neighborUnvisited(int x,int y, Dir d){
  int nx=x, ny=y;
  if (d==NORTH) ny++; else if (d==EAST) nx++; else if (d==SOUTH) ny--; else nx--;
  return inBounds(nx,ny) && reachableCell(nx,ny) && !cellVisited(nx,ny) && openEdge(x,y,d);
}
void planNextTurn() {
  Dir f = pose.stepDir, l = leftOf(f), r = rightOf(f);
  if (neighborUnvisited(pose.x, pose.y, l)) { plannedTurnRel = +1; return; }
  if (neighborUnvisited(pose.x, pose.y, r)) { plannedTurnRel = -1; return; }
  if (neighborUnvisited(pose.x, pose.y, f)) { plannedTurnRel =  0; return; }
  plannedTurnRel = 0; // default continue
}

// ===================== Setup =====================
void setup() {
  Serial.begin(9600);
  Wire.begin();
  // while (!Serial);

  Serial.println(F("Setup starting..."));
  setupMPU();
  initializeLidars();

  controller1.zeroAndSetTarget(encoder1.getRotation(), 0.0);
  controller2.zeroAndSetTarget(encoder2.getRotation(), 0.0);
  distanceController.zeroAndSetTarget(0, 100.0);

  initMaze();
  setStartPose(1, MAZE_N-2, NORTH);
  senseAndUpdateWalls();
  saveMazeToEEPROM();
  planNextTurn();
  printAsciiMap();

  Serial.println(F("Setup done."));
}

// ===================== Main Loop =====================
void loop() {
  mpu.update();
  updateLidars();
  currAngle = mpu.getAngleZ();

  static float targetAngle = 0.0f;
  unsigned long now = millis();

  // ===== Try to execute planned exploration turn early (if side is open) =====
  const float SIDE_MARGIN = 10.0f;
  if (!turning && plannedTurnRel != 0 && distFront > FRONT_TURN_MM) {
    bool wantLeft  = (plannedTurnRel > 0);
    bool wantRight = (plannedTurnRel < 0);
    bool canLeft   = wantLeft  && (distLeft  > (SIDE_WALL_PRESENT_MM + SIDE_MARGIN));
    bool canRight  = wantRight && (distRight > (SIDE_WALL_PRESENT_MM + SIDE_MARGIN));

    if (canLeft || canRight) {
      float dir = wantLeft ? +1.0f : -1.0f;
      float tgt = currAngle + 90.0f * dir;

      turnTarget    = tgt;
      turnReleaseMs = millis() + TURN_HOLD_MS;
      turning       = true;

      // nudge to start
      if (dir > 0) {
        motor1.setPWM( forwardSpeedRight * TURN_PWM_SCALE);
        motor2.setPWM( forwardSpeedLeft  * TURN_PWM_SCALE);
      } else {
        motor1.setPWM(-forwardSpeedRight * TURN_PWM_SCALE);
        motor2.setPWM(-forwardSpeedLeft  * TURN_PWM_SCALE);
      }

      plannedTurnRel = 0; // consume plan
      printLidarReadings();
      return;
    }
  }

  // ===== Turn-latch control =====
  if (turning) {
    float err = angleError(currAngle, turnTarget);  // [-180, 180]

    if (err > TURN_TOL) { // increase angle -> left turn
      motor1.setPWM( forwardSpeedRight * TURN_PWM_SCALE);
      motor2.setPWM( forwardSpeedLeft  * TURN_PWM_SCALE);
    } else if (err < -TURN_TOL) { // decrease angle -> right turn
      motor1.setPWM(-forwardSpeedRight * TURN_PWM_SCALE);
      motor2.setPWM(-forwardSpeedLeft  * TURN_PWM_SCALE);
    } else {
      motor1.setPWM(0);
      motor2.setPWM(0);
    }

    if (now >= turnReleaseMs && err <= TURN_TOL && err >= -TURN_TOL) {
      turning = false;

      // adopt new grid direction and fight drift
      pose.heading = quantizeHeadingFromYaw(gridYawDeg());
      pose.stepDir = pose.heading;
      pose.mmInCell = 0.0f;
      float want = float(pose.stepDir) * 90.0f;
      pose.yawAlignDeg += (want - gridYawDeg());

      senseAndUpdateWalls();
      saveMazeToEEPROM();
      planNextTurn();
      printAsciiMap();
    }

    printLidarReadings();
    return;
  }

  // ===== Scan at cell centers after a brief stop =====
  if (cellScanLatched) {
    if (millis() < cellScanReleaseMs) {
      // keep motors off while pausing
      motor1.setPWM(0);
      motor2.setPWM(0);
      printLidarReadings();
      return;
    }
    cellScanLatched = false;

    // stationary at cell center -> scan & plan
    senseAndUpdateWalls();
    saveMazeToEEPROM();
    planNextTurn();
    printAsciiMap();
    // fall through to driving
  }

  // ===== Reactive decision logic (old behavior kept) =====
  float effectiveFrontTurn = FRONT_TURN_MM;
  if (distLeft < SIDE_WALL_PRESENT_MM && distRight < SIDE_WALL_PRESENT_MM) {
    effectiveFrontTurn += FRONT_TURN_MM_CORRIDOR_BONUS;
  }

  if (distFront < effectiveFrontTurn) {
    // wall on left only -> turn right
    if (distLeft < SIDE_WALL_PRESENT_MM && distRight > SIDE_WALL_PRESENT_MM) {
      targetAngle    = currAngle - 90.0f;
      turnTarget     = targetAngle;
      turnReleaseMs  = now + TURN_HOLD_MS;
      turning        = true;

      // starting nudge (right turn)
      motor1.setPWM(-forwardSpeedRight*0.25f);
      motor2.setPWM(-forwardSpeedLeft *0.25f);

      printLidarReadings();
      return;
    }

    // wall on right only -> turn left
    if (distRight < SIDE_WALL_PRESENT_MM && distLeft > SIDE_WALL_PRESENT_MM) {
      targetAngle    = currAngle + 90.0f;
      turnTarget     = targetAngle;
      turnReleaseMs  = now + TURN_HOLD_MS;
      turning        = true;

      // starting nudge (left turn)
      motor1.setPWM( forwardSpeedRight*0.25f);
      motor2.setPWM( forwardSpeedLeft *0.25f);

      printLidarReadings();
      return;
    }

    // corridor (both sides have walls) -> forced 90°
    if (distLeft < SIDE_WALL_PRESENT_MM && distRight < SIDE_WALL_PRESENT_MM) {
      targetAngle    = currAngle + 90.0f;
      turnTarget     = targetAngle;
      turnReleaseMs  = now + TURN_HOLD_MS;
      turning        = true;

      motor1.setPWM( forwardSpeedRight*0.25f);
      motor2.setPWM( forwardSpeedLeft *0.25f);

      printLidarReadings();
      return;
    }

    // both sides open → pick clearer side (no bypass)
    if (distLeft > SIDE_WALL_PRESENT_MM && distRight > SIDE_WALL_PRESENT_MM) {
      int8_t dir = (distLeft >= distRight) ? +1 : -1;  // +1 = left, -1 = right
      targetAngle   = currAngle + (90.0f * dir);
      turnTarget    = targetAngle;
      turnReleaseMs = now + TURN_HOLD_MS;
      turning       = true;

      if (dir > 0) {
        motor1.setPWM( forwardSpeedRight * TURN_PWM_SCALE);
        motor2.setPWM( forwardSpeedLeft  * TURN_PWM_SCALE);
      } else {
        motor1.setPWM(-forwardSpeedRight * TURN_PWM_SCALE);
        motor2.setPWM(-forwardSpeedLeft  * TURN_PWM_SCALE);
      }

      printLidarReadings();
      return;
    }
  }
  else if (distLeft < SIDE_TOO_CLOSE_MM) {    // too close to left
    LeftSpeed  = (forwardSpeedLeft + 30)*0.5f;
    RightSpeed = (forwardSpeedRight - 30)*0.5f;

    float err   = angleError(gridYawDeg(), float(pose.stepDir) * 90.0f);
    if (fabs(err) < 1.0f) err = 0;
    float twist = clampf(0.6f * err, -20, 20);

    setForwardWithTwist(RightSpeed, LeftSpeed, twist);
    updatePoseFromMotion();
  }
  else if (distRight < SIDE_TOO_CLOSE_MM) {   // too close to right
    LeftSpeed  = (forwardSpeedLeft - 30)*0.5f;
    RightSpeed = (forwardSpeedRight + 30)*0.5f;

    float err   = angleError(gridYawDeg(), float(pose.stepDir) * 90.0f);
    if (fabs(err) < 1.0f) err = 0;
    float twist = clampf(0.6f * err, -20, 20);

    setForwardWithTwist(RightSpeed, LeftSpeed, twist);
    updatePoseFromMotion();
  }
  else { // straight
    float err   = angleError(gridYawDeg(), float(pose.stepDir) * 90.0f);
    if (fabs(err) < 1.0f) err = 0;
    float twist = clampf(0.6f * err, -20, 20);

    setForwardWithTwist(forwardSpeedRight, forwardSpeedLeft, twist);
    updatePoseFromMotion();
  }

  // Optional odom debug:
  // static unsigned long lastDBG=0;
  // if (millis()-lastDBG>300) {
  //   lastDBG = millis();
  //   Serial.print(F("mmInCell=")); Serial.print(pose.mmInCell,1);
  //   Serial.print(F(" dmm="));     Serial.print(deltaForwardMM(),1);
  //   Serial.print(F(" encL="));    Serial.print(encoder1.getRotation());
  //   Serial.print(F(" encR="));    Serial.println(encoder2.getRotation());
  // }

  maybePrintStatus();
  printLidarReadings();
}

#include "global.hpp"
#include "Motor.hpp"
#include "Encoder.hpp"
#include "PIDController.hpp"
#include "mpu.hpp"
#include "lidar.hpp"

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

// Angle PID Controller for rotation control (from 3.2)
mtrn3100::PIDController angleController(3.5, 0.3, 0.1);

// --- Custom Angle Tracking & Drift Correction (from 3.2) ---
float ahrs_angle = 0.0;          // Our manually integrated angle, more robust to drift.
float gyro_z_bias = 0.0;         // Stores the dynamically calculated gyro bias.
unsigned long last_update_micros = 0; // For precise dt calculation.
bool is_calibrating = true;      // Flag to control initial calibration.
#define CALIBRATION_SAMPLES 1000 // Number of samples for calibration.
int sample_count = 0;            // Counter for calibration samples.
float gyro_z_sum = 0;            // Sum for averaging samples.

// Command execution variables (from 3.3)
size_t currentCommandIndex = 0;
bool commandInProgress = false;
unsigned long lastCommandTime = 0;

// Store initial gyro bias to reset before each rotation (for consistent performance)
float initial_gyro_bias = 0.0;

// Lidar-based course correction parameters
const float WALL_TOO_CLOSE_THRESHOLD = 80.0f;  // mm - when to trigger correction
const float WALL_SAFE_DISTANCE = 120.0f;       // mm - target distance after correction
const float CORRECTION_ANGLE = 15.0f;          // degrees - how much to adjust
const int MAX_CORRECTION_ATTEMPTS = 3;         // prevent infinite correction loops

// Add wall alignment parameters after course correction parameters (around line 50)
const float TARGET_WALL_DISTANCE = 100.0f;     // mm - ideal distance to maintain from walls
const float ALIGNMENT_KP = 0.5f;               // Proportional gain for alignment correction
const float ALIGNMENT_MAX_ADJUST = 20.0f;      // Max speed adjustment (PWM)

// Add after ALIGNMENT_MAX_ADJUST (around line 53)
const float MAX_WALL_DISTANCE = 200.0f;        // mm - beyond this, consider wall "disappeared"
const float MEMORY_DECAY_FACTOR = 0.5f;        // How much to reduce correction strength when using memory

// Movement parameters (configurable)
const float FORWARD_DISTANCE_MM = 330.0f;  // Distance to move forward (can be adjusted)
const float forwardSpeedRight = 100.0f;
const float forwardSpeedLeft = 98.0f;

// Encoder tracking for forward movement
float encoder1Start = 0.0;
float encoder2Start = 0.0;

// Target angle for rotations
float targetAngle = 0.0;

// Add normalizeAngle function after angleError (around line 95, before executeRotation)
float normalizeAngle(float angle) {
    angle = fmod(angle + 180.0f, 360.0f) - 180.0f;
    return angle;
}

// Lidar-based course correction function
bool performCourseCorrection() {
    updateLidars();  // Get fresh lidar readings
    
    bool needsCorrection = false;
    char correctionDirection = 'n';  // 'l' = left too close, 'r' = right too close
    
    // Determine if correction is needed and which direction
    if (distLeft < WALL_TOO_CLOSE_THRESHOLD) {
        needsCorrection = true;
        correctionDirection = 'l';  // Too close to left wall, need to turn right
        Serial.print("COURSE CORRECTION: Too close to LEFT wall (");
        Serial.print(distLeft, 1);
        Serial.println("mm)");
    } else if (distRight < WALL_TOO_CLOSE_THRESHOLD) {
        needsCorrection = true;
        correctionDirection = 'r';  // Too close to right wall, need to turn left
        Serial.print("COURSE CORRECTION: Too close to RIGHT wall (");
        Serial.print(distRight, 1);
        Serial.println("mm)");
    }
    
    if (!needsCorrection) {
        return false;  // No correction needed
    }
    
    // Perform correction sequence
    Serial.println("Starting course correction sequence...");
    
    // Step 1: Turn away from the wall
    float correctionTarget;
    if (correctionDirection == 'l') {
        // Turn right to get away from left wall
        correctionTarget = normalizeAngle(ahrs_angle - CORRECTION_ANGLE);
        Serial.print("Turning RIGHT ");
    } else {
        // Turn left to get away from right wall  
        correctionTarget = normalizeAngle(ahrs_angle + CORRECTION_ANGLE);
        Serial.print("Turning LEFT ");
    }
    Serial.print(CORRECTION_ANGLE, 1);
    Serial.println("° to avoid wall");
    
    // Execute correction rotation
    angleController.zeroAndSetTarget(ahrs_angle, correctionTarget);
    unsigned long correctionStart = millis();
    
    while (millis() - correctionStart < 3000) {  // 3 second timeout
        mpu.update();
        
        // Update angle calculation
        unsigned long now_micros = micros();
        float dt = (now_micros - last_update_micros) / 1000000.0f;
        last_update_micros = now_micros;
        float gyro_z = mpu.getGyroZ();
        ahrs_angle += (gyro_z - gyro_z_bias) * dt;
        ahrs_angle = normalizeAngle(ahrs_angle);
        
        float error = angleError(ahrs_angle, correctionTarget);
        float turnSpeed = constrain(angleController.compute(ahrs_angle), -25.0f, 25.0f);
        
        if (abs(error) > 5.0f) {
            motor1.setPWM(turnSpeed);
            motor2.setPWM(turnSpeed);
        } else {
            // Correction rotation complete
            motor1.setPWM(0);
            motor2.setPWM(0);
            break;
        }
        delay(10);
    }
    
    motor1.setPWM(0);
    motor2.setPWM(0);
    delay(200);
    
    // Step 2: Check if we're now at a safe distance
    updateLidars();
    float currentDistance = (correctionDirection == 'l') ? distLeft : distRight;
    
    if (currentDistance >= WALL_SAFE_DISTANCE) {
        Serial.print("Course correction successful! Distance now: ");
        Serial.print(currentDistance, 1);
        Serial.println("mm");
        return true;
    } else {
        Serial.print("Correction insufficient. Distance still: ");
        Serial.print(currentDistance, 1);
        Serial.println("mm");
        return false;  // May need additional correction
    }
}

// Add wall alignment function after performCourseCorrection (around line 165)
void applyWallAlignment(float& leftSpeed, float& rightSpeed) {
    static float lastLeftDist = TARGET_WALL_DISTANCE;
    static float lastRightDist = TARGET_WALL_DISTANCE;
    
    // Get current distances
    float leftDist = distLeft;
    float rightDist = distRight;
    
    // Update memory if valid readings
    bool leftValid = (leftDist > 0 && leftDist < MAX_WALL_DISTANCE && leftDist != 255);
    bool rightValid = (rightDist > 0 && rightDist < MAX_WALL_DISTANCE && rightDist != 255);
    
    if (leftValid) {
        lastLeftDist = leftDist;
    }
    if (rightValid) {
        lastRightDist = rightDist;
    }
    
    // If both invalid, no correction possible
    if (!leftValid && !rightValid) {
        Serial.println(F("No walls detected - no alignment correction"));
        return;
    }
    
    float effectiveLeft = leftValid ? leftDist : lastLeftDist;
    float effectiveRight = rightValid ? rightDist : lastRightDist;
    
    // Calculate correction factor (reduce strength if using memory)
    float correctionStrength = 1.0f;
    if (!leftValid) correctionStrength *= MEMORY_DECAY_FACTOR;
    if (!rightValid) correctionStrength *= MEMORY_DECAY_FACTOR;
    
    // If effectively both "walls" available (current or remembered)
    if (effectiveLeft < MAX_WALL_DISTANCE && effectiveRight < MAX_WALL_DISTANCE) {
        float centerError = effectiveLeft - effectiveRight;
        float correction = centerError * ALIGNMENT_KP * correctionStrength;
        correction = constrain(correction, -ALIGNMENT_MAX_ADJUST, ALIGNMENT_MAX_ADJUST);
        
        leftSpeed -= correction;
        rightSpeed += correction;
        
        Serial.print(F("Centering "));
        if (!leftValid || !rightValid) Serial.print(F("(using memory) "));
        Serial.print(F("Error: "));
        Serial.println(centerError, 1);
    } 
    // Single side (using current or memory)
    else if (effectiveLeft < MAX_WALL_DISTANCE) {
        float error = effectiveLeft - TARGET_WALL_DISTANCE;
        float correction = error * ALIGNMENT_KP * correctionStrength;
        correction = constrain(correction, -ALIGNMENT_MAX_ADJUST, ALIGNMENT_MAX_ADJUST);
        
        leftSpeed -= correction;
        rightSpeed += correction;
        
        Serial.print(F("Left follow "));
        if (!leftValid) Serial.print(F("(memory) "));
        Serial.print(F("Error: "));
        Serial.println(error, 1);
    } 
    else if (effectiveRight < MAX_WALL_DISTANCE) {
        float error = effectiveRight - TARGET_WALL_DISTANCE;
        float correction = error * ALIGNMENT_KP * correctionStrength;
        correction = constrain(correction, -ALIGNMENT_MAX_ADJUST, ALIGNMENT_MAX_ADJUST);
        
        leftSpeed += correction;
        rightSpeed -= correction;
        
        Serial.print(F("Right follow "));
        if (!rightValid) Serial.print(F("(memory) "));
        Serial.print(F("Error: "));
        Serial.println(error, 1);
    }
}

void setup() {
    Serial.begin(9600);
    Wire.begin();
    while (!Serial);  // Wait for Serial

    Serial.println("Encoder Navigation Setup starting...");

    setupMPU();           // Initialize MPU
    initializeLidars();   // Initialize LIDARs for course correction

    // --- Initial Gyro Bias Calibration (from 3.2) ---
    Serial.println("Calibrating Gyro... Keep robot stationary.");

    // Update initial calibration in setup() to match dynamic style (replace lines 69-74)
    gyro_z_sum = 0;
    for (int i = 0; i < 100; i++) {  // Match dynamic sample count
        mpu.update();
        gyro_z_sum += mpu.getGyroZ();
        delay(2);
    }
    gyro_z_bias = gyro_z_sum / 100;
    gyro_z_sum = 0;
    
    // Store the fresh initial bias for resetting before each rotation
    initial_gyro_bias = gyro_z_bias;

    Serial.print("Gyro Z bias calculated: ");
    Serial.println(gyro_z_bias);

    // Initialize angle controller
    angleController.zeroAndSetTarget(0.0, 0.0);

    Serial.println("Setup done. Ready to execute commands.");
    last_update_micros = micros(); // Initialize timer for angle calculation
}

// Angle error calculation with wrap-around handling
float angleError(float current, float target) {
    float error = target - current;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

// Function to execute rotation using 3.2 method
bool executeRotation(float targetAngleLocal) {
    static unsigned long rotationStartTime = 0;
    static bool rotationStarted = false;
    static int stableCount = 0;  // Counter for stable readings
    
    mpu.update();
    
    // --- Custom Angle Calculation with Dynamic Bias Correction (from 3.2) ---
    unsigned long now_micros = micros();
    float dt = (now_micros - last_update_micros) / 1000000.0f;
    last_update_micros = now_micros;
    float gyro_z = mpu.getGyroZ();
    ahrs_angle += (gyro_z - gyro_z_bias) * dt;

    // After calculating ahrs_angle (insert after line 109)
    ahrs_angle = normalizeAngle(ahrs_angle);

    float currentAngle = ahrs_angle;
    float error = angleError(currentAngle, targetAngleLocal);
    
    // Update PID controller target (only reset if this is a new rotation)
    // Note: zeroAndSetTarget resets integral/derivative terms
    if (!rotationStarted) {
        angleController.zeroAndSetTarget(currentAngle, targetAngleLocal);
        Serial.println("PID reset for new rotation");
    }
    float turnSpeed = constrain(angleController.compute(currentAngle), -35.0f, 35.0f);

    // Initialize rotation timer
    if (!rotationStarted) {
        rotationStartTime = millis();
        rotationStarted = true;
        stableCount = 0;
    }

    // DISABLE dynamic recalibration during rotation to maintain first rotation quality
    // This prevents the degradation you observed in subsequent rotations
    // Reset any accumulated calibration data to prevent interference
    stableCount = 0;
    sample_count = 0;
    gyro_z_sum = 0;

    // After recalibration (insert after line 138 inside the if, after ahrs_angle correction)
    ahrs_angle = normalizeAngle(ahrs_angle);

    // Check rotation timeout (prevent infinite loops)
    unsigned long rotationTime = millis() - rotationStartTime;
    bool timeoutReached = rotationTime > 10000;  // 10 second timeout

    // Check if rotation is complete (increased deadband to 6 degrees)
    if (abs(error) > 6.0f && !timeoutReached) {
        motor1.setPWM(turnSpeed);
        motor2.setPWM(turnSpeed);
        Serial.print("Rotating - Current: "); Serial.print(currentAngle, 2);
        Serial.print(" | Target: "); Serial.print(targetAngleLocal, 2);
        Serial.print(" | Error: "); Serial.print(error, 2);
        Serial.print(" | Time: "); Serial.println(rotationTime);
        return false;  // Still rotating
    } else {
        motor1.setPWM(0);
        motor2.setPWM(0);
        rotationStarted = false;  // Reset for next rotation
        stableCount = 0;
        
        if (timeoutReached) {
            Serial.print("Rotation timeout reached after ");
            Serial.print(rotationTime);
            Serial.println("ms");
        } else {
            Serial.print("Rotation complete - Final error: ");
            Serial.println(error, 2);
            
            // LIDAR COURSE CORRECTION: Check if we're too close to walls after rotation
            static int correctionAttempts = 0;
            if (correctionAttempts < MAX_CORRECTION_ATTEMPTS) {
                bool correctionPerformed = performCourseCorrection();
                if (correctionPerformed) {
                    correctionAttempts++;
                    Serial.print("Course correction attempt ");
                    Serial.print(correctionAttempts);
                    Serial.print(" of ");
                    Serial.println(MAX_CORRECTION_ATTEMPTS);
                    
                    // Continue rotation with corrected position - don't return yet
                    rotationStarted = false;  // Allow re-entry to rotation logic
                    return false;  // Continue rotation to reach proper final angle
                } else {
                    correctionAttempts = 0;  // Reset for next rotation
                }
            } else {
                Serial.println("Max correction attempts reached - accepting current position");
                correctionAttempts = 0;  // Reset for next rotation
            }
        }
        
        delay(300); // Reduced pause after rotation
        return true;  // Rotation complete
    }
}

// Function to execute forward movement using encoders
bool executeForward() {
    float distance1 = abs(encoder1.getDistanceMM() - encoder1Start);
    float distance2 = abs(encoder2.getDistanceMM() - encoder2Start);
    float avgDistance = (distance1 + distance2) / 2.0;

    if (avgDistance < FORWARD_DISTANCE_MM) {
        updateLidars();  // Get fresh readings

        float adjustedLeft = forwardSpeedLeft;
        float adjustedRight = forwardSpeedRight;

        applyWallAlignment(adjustedLeft, adjustedRight);

        // Then set PWM with adjusted speeds
        motor1.setPWM(adjustedRight);
        motor2.setPWM(-adjustedLeft);
        
        Serial.print(F("Moving forward - Distance: ")); 
        Serial.print(avgDistance / 10.0f, 1);
        Serial.print(F("mm / ")); 
        Serial.print(FORWARD_DISTANCE_MM / 10.0f, 1);
        Serial.println(F("mm"));
        
        return false;  // Still moving
    } else {
        motor1.setPWM(0);
        motor2.setPWM(0);
        Serial.println("Forward movement complete");
        delay(500); // Brief pause after forward movement
        return true;   // Movement complete
    }
}

void loop() {
    static unsigned long lastDisplayUpdate = 0;
    static unsigned long lastIdleCalib = 0;
    static unsigned long lastLidarUpdate = 0;
    unsigned long now = millis();

    // Update lidars periodically for course correction
    if (now - lastLidarUpdate >= 100) {  // Update every 100ms
        updateLidars();
        lastLidarUpdate = now;
    }

    // Check if there are more commands to execute
    if (currentCommandIndex < comm.length()) {
        char cmd = comm.charAt(currentCommandIndex);

        if (!commandInProgress) {
            Serial.print("Executing command: ");
            Serial.println(cmd);

            if (cmd == 'l') {
                // Turn left (90 degrees counterclockwise)
                // RESET TO FIRST ROTATION CONDITIONS
                gyro_z_bias = initial_gyro_bias;  // Use fresh original bias
                ahrs_angle = normalizeAngle(ahrs_angle);
                targetAngle = normalizeAngle(ahrs_angle + 90.0);
                // Reset PID controller for new rotation
                angleController.zeroAndSetTarget(ahrs_angle, targetAngle);
                Serial.print("LEFT TURN - Fresh reset | Current: ");
                Serial.print(ahrs_angle, 2);
                Serial.print("° -> Target: ");
                Serial.print(targetAngle, 2);
                Serial.println("°");
                commandInProgress = true;
            } else if (cmd == 'r') {
                // Turn right (90 degrees clockwise)
                // RESET TO FIRST ROTATION CONDITIONS
                gyro_z_bias = initial_gyro_bias;  // Use fresh original bias
                ahrs_angle = normalizeAngle(ahrs_angle);
                targetAngle = normalizeAngle(ahrs_angle - 90.0);
                // Reset PID controller for new rotation
                angleController.zeroAndSetTarget(ahrs_angle, targetAngle);
                Serial.print("RIGHT TURN - Fresh reset | Current: ");
                Serial.print(ahrs_angle, 2);
                Serial.print("° -> Target: ");
                Serial.print(targetAngle, 2);
                Serial.println("°");
                commandInProgress = true;
            } else if (cmd == 'f') {
                // Move forward (no PID needed for encoder-based movement)
                encoder1Start = encoder1.getDistanceMM();
                encoder2Start = encoder2.getDistanceMM();
                Serial.println("Starting forward movement (no PID)");
                commandInProgress = true;
            } else {
                Serial.print("Unknown command: ");
                Serial.println(cmd);
                currentCommandIndex++;  // Skip unknown command
            }
        }

        // Execute the current command
        if (commandInProgress) {
            bool commandComplete = false;

            if (cmd == 'l' || cmd == 'r') {
                // Execute rotation
                commandComplete = executeRotation(targetAngle);
            } else if (cmd == 'f') {
                // Execute forward movement
                commandComplete = executeForward();
            }

            // Move to next command if current one is complete
            if (commandComplete) {
                commandInProgress = false;
                currentCommandIndex++;
                
                // Reset PID controller when transitioning between movements
                if (cmd == 'l' || cmd == 'r') {
                    angleController.zeroAndSetTarget(ahrs_angle, ahrs_angle);
                    Serial.println("PID reset after rotation completion");
                    // Also add normalization in idle calibration (after updating gyro_z_bias in the idle loop)
                    ahrs_angle = normalizeAngle(ahrs_angle);
                }
                
                Serial.print("Command '"); Serial.print(cmd); 
                Serial.println("' completed. Moving to next command.");
            }
        }
    } else {
        // All commands completed - stop motors
        motor1.setPWM(0);
        motor2.setPWM(0);
        
        if (now - lastDisplayUpdate >= 2000) {  // Update every 2 seconds
            Serial.println("All commands completed. Robot idle.");
            lastDisplayUpdate = now;
        }

        // DISABLE idle calibration to maintain first rotation performance 
        // Just normalize angle periodically to prevent accumulation
        if (now - lastIdleCalib >= 10000) {  // Every 10 seconds, just normalize
            ahrs_angle = normalizeAngle(ahrs_angle);
            lastIdleCalib = now;
            Serial.println("Idle: Angle normalized (bias unchanged)");
        }
    }

    // Also, to address potential integral windup in PID, add this to PID reset points (e.g., after zeroAndSetTarget calls):
    angleController.compute(ahrs_angle);  // Dummy compute to clear any remaining state

    delay(10);  // Small delay for loop stability
}

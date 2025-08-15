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
mtrn3100::PIDController distanceController(2.0, 0.0, 0.0);  // Kp, Ki, Kd for distance control

// Angle PID Controller for rotation control (tuned for stability)
mtrn3100::PIDController angleController(2.0, 0.00, 0.0);  // Reduced gains to prevent oscillation

// Store initial angle
float initialAngle = 0.0;
size_t currentCommandIndex = 0;
bool commandInProgress = false;
unsigned long lastCommandTime = 0;
// const float forwardSpeedRight = 130.0f;
// const float forwardSpeedLeft = 127.5f;

const float forwardSpeedRight = 100.0f;
const float forwardSpeedLeft = 98.0f;
float encoder1Start = 0.0;
float encoder2Start = 0.0;
float LeftSpeed = 0;
float RightSpeed = 0;

// Left wheel needs to be slower

// Wall follow controller
mtrn3100::WallFollow wallFollow(MOT1PWM, MOT1DIR, MOT2PWM, MOT2DIR, 20.0f, 5.0f, 2.0f);

void setup() {
    Serial.begin(9600);
    Wire.begin();
    while (!Serial);  // Wait for Serial (if needed)

    // // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    // if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    // Serial.println(F("SSD1306 allocation failed"));
    // for(;;); // Don't proceed, loop forever
    // }

    // // Clear the buffer
    // display.clearDisplay();
    // display.setTextSize(1);
    // display.setTextColor(SSD1306_WHITE);
    // display.setCursor(0,0);
    // display.println(F("Front Lidar Ready"));
    // display.println(F("Distance Display"));
    // display.display();
    // delay(2000);
  
    Serial.println("Setup starting...");

    setupMPU();           // Initializes MPU
    initializeLidars();   // Initializes LIDARs
    controller1.zeroAndSetTarget(encoder1.getRotation(), 0.0);
    controller2.zeroAndSetTarget(encoder2.getRotation(), 0.0);
    
    // Initialize distance PID controller with target distance of 100mm
    distanceController.zeroAndSetTarget(0, 100.0);  // Target 100mm from front wall
    wallFollow.begin();   // Start the wall follow behavior

    Serial.println("Setup done.");
}

// Add at top of file
bool shouldMoveForward = false;
float angleError(float current, float target) {
    float error = target - current;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

// void screen(char command, float value) {
//   display.clearDisplay();
//   display.setCursor(0, 0);

//   display.setTextSize(1);
//   display.print("Cmd: ");
//   display.print(command);

//   display.setCursor(0, 10);
//   display.setTextSize(1);

//   if (command == 'f') {
//     display.print("Distance: ");
//     display.print(value, 1);
//     display.println(" mm");
//   } else if (command == 'l' || command == 'r') {
//     display.print("Angle: ");
//     display.print(value, 1);
//     display.println(" deg");
//   } else {
//     display.print("Idle");
//   }

//   display.display();
// }

void loop() {
    mpu.update();
    updateLidars();
    currAngle = mpu.getAngleZ();

    static float targetAngle = 0.0;
    static unsigned long lastDisplayUpdate = 0;
    unsigned long now = millis();

    if (currentCommandIndex < comm.length()) {
        char cmd = comm.charAt(currentCommandIndex);

        if (!commandInProgress) {
            Serial.print("Executing command: ");
            Serial.println(cmd);

            // screen(cmd, 0.0f);  // Initial display update

            if (cmd == 'l') {
                targetAngle = currAngle + 90.0;
                commandInProgress = true;
            } else if (cmd == 'r') {
                targetAngle = currAngle - 90.0;
                commandInProgress = true;
            } else if (cmd == 'f') {
                encoder1Start = encoder1.getDistanceMM();
                encoder2Start = encoder2.getDistanceMM();
                motor1.setPWM(forwardSpeedRight);
                motor2.setPWM(-forwardSpeedLeft);
                commandInProgress = true;
            } else {
                Serial.print("Unknown command: ");
                Serial.println(cmd);
                currentCommandIndex++;  // Skip unknown
            }
        }

        // Turn left
        if (commandInProgress && cmd == 'l') {
            if (now - lastDisplayUpdate >= 100) {
                // screen(cmd, currAngle);
                lastDisplayUpdate = now;
            }

            if (angleError(currAngle, targetAngle) > 3.0f) {
                motor1.setPWM(forwardSpeedRight*0.25);
                motor2.setPWM(forwardSpeedLeft*0.25);
            } else {
                motor1.setPWM(0);
                motor2.setPWM(0);
                commandInProgress = false;
                currentCommandIndex++;
            }
        }

        // Turn right
        else if (commandInProgress && cmd == 'r') {
            if (now - lastDisplayUpdate >= 100) {
                // screen(cmd, currAngle);
                lastDisplayUpdate = now;
            }

            if (angleError(currAngle, targetAngle) < -3.0f) {
                motor1.setPWM(-forwardSpeedRight*0.25);
                motor2.setPWM(-forwardSpeedLeft*0.25);
            } else {
                motor1.setPWM(0);
                motor2.setPWM(0);
                commandInProgress = false;
                currentCommandIndex++;
            }
        }

        // Move forward
       else if (commandInProgress && cmd == 'f') {
            float d1 = encoder1.getDistanceMM() - encoder1Start;   // left wheel (per your defines)
            float d2 = encoder2.getDistanceMM() - encoder2Start;   // right wheel

            // Make it robust regardless of encoder sign convention
            float rawDistance = 0.5f * (fabsf(d1) + fabsf(d2));   

            if (now - lastDisplayUpdate >= 100) {
                // screen(cmd, rawDistance);
                lastDisplayUpdate = now;

                Serial.print("Raw Distance (Encoder2): ");
                Serial.println(rawDistance, 1);
                Serial.print("Encoder2.get(): ");
                Serial.print(encoder2.getDistanceMM(), 1);
                Serial.print(" | Start: ");
                Serial.println(encoder2Start, 1);
                Serial.print("Encoder1 (mm): ");
                Serial.println(encoder1.getDistanceMM(), 1);

                Serial.print("Encoder2 (mm): ");
                Serial.println(encoder2.getDistanceMM(), 1);

            }
            // if (distFront < 105.00) {
            //     motor1.setPWM(0);
            //     motor2.setPWM(0);
            //     commandInProgress = false;
            //     currentCommandIndex++;
            //     if (currentCommandIndex > comm.length()) {
            //         motor1.setPWM(0);
            //         motor2.setPWM(0);
            //     }
            // }


            // This could be way too simple but if it keeps looping through my thoughts are that it would only jump into this loop when needed?
            // Sorry hard to test without Remi - can look at it tomorrow

            if (rawDistance < 270.0f) {
                motor1.setPWM(forwardSpeedRight);
                motor2.setPWM(-forwardSpeedLeft);
                if (distFront < 105.00) {
                    motor1.setPWM(0);
                    motor2.setPWM(0);
                    commandInProgress = false;
                    currentCommandIndex++;
                }
                if (distLeft < 30) {
                    LeftSpeed = (forwardSpeedLeft + 30)*0.5;
                    RightSpeed = (forwardSpeedRight - 30)*0.5;
                    motor1.setPWM(RightSpeed);
                    motor2.setPWM(-LeftSpeed);
                }

                if (distRight < 30) {
                    LeftSpeed = (forwardSpeedLeft - 30)*0.5;
                    RightSpeed = (forwardSpeedRight + 30)*0.5;
                    motor1.setPWM(RightSpeed);
                    motor2.setPWM(-LeftSpeed);
                }
                if (currentCommandIndex > comm.length()) {
                    motor1.setPWM(0);
                    motor2.setPWM(0);
                }
            } else {
                // motor1.setPWM(0);
                // motor2.setPWM(0);
                commandInProgress = false;
                currentCommandIndex++;
            }

    } else {
        motor1.setPWM(0);
        motor2.setPWM(0);
        if (now - lastDisplayUpdate >= 500) {  // Idle update slower
            // screen('-', 0.0f);
            lastDisplayUpdate = now;
        }
    }

    printLidarReadings();
    }
}

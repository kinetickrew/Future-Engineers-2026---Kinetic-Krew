#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>


// ==========================================
//           PIN & HARDWARE DEFINITIONS
// ==========================================
#define SERVO_PIN 13


// Left Motor Pins
#define MOTOR_IN1  25
#define MOTOR_IN2  26
#define MOTOR_PWM  33


// I2C Multiplexer Channels (HW-617)
#define MUX_CH_LEFT   0
#define MUX_CH_CENTER 1  // New Center LiDAR channel
#define MUX_CH_RIGHT  2
#define MUX_CH_BNO    4


#define TFLUNA_I2C_ADDR 0x10


// ==========================================
//          CONFIGURABLE VARIABLES
// ==========================================
const float SLOW_STEER_THRESHOLD = 45.0; // Angle (in degrees) where the servo starts returning to center
const int STRAIGHT_SPEED = 255;  // Cruise speed for driving straight (0-255)
const int TURN_SPEED     = 180;  // Controlled speed for turning to prevent overshooting
const int BACKWARD_SPEED = -255; // Speed for backing up (if needed)


const int SERVO_CENTER   = 90;   // Dead-center steering alignment
const int SERVO_MAX_LEFT = 135;  // Physical mechanical limit for left turn
const int SERVO_MAX_RIGHT= 45;   // Physical mechanical limit for right turn


// Change to true if your steering corrections move backwards during testing
const bool INVERT_STEERING = false;


const float STEERING_KP  = 2.0;  // Sensitivity adjustment for straight-line micro-corrections
const int SPIKE_THRESHOLD = 50;  // Trigger turning mode if sensor difference changes by this many cm
const int MAX_TURNS      = 12;   // Total number of turns allowed before tracking the final stop distance


// ==========================================
//          GLOBAL STATE VARIABLES
// ==========================================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Servo steeringServo;


enum RobotState { DRIVING_STRAIGHT, TURNING, ROBOT_STOPPED };
RobotState currentState = DRIVING_STRAIGHT;


float straightTargetHeading = 0.0;
float turnTargetHeading     = 0.0;
bool isTurningLeft          = false;


int totalTurnsCount         = 0;
bool hasTurnedOnce          = false; // Becomes true permanently on the first turn
bool lockedDirectionLeft    = false; // Remembers if our layout is strictly Left or Right


// Global telemetry variables
int16_t currentLeftDist     = -1;
int16_t currentCenterDist   = -1; // Center LiDAR distance variable
int16_t currentRightDist    = -1;
int finalServoAngle         = 90;
float headingError          = 0.0;
float angleDifference       = 0.0;


// ==========================================
//            I2C & SENSOR FUNCTIONS
// ==========================================
void selectMuxChannel(uint8_t channel) {
if (channel > 7) return;
Wire.beginTransmission(0x70);
Wire.write(1 << channel);
Wire.endTransmission();
}


int16_t getLunaDistance(uint8_t channel) {
selectMuxChannel(channel);
Wire.beginTransmission(TFLUNA_I2C_ADDR);
Wire.write(0x00);
if (Wire.endTransmission() != 0) return -1;
Wire.requestFrom(TFLUNA_I2C_ADDR, 2);
if (Wire.available() >= 2) {
  uint8_t lowByte = Wire.read();
  uint8_t highByte = Wire.read();
  return (lowByte + (highByte << 8));
}
return -1;
}


float getCurrentHeading() {
selectMuxChannel(MUX_CH_BNO);
sensors_event_t event;
bno.getEvent(&event);
return event.orientation.x;
}


// ==========================================
//            ACTUATOR FUNCTIONS
// ==========================================
void setMotorOutput(int speed) {
  if (speed >= 0) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
  } else {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    speed = -speed; // Convert negative to positive for PWM calculation
  }
  analogWrite(MOTOR_PWM, constrain(speed, 0, 255));
}

// ==========================================
//           BEHAVIOR LOGIC MODES
// ==========================================
void checkObstacles() {
 if (currentLeftDist == -1 || currentRightDist == -1) return;


 int16_t difference = currentLeftDist - currentRightDist;


 // 1. HARD LOCK CHECK: If we already picked a direction, force evaluation down only one branch
 if (hasTurnedOnce) {
   if (lockedDirectionLeft) {
     // Robot is ONLY allowed to look for Left turns
     if (difference > SPIKE_THRESHOLD) {
       float currentHeading = getCurrentHeading();
       isTurningLeft = true;
       turnTargetHeading = currentHeading - 90.0;
       if (turnTargetHeading < 0.0) turnTargetHeading += 360.0;
       currentState = TURNING;
     }
   } else {
     // Robot is ONLY allowed to look for Right turns
     if (difference < -SPIKE_THRESHOLD) {
       float currentHeading = getCurrentHeading();
       isTurningLeft = false;
       turnTargetHeading = currentHeading + 90.0;
       if (turnTargetHeading >= 360.0) turnTargetHeading -= 360.0;
       currentState = TURNING;
     }
   }
   return; // Exit out early so the baseline code below never gets touched
 }


 // 2. FIRST TURN RUN: Runs only when hasTurnedOnce is still false
 if (difference > SPIKE_THRESHOLD) {
   float currentHeading = getCurrentHeading();
   isTurningLeft = true;
   turnTargetHeading = currentHeading - 90.0;
   if (turnTargetHeading < 0.0) turnTargetHeading += 360.0;
  
   // Set permanent directional lock to LEFT
   hasTurnedOnce = true;
   lockedDirectionLeft = true;
   Serial.println("!!! LAYOUT INITIALIZED: PERMANENT LEFT TURN ONLY LOCK ACTIVATED !!!");


   currentState = TURNING;
 }
 else if (difference < -SPIKE_THRESHOLD) {
   float currentHeading = getCurrentHeading();
   isTurningLeft = false;
   turnTargetHeading = currentHeading + 90.0;
   if (turnTargetHeading >= 360.0) turnTargetHeading -= 360.0;
  
   // Set permanent directional lock to RIGHT
   hasTurnedOnce = true;
   lockedDirectionLeft = false;
   Serial.println("!!! LAYOUT INITIALIZED: PERMANENT RIGHT TURN ONLY LOCK ACTIVATED !!!");


   currentState = TURNING;
 }
}


void driveStraightMode(float currentHeading) {
setMotorOutput(STRAIGHT_SPEED);

headingError = straightTargetHeading - currentHeading;
if (headingError > 180.0)  headingError -= 360.0;
if (headingError < -180.0) headingError += 360.0;


int steeringCorrection = (int)(headingError * STEERING_KP);
if (INVERT_STEERING) {
  finalServoAngle = SERVO_CENTER + steeringCorrection;
} else {
  finalServoAngle = SERVO_CENTER - steeringCorrection;
}
finalServoAngle = constrain(finalServoAngle, SERVO_MAX_RIGHT, SERVO_MAX_LEFT);
steeringServo.write(finalServoAngle);
}


void executeTurnMode(float currentHeading) {
setMotorOutput(TURN_SPEED);

// Calculate remaining angle to target
angleDifference = currentHeading - turnTargetHeading;
if (angleDifference > 180.0)  angleDifference -= 360.0;
if (angleDifference < -180.0) angleDifference += 360.0;
float remainingAngle = abs(angleDifference);


// Smooth steering reduction logic
if (remainingAngle < SLOW_STEER_THRESHOLD) {
  // Map the remaining angle smoothly between 0 (center) and max steering offset
  float progressFactor = remainingAngle / SLOW_STEER_THRESHOLD; // Ranges from 1.0 down to 0.0
 
  if (isTurningLeft) {
    int maxOffset = SERVO_MAX_LEFT - SERVO_CENTER;
    finalServoAngle = SERVO_CENTER + (int)(maxOffset * progressFactor);
  } else {
    int maxOffset = SERVO_CENTER - SERVO_MAX_RIGHT;
    finalServoAngle = SERVO_CENTER - (int)(maxOffset * progressFactor);
  }
} else {
  // Outside the threshold: Maintain maximum steering lock
  if (isTurningLeft) {
    finalServoAngle = SERVO_MAX_LEFT;
  } else {
    finalServoAngle = SERVO_MAX_RIGHT;
  }
}


// Apply the dynamically adjusted steering angle
steeringServo.write(finalServoAngle);


// End-of-turn check
if (remainingAngle < 6.0) {
  totalTurnsCount++;
   // Note: The stop condition has been moved to loop() to allow
  // the robot to straighten out and check the center wall distance.
  steeringServo.write(SERVO_CENTER);
  finalServoAngle = SERVO_CENTER;
  straightTargetHeading = turnTargetHeading;
  currentState = DRIVING_STRAIGHT;
  delay(150);
}
}


void printTelemetry(float currentHeading) {
 if (currentState == DRIVING_STRAIGHT) Serial.print("MODE: STRAIGHT");
 else if (currentState == TURNING)     Serial.print("MODE: TURNING ");


 Serial.print(" | L: "); Serial.print(currentLeftDist); Serial.print("cm");
 Serial.print(" | C: "); Serial.print(currentCenterDist); Serial.print("cm"); // Printed center distance
 Serial.print(" | R: "); Serial.print(currentRightDist); Serial.print("cm");
 Serial.print(" | Turns: "); Serial.print(totalTurnsCount);


 if (hasTurnedOnce) {
   Serial.print(" | LOCK: "); Serial.print(lockedDirectionLeft ? "LEFT_ONLY" : "RIGHT_ONLY");
 } else {
   Serial.print(" | LOCK: NONE");
 }


 if (currentState == DRIVING_STRAIGHT) {
   Serial.print(" | Target: "); Serial.print(straightTargetHeading, 1);
   Serial.print("° | Current: "); Serial.print(currentHeading, 1);
   Serial.print("° | Error: "); Serial.print(headingError, 1);
   Serial.print("°");
 } else if (currentState == TURNING) {
   Serial.print(" | Target: "); Serial.print(turnTargetHeading, 1);
   Serial.print("° | Current: "); Serial.print(currentHeading, 1);
   Serial.print("° | Delta: "); Serial.print(abs(angleDifference), 1);
   Serial.print("°");
 }


 Serial.print(" | Servo Angle: "); Serial.print(finalServoAngle); Serial.println("°");
}


// ==========================================
//             CORE ARDUINO SETUP
// ==========================================
void setup() {
Serial.begin(115200);
Wire.begin(21, 22);


pinMode(MOTOR_IN1, OUTPUT); 
pinMode(MOTOR_IN2, OUTPUT); 
pinMode(MOTOR_PWM, OUTPUT);


steeringServo.attach(SERVO_PIN);
steeringServo.write(SERVO_CENTER);


selectMuxChannel(MUX_CH_BNO);
if (!bno.begin()) {
  Serial.println("Critical Error: BNO055 missing on Multiplexer channel 4!");
  while (1);
}
delay(500);
bno.setExtCrystalUse(true);


straightTargetHeading = getCurrentHeading();
}


// ==========================================
//             MAIN EXECUTION LOOP
// ==========================================
void loop() {
if (currentState == ROBOT_STOPPED) {
  setMotorOutput(0);
  steeringServo.write(SERVO_CENTER);
  Serial.print("STATUS: Finished. Total Turns Executed: ");
  Serial.println(totalTurnsCount);
  delay(1000);
  return;
}


// Read distances from all three TF-Luna Sensors
currentLeftDist   = getLunaDistance(MUX_CH_LEFT);
currentCenterDist = getLunaDistance(MUX_CH_CENTER);
currentRightDist  = getLunaDistance(MUX_CH_RIGHT);


float currentHeading = getCurrentHeading();


// FINAL STOP LOGIC: Triggered only after 12 turns are done and center wall is < 165cm
if (totalTurnsCount >= MAX_TURNS && currentCenterDist > 0 && currentCenterDist < 165) {
  currentState = ROBOT_STOPPED;
  return;
}


if (currentState == DRIVING_STRAIGHT) {
  checkObstacles();
  driveStraightMode(currentHeading);
}
else if (currentState == TURNING) {
  executeTurnMode(currentHeading);
}


printTelemetry(currentHeading);
delay(30);
}


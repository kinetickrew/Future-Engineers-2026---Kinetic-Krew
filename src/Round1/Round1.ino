#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "BluetoothSerial.h"
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
#define MUX_CH_BNO    5


#define TFLUNA_I2C_ADDR 0x10


// ==========================================
//          CONFIGURABLE VARIABLES
// ==========================================
 float SLOW_STEER_THRESHOLD = 50.0; // Angle (in degrees) where the servo starts returning to center
int STRAIGHT_SPEED = 150;  // Cruise speed for driving straight (0-255)
int TURN_SPEED     = 150;  // Controlled speed for turning to prevent overshooting
int BACKWARD_SPEED = -150; // Speed for backing up (if needed)


int SERVO_CENTER   = 90;   // Dead-center steering alignment
int DIFF = 35; 
int SERVO_MAX_LEFT = SERVO_CENTER + DIFF;  // Physical mechanical limit for left turn
int SERVO_MAX_RIGHT= SERVO_CENTER - DIFF;   // Physical mechanical limit for right turn

// Change to true if your steering corrections move backwards during testing
const bool INVERT_STEERING = true;


const float STEERING_KP  = 1.2;   // Proportional gain — how hard it steers based on current error
const float STEERING_KD  = 0.15 ;   // Derivative gain — how hard it "brakes" based on how fast error is changing
const float HEADING_DEADBAND = 1.5;   // degrees — ignore jitter smaller than this
const int   MAX_STEER_CORRECTION = 20; // clamp — no single correction can swing the servo too hard
const int SPIKE_THRESHOLD = 100;  // Trigger turning mode if sensor difference changes by this many cm
const int MAX_TURNS      = 12;   // Total number of turns allowed before tracking the final stop distance

unsigned long turnCooldownUntil = 0;   // NEW: timestamp until which obstacle checks are ignored
const unsigned long TURN_COOLDOWN_MS = 300; // tune this — how long to ignore after a turn

// Cardinal heading snap table — every commanded turn target is forced onto one of these,
// so sensor noise/drift can never leave the robot aiming at something like 250°.
const float CARDINAL_HEADINGS[4] = {0.0, 90.0, 180.0, 270.0};

// ==========================================
//          GLOBAL STATE VARIABLES
// ==========================================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Servo steeringServo;
BluetoothSerial SerialBT;    // Added Bluetooth object


enum RobotState { DRIVING_STRAIGHT, TURNING, ROBOT_STOPPED };
RobotState currentState = DRIVING_STRAIGHT;


float straightTargetHeading = 0.0;
float turnTargetHeading     = 0.0;
bool isTurningLeft          = false;


int totalTurnsCount         = 0
;
bool hasTurnedOnce          = false; // Becomes true permanently on the first turn
bool lockedDirectionLeft    = false; // Remembers if our layout is strictly Left or Right


// Global telemetry variables
int16_t currentLeftDist     = -1;
int16_t currentCenterDist   = -1; // Center LiDAR distance variable
int16_t currentRightDist    = -1;
int finalServoAngle         = 90;
float headingError          = 0.0;
float angleDifference       = 0.0;
float lastHeadingError       = 0.0;   // NEW: needed to calculate the D term
unsigned long lastHeadingTime = 0;    // NEW: for time-based derivative

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

float filteredHeading = 0.0;
bool headingInitialized = false;

float getSmoothedHeading() {
  float raw = getCurrentHeading();
  if (!headingInitialized) {
    filteredHeading = raw;
    headingInitialized = true;
    return filteredHeading;
  }

  // Find the shortest angular distance from filtered to raw (handles the 0/360 wrap)
  float diff = raw - filteredHeading;
  if (diff > 180.0)  diff -= 360.0;
  if (diff < -180.0) diff += 360.0;

  filteredHeading += 0.2 * diff;   // same 0.8/0.2 blend, but wrap-safe
  if (filteredHeading < 0.0)    filteredHeading += 360.0;
  if (filteredHeading >= 360.0) filteredHeading -= 360.0;

  return filteredHeading;
}

// ==========================================
//        CARDINAL HEADING SNAP HELPER
// ==========================================
// Rounds any angle to the nearest of 0/90/180/270, wrapping correctly around 0/360.
float snapToCardinal(float angle) {
  angle = fmod(angle, 360.0);
  if (angle < 0.0) angle += 360.0;

  float best = CARDINAL_HEADINGS[0];
  float bestDiff = 999.0;

  for (int i = 0; i < 4; i++) {
    float diff = fabs(angle - CARDINAL_HEADINGS[i]);
    if (diff > 180.0) diff = 360.0 - diff; // shortest wrap-around distance
    if (diff < bestDiff) {
      bestDiff = diff;
      best = CARDINAL_HEADINGS[i];
    }
  }
  return best;
}

// Computes a turn target: takes the raw current heading +/-90 depending on direction,
// then snaps it onto the nearest cardinal so the robot always ends up aimed at
// exactly 0, 90, 180, or 270 — never something drifted like 250.
float computeTurnTarget(float currentHeading, bool turningLeft) {
  float raw = turningLeft ? (currentHeading - 90.0) : (currentHeading + 90.0);
  raw = fmod(raw, 360.0);
  if (raw < 0.0) raw += 360.0;
  return snapToCardinal(raw);
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
//             BEHAVIOR LOGIC MODES
// ==========================================
void checkObstacles() {
  if (millis() < turnCooldownUntil) return;   // NEW: skip checking right after a turn
  if (currentLeftDist == -1 || currentRightDist == -1) return;


  int16_t difference = currentLeftDist - currentRightDist;


  // 1. HARD LOCK CHECK: If we already picked a direction, force evaluation down only one branch
  if (hasTurnedOnce) {
    if (lockedDirectionLeft) {
      if (difference > SPIKE_THRESHOLD) {
        float currentHeading = getCurrentHeading();
        isTurningLeft = true;
        turnTargetHeading = computeTurnTarget(currentHeading, true);
        currentState = TURNING;
      }
    } else {
      if (difference < -SPIKE_THRESHOLD) {
        float currentHeading = getCurrentHeading();
        isTurningLeft = false;
        turnTargetHeading = computeTurnTarget(currentHeading, false);
        currentState = TURNING;
      }
    }
    return;
  }


  // 2. FIRST TURN RUN
  if (difference > SPIKE_THRESHOLD) {
    float currentHeading = getCurrentHeading();
    isTurningLeft = true;
    turnTargetHeading = computeTurnTarget(currentHeading, true);
    hasTurnedOnce = true;
    lockedDirectionLeft = true;
   
    Serial.println("!!! LAYOUT INITIALIZED: PERMANENT LEFT TURN ONLY LOCK ACTIVATED !!!");
    SerialBT.println("!!! LAYOUT INITIALIZED: PERMANENT LEFT TURN ONLY LOCK ACTIVATED !!!");


    currentState = TURNING;
  }
  else if (difference < -SPIKE_THRESHOLD) {
    float currentHeading = getCurrentHeading();
    isTurningLeft = false;
    turnTargetHeading = computeTurnTarget(currentHeading, false);
    hasTurnedOnce = true;
    lockedDirectionLeft = false;
   
    Serial.println("!!! LAYOUT INITIALIZED: PERMANENT RIGHT TURN ONLY LOCK ACTIVATED !!!");
    SerialBT.println("!!! LAYOUT INITIALIZED: PERMANENT RIGHT TURN ONLY LOCK ACTIVATED !!!");


    currentState = TURNING;
  }
}


void driveStraightMode(float currentHeading) {
  setMotorOutput(STRAIGHT_SPEED);

  float rawError = straightTargetHeading - currentHeading;
  if (rawError > 180.0)  rawError -= 360.0;
  if (rawError < -180.0) rawError += 360.0;

  headingError = rawError; // keep raw value for telemetry + rate calc (no snapping to 0)

  // D term: real rate of change (degrees per second), computed from the RAW error
  // so it never sees a fake jump caused by the deadband
  unsigned long now = millis();
  float dt = (now - lastHeadingTime) / 1000.0;
  if (dt < 0.001) dt = 0.001;

  float errorRate = (rawError - lastHeadingError) / dt;

  // Deadband only affects the P term — small noise doesn't trigger a steering push,
  // but the D term still sees smooth, continuous data
  float pTermInput = (abs(rawError) < HEADING_DEADBAND) ? 0.0 : rawError;

  int steeringCorrection = (int)(pTermInput * STEERING_KP + errorRate * STEERING_KD);
  steeringCorrection = constrain(steeringCorrection, -MAX_STEER_CORRECTION, MAX_STEER_CORRECTION);

  lastHeadingError = rawError;
  lastHeadingTime = now;

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


  angleDifference = currentHeading - turnTargetHeading;
  if (angleDifference > 180.0)  angleDifference -= 360.0;
  if (angleDifference < -180.0) angleDifference += 360.0;
  float remainingAngle = abs(angleDifference);


  int leftExtreme  = INVERT_STEERING ? SERVO_MAX_RIGHT : SERVO_MAX_LEFT;
  int rightExtreme = INVERT_STEERING ? SERVO_MAX_LEFT  : SERVO_MAX_RIGHT;

  if (remainingAngle < SLOW_STEER_THRESHOLD) {
    float progressFactor = remainingAngle / SLOW_STEER_THRESHOLD;
    if (isTurningLeft) {
      int maxOffset = leftExtreme - SERVO_CENTER;
      finalServoAngle = SERVO_CENTER + (int)(maxOffset * progressFactor);
    } else {
      int maxOffset = SERVO_CENTER - rightExtreme;
      finalServoAngle = SERVO_CENTER - (int)(maxOffset * progressFactor);
    }
  } else {
    finalServoAngle = isTurningLeft ? leftExtreme : rightExtreme;
  }


  steeringServo.write(finalServoAngle);


  if (remainingAngle < 6.0) {
  totalTurnsCount++;
  steeringServo.write(SERVO_CENTER);
  finalServoAngle = SERVO_CENTER;
  straightTargetHeading = turnTargetHeading;
  currentState = DRIVING_STRAIGHT;
  turnCooldownUntil = millis() + TURN_COOLDOWN_MS;   // NEW
  }
}


// ==========================================
//      BLUETOOTH DUAL-PRINT HELPERS
// ==========================================
template <typename T>
void btPrint(T msg) {
  Serial.print(msg);
  SerialBT.print(msg);
}


template <typename T>
void btPrintln(T msg) {
  Serial.println(msg);
  SerialBT.println(msg);
}


void printTelemetry(float currentHeading) {
  if (currentState == DRIVING_STRAIGHT) btPrint("MODE: STRAIGHT");
  else if (currentState == TURNING)     btPrint("MODE: TURNING ");


  btPrint(" | L: "); btPrint(currentLeftDist); btPrint("cm");
  btPrint(" | C: "); btPrint(currentCenterDist); btPrint("cm");
  btPrint(" | R: "); btPrint(currentRightDist); btPrint("cm");
  btPrint(" | Turns: "); btPrint(totalTurnsCount);


  if (hasTurnedOnce) {
    btPrint(" | LOCK: "); btPrint(lockedDirectionLeft ? "LEFT_ONLY" : "RIGHT_ONLY");
  } else {
    btPrint(" | LOCK: NONE");
  }


  if (currentState == DRIVING_STRAIGHT) {
    btPrint(" | Target: "); btPrint(straightTargetHeading);
    btPrint("° | Current: "); btPrint(currentHeading);
    btPrint("° | Error: "); btPrint(headingError);
    btPrint("°");
  } else if (currentState == TURNING) {
    btPrint(" | Target: "); btPrint(turnTargetHeading);
    btPrint("° | Current: "); btPrint(currentHeading);
    btPrint("° | Delta: "); btPrint(abs(angleDifference));
    btPrint("°");
  }


  btPrint(" | Servo Angle: "); btPrint(finalServoAngle); btPrintln("°");
}


// ==========================================
//             CORE ARDUINO SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  SerialBT.begin("ESP32_Robot_Telemetry");

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);


  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_CENTER);


  selectMuxChannel(MUX_CH_BNO);
  if (!bno.begin()) {
    Serial.println("Critical Error: BNO055 missing on Multiplexer channel 4!");
    SerialBT.println("Critical Error: BNO055 missing on Multiplexer channel 4!");
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
    SerialBT.print("STATUS: Finished. Total Turns Executed: ");
    SerialBT.println(totalTurnsCount);
   
    delay(1000);
    return;
  }


  currentLeftDist   = getLunaDistance(MUX_CH_LEFT);
  currentCenterDist = getLunaDistance(MUX_CH_CENTER);
  currentRightDist  = getLunaDistance(MUX_CH_RIGHT);


  float currentHeading = getSmoothedHeading();


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
  delay(20);
}



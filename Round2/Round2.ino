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
BluetoothSerial SerialBT;    // Added Bluetooth object


enum RobotState { DRIVING_STRAIGHT, TURNING, ROBOT_STOPPED, PI_OVERRIDE };
RobotState currentState = DRIVING_STRAIGHT;

// ==========================================
//      PI (RASPBERRY PI) COMMS STATE
// ==========================================
// Pi sends commands over the same USB Serial used for programming/debug.
// Format: "D,<servo_angle>,<motor_speed>\n" to take control (dodge),
//         "R\n" to hand control back to autonomous driving.
unsigned long lastPiCommandMillis = 0;
const unsigned long PI_COMMAND_TIMEOUT_MS = 300; // Failsafe: resume autonomous if Pi goes silent
String lastCameraStatus = "NO DETECTION"; // Updated by D/R commands, shown every telemetry line


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
//        RASPBERRY PI COMMAND HANDLING
// ==========================================
void checkForPiCommands() {
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    if (line.startsWith("D,")) {
      // Format: D,<color>,<servo_angle>,<motor_speed>  e.g. "D,RED,60,180"
      int c1 = line.indexOf(',');
      int c2 = line.indexOf(',', c1 + 1);
      int c3 = line.indexOf(',', c2 + 1);
      if (c2 == -1 || c3 == -1) continue; // malformed, ignore this line

      String colorStr = line.substring(c1 + 1, c2);
      int servoAngle  = line.substring(c2 + 1, c3).toInt();
      int motorSpeed  = line.substring(c3 + 1).toInt();

      // Always constrain to physical/safe limits, even though the Pi
      // should already be sending safe values -- never trust the wire.
      servoAngle = constrain(servoAngle, SERVO_MAX_RIGHT, SERVO_MAX_LEFT);
      motorSpeed = constrain(motorSpeed, -255, 255);

      steeringServo.write(servoAngle);
      setMotorOutput(motorSpeed);
      finalServoAngle = servoAngle;

      lastCameraStatus = colorStr + " DETECTED";
      currentState = PI_OVERRIDE;
      lastPiCommandMillis = millis();
    }
    else if (line == "R") {
      // Pi is handing control back -- re-anchor heading so the robot
      // doesn't try to correct back to whatever direction it was facing
      // before the dodge.
      lastCameraStatus = "NO DETECTION";
      currentState = DRIVING_STRAIGHT;
      straightTargetHeading = getCurrentHeading();
    }
  }
}


// ==========================================
//             BEHAVIOR LOGIC MODES
// ==========================================
void checkObstacles() {
  if (currentLeftDist == -1 || currentRightDist == -1) return;


  int16_t difference = currentLeftDist - currentRightDist;


  // 1. HARD LOCK CHECK: If we already picked a direction, force evaluation down only one branch
  if (hasTurnedOnce) {
    if (lockedDirectionLeft) {
      if (difference > SPIKE_THRESHOLD) {
        float currentHeading = getCurrentHeading();
        isTurningLeft = true;
        turnTargetHeading = currentHeading - 90.0;
        if (turnTargetHeading < 0.0) turnTargetHeading += 360.0;
        currentState = TURNING;
      }
    } else {
      if (difference < -SPIKE_THRESHOLD) {
        float currentHeading = getCurrentHeading();
        isTurningLeft = false;
        turnTargetHeading = currentHeading + 90.0;
        if (turnTargetHeading >= 360.0) turnTargetHeading -= 360.0;
        currentState = TURNING;
      }
    }
    return;
  }


  // 2. FIRST TURN RUN
  if (difference > SPIKE_THRESHOLD) {
    float currentHeading = getCurrentHeading();
    isTurningLeft = true;
    turnTargetHeading = currentHeading - 90.0;
    if (turnTargetHeading < 0.0) turnTargetHeading += 360.0;
    hasTurnedOnce = true;
    lockedDirectionLeft = true;
   
    SerialBT.println("!!! LAYOUT INITIALIZED: PERMANENT LEFT TURN ONLY LOCK ACTIVATED !!!");


    currentState = TURNING;
  }
  else if (difference < -SPIKE_THRESHOLD) {
    float currentHeading = getCurrentHeading();
    isTurningLeft = false;
    turnTargetHeading = currentHeading + 90.0;
    if (turnTargetHeading >= 360.0) turnTargetHeading -= 360.0;
    hasTurnedOnce = true;
    lockedDirectionLeft = false;
   
    SerialBT.println("!!! LAYOUT INITIALIZED: PERMANENT RIGHT TURN ONLY LOCK ACTIVATED !!!");


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


  angleDifference = currentHeading - turnTargetHeading;
  if (angleDifference > 180.0)  angleDifference -= 360.0;
  if (angleDifference < -180.0) angleDifference += 360.0;
  float remainingAngle = abs(angleDifference);


  if (remainingAngle < SLOW_STEER_THRESHOLD) {
    float progressFactor = remainingAngle / SLOW_STEER_THRESHOLD;
    if (isTurningLeft) {
      int maxOffset = SERVO_MAX_LEFT - SERVO_CENTER;
      finalServoAngle = SERVO_CENTER + (int)(maxOffset * progressFactor);
    } else {
      int maxOffset = SERVO_CENTER - SERVO_MAX_RIGHT;
      finalServoAngle = SERVO_CENTER - (int)(maxOffset * progressFactor);
    }
  } else {
    if (isTurningLeft) {
      finalServoAngle = SERVO_MAX_LEFT;
    } else {
      finalServoAngle = SERVO_MAX_RIGHT;
    }
  }


  steeringServo.write(finalServoAngle);


  if (remainingAngle < 6.0) {
    totalTurnsCount++;
    steeringServo.write(SERVO_CENTER);
    finalServoAngle = SERVO_CENTER;
    straightTargetHeading = turnTargetHeading;
    currentState = DRIVING_STRAIGHT;
    delay(150);
  }
}


// ==========================================
//      BLUETOOTH DUAL-PRINT HELPERS
// ==========================================
template <typename T> void btPrint(T msg) {
  // NOTE: Serial (USB) is now reserved exclusively for Pi command traffic.
  // Telemetry goes out over Bluetooth only so the two channels never collide.
  SerialBT.print(msg);
}


template <typename T> void btPrintln(T msg) {
  SerialBT.println(msg);
}


void printTelemetry(float currentHeading) {
  if (currentState == DRIVING_STRAIGHT) btPrint("MODE: STRAIGHT");
  else if (currentState == TURNING)     btPrint("MODE: TURNING ");
  else if (currentState == PI_OVERRIDE) btPrint("MODE: DODGING ");


  btPrint(" | L: "); btPrint(currentLeftDist); btPrint("cm");
  btPrint(" | C: "); btPrint(currentCenterDist); btPrint("cm");
  btPrint(" | R: "); btPrint(currentRightDist); btPrint("cm");
  btPrint(" | Turns: "); btPrint(totalTurnsCount);
  btPrint(" | CAM: "); btPrint(lastCameraStatus);


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
  } else if (currentState == PI_OVERRIDE) {
    btPrint(" | Current: "); btPrint(currentHeading); btPrint("°");
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
  checkForPiCommands();

  if (currentState == PI_OVERRIDE) {
    // Failsafe: if the Pi stops sending commands (crash, cable issue,
    // lost detection) fall back to autonomous driving instead of coasting
    // on a stale command forever.
    if (millis() - lastPiCommandMillis > PI_COMMAND_TIMEOUT_MS) {
      currentState = DRIVING_STRAIGHT;
      straightTargetHeading = getCurrentHeading();
    } else {
      // Motor/servo were already set directly in checkForPiCommands().
      printTelemetry(getCurrentHeading());
      delay(30);
      return;
    }
  }

  if (currentState == ROBOT_STOPPED) {
    setMotorOutput(0);
    steeringServo.write(SERVO_CENTER);
   
    SerialBT.print("STATUS: Finished. Total Turns Executed: ");
    SerialBT.println(totalTurnsCount);
   
    delay(1000);
    return;
  }


  currentLeftDist   = getLunaDistance(MUX_CH_LEFT);
  currentCenterDist = getLunaDistance(MUX_CH_CENTER);
  currentRightDist  = getLunaDistance(MUX_CH_RIGHT);


  float currentHeading = getCurrentHeading();


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

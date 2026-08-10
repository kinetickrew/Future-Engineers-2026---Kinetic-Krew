#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ESP32Servo.h>
#include "BluetoothSerial.h"

// ==========================================
//           PIN & HARDWARE DEFINITIONS
// ==========================================
#define I2C_SDA 21
#define PIN_I2C_SCL 22
#define Servo_Pin 13
#define PCA_I2C_ADDR 0x70
#define TFLUNA_I2C_ADDR 0x10
#define BNO_I2C_ADDR 0x28

// I2C Multiplexer Channels (HW-617 / PCA9548A)
#define Mux_Ch_Left 0
#define Mux_Ch_Center 1
#define Mux_Ch_Right 2
#define Mux_Ch_BNO 4

// Drive motor pins (N20 via TB6612, channel A)
#define Motor_In1 25
#define Motor_In2 26
#define Motor_PWM 33

// ==========================================
//          CONFIGURABLE VARIABLES
// ==========================================

// NOTE: these were originally #define constants. They're now plain variables
// so the Bluetooth tuning console below can change them at runtime, exactly
// like the WiFi-tuner build does for its own tunables. Nothing else about
// the control logic below has changed.

float Diff = 50;
float Servo_Center = 90;
float Servo_Max_Left  = Servo_Center - Diff;
float Servo_Max_Right = Servo_Center + Diff;
float Kp       = 1;     // servo deg per deg of heading error
int   Spike_Threshold    = 150;   // side reading above this = corner/opening
float TurnAngle       = 90;    // heading change per detected opening
float Heading_Deadband  = 2.0;
int   Straight_Speed    = 255;   // cruise duty 0..255 — tune on the mat
int   Start_Delay      = 1000;  // hands-off pause inside startDrive()
int   Max_Turns      = 12;    // 3 laps x 4 corners
int   Stop_Condition        = 500;   // drive this long AFTER the last turn completes
float RA    = 15;    // heading within this many deg of target = last turn considered complete
unsigned long Calibrate_Max = 4000; // failsafe: start the run-on anyway if heading never settles

// Turn cooldown — after a turn is registered, no NEW turn can be detected for this long.
// Independent of the LiDAR mute window below — change freely.
unsigned long TURN_COOLDOWN_MS = 1000;

// ---- Fixed / non-tunable settings (left as #define, matches original) ----
#define Servo_Min  500    
#define Servo_Max  2400  
#define Code_Loop_Delay 20
#define Motor_Max_PWM 255
#define Motor_Dir false  
#define Start_Button 32

// ==========================================
//          GLOBAL STATE VARIABLES
// ==========================================
Adafruit_BNO055 imuSensor = Adafruit_BNO055(55, BNO_I2C_ADDR, &Wire);
Servo steerServo;
BluetoothSerial SerialBT;
ESP32PWM motorPwm; 

float headingTarget = 0;   // the heading we steer to hold
String currentMode = "STRAIGHT";      // STRAIGHT or TURNING, for telemetry
bool hasTurnedOnce = false;           // becomes true permanently on the first turn
bool lockedDirectionLeft = false;     // true = locked to left turns only, false = right only
int turnsCompleted = 0;
float headingError = 0.0;

// ==========================================
//        BLUETOOTH DUAL-PRINT HELPERS
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

// ==========================================
//            I2C & SENSOR FUNCTIONS
// ==========================================
void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(PCA_I2C_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

float wrapAngle180(float a) {
  while (a >  180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

// TF-Luna distance (cm), -1 if no response
int getLunaDistance(uint8_t channel) {
  selectMuxChannel(channel);
  delayMicroseconds(50);
  Wire.beginTransmission(TFLUNA_I2C_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return -1;
  if (Wire.requestFrom((uint8_t)TFLUNA_I2C_ADDR, (uint8_t)2) != 2) return -1;
  uint8_t distLoByte = Wire.read();
  uint8_t distHiByte = Wire.read();
  return distLoByte | (distHiByte << 8);
}

int distLeft()   { return getLunaDistance(Mux_Ch_Left);   }
int distCenter() { return getLunaDistance(Mux_Ch_Center); }
int distRight()  { return getLunaDistance(Mux_Ch_Right);  }

float getCurrentHeading() {
  selectMuxChannel(Mux_Ch_BNO);
  delayMicroseconds(50);
  imu::Vector<3> eulerVec = imuSensor.getVector(Adafruit_BNO055::VECTOR_EULER);
  return eulerVec.x();                     // yaw / heading, 0-360
}

void captureHeadingReference() {
  headingTarget = getCurrentHeading();    // current direction = heading to hold
}

// ==========================================
//            ACTUATOR FUNCTIONS
// ==========================================
// Steering: proportional correction toward headingTarget
int steerToHeading(float currentHeading, float target) {
  headingError = wrapAngle180(currentHeading - target);
  if (fabsf(headingError) < Heading_Deadband) headingError = 0;
  float steerAngle = Servo_Center - Kp * headingError;
  steerAngle = constrain(steerAngle, Servo_Max_Left, Servo_Max_Right);
  steerServo.write((int)steerAngle);
  return (int)steerAngle;
}

void motorBrake() {
  digitalWrite(Motor_In1, HIGH);
  digitalWrite(Motor_In2, HIGH);
  motorPwm.write(0);
}

void motorCoast() {
  digitalWrite(Motor_In1, LOW);
  digitalWrite(Motor_In2, LOW);
  motorPwm.write(Motor_Max_PWM);
}

void setMotorSpeed(int speedSigned) {   // -255..255, sign = direction, 0 = brake
  if (Motor_Dir) speedSigned = -speedSigned;
  speedSigned = constrain(speedSigned, -Motor_Max_PWM, Motor_Max_PWM);
  if (speedSigned == 0) { motorBrake(); return; }
  digitalWrite(Motor_In1, speedSigned > 0 ? HIGH : LOW);
  digitalWrite(Motor_In2, speedSigned > 0 ? LOW  : HIGH);
  motorPwm.write(abs(speedSigned));
}

void startDrive() {
  delay(Start_Delay);              // hands-off pause
  setMotorSpeed(Straight_Speed);
}

// ==========================================
//             INIT FUNCTIONS
// ==========================================
void initImu() {
  Wire.begin(I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  selectMuxChannel(Mux_Ch_BNO);
  delayMicroseconds(50);
  if (!imuSensor.begin()) {               // OPERATION_MODE_IMUPLUS for motor-heavy robots
    btPrintln("BNO055 not found! Check ch4 wiring / address.");
    while (1) delay(10);
  }
  delay(1000);
  imuSensor.setExtCrystalUse(true);
}

void initSteerServo() {
  steerServo.setPeriodHertz(50);
  steerServo.attach(Servo_Pin, Servo_Min, Servo_Max);
}

void initDriveMotor() {
  pinMode(Motor_In1, OUTPUT);
  pinMode(Motor_In2, OUTPUT);
  motorPwm.attachPin(Motor_PWM, 20000, 8);   // 20kHz, 8-bit to match Motor_Max_PWM=255
  motorPwm.write(0);   // known-safe state before driving

  btPrintln("[motor] initialized on ESP32PWM (shared LEDC w/ servo)");
}

// Rule 9.11 wait-for-start
// Wiring assumption: button to GND, active LOW with internal pullup.
void waitForStartButton() {
  pinMode(Start_Button, INPUT_PULLUP);
  btPrintln("[start] waiting for start button (GPIO32 external)...");
  while (digitalRead(Start_Button) == HIGH) delay(10);   // wait for press
  delay(50);                                                 // debounce
  while (digitalRead(Start_Button) == LOW)  delay(10);   // wait for release
  btPrintln("[start] go - capturing heading reference");
}

// ==========================================
//        BLUETOOTH TUNING COMMAND PARSER
// ==========================================
// Accepts lines like: KP=1.4
// Runtime-only — nothing is written to flash.
void handleBluetoothCommands() {
  static String btBuffer = "";

  while (SerialBT.available()) {
    char c = SerialBT.read();

    if (c == '\n' || c == '\r') {
      btBuffer.trim();
      if (btBuffer.length() > 0) {
        int eqIndex = btBuffer.indexOf('=');
        if (eqIndex > 0) {
          String name  = btBuffer.substring(0, eqIndex);
          String value = btBuffer.substring(eqIndex + 1);
          name.trim();
          value.trim();
          name.toUpperCase();

          float fVal = value.toFloat();
          bool recognized = true;

          if      (name == "L")    Servo_Max_Left     = fVal;
          else if (name == "R")    Servo_Max_Right     = fVal;
          else if (name == "C") Servo_Center  = fVal;
          else if (name == "P")          Kp       = fVal;
          else if (name == "S")         Spike_Threshold    = (int)fVal;
          else if (name == "TA")    TurnAngle       = fVal;
          else if (name == "D")    Heading_Deadband  = fVal;
          else if (name == "SS")       Straight_Speed    = (int)fVal;
          else if (name == "SD")  Start_Delay      = (int)fVal;
          else if (name == "MT")    Max_Turns      = (int)fVal;
          else if (name == "SC")    Stop_Condition        = (int)fVal;
          else if (name == "RA")   RA    = fVal;
          else if (name == "CM") Calibrate_Max = (unsigned long)fVal;
          else if (name == "TC")    TURN_COOLDOWN_MS    = (unsigned long)fVal;
          else recognized = false;

          if (recognized) {
            SerialBT.print("OK: "); SerialBT.print(name);
            SerialBT.print(" set to "); SerialBT.println(value);
          } else {
            SerialBT.print("ERR: unknown variable '"); SerialBT.print(name); SerialBT.println("'");
          }
        } else {
          SerialBT.println("ERR: expected format NAME=VALUE");
        }
      }
      btBuffer = "";
    } else {
      btBuffer += c;
    }
  }
}

// ==========================================
//             CORE ARDUINO SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Robot_Telemetry");
  delay(300);
  initSteerServo();
  delay(300);
  initImu();
  initDriveMotor();
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);  // LED on to show setup complete
  waitForStartButton();       // rule 9.11 no-touch start - blocks here until the button
  captureHeadingReference();   // heading reference captured AT the start press
  btPrintln("\nReady: driving straight, turning at openings.\n");
  digitalWrite(2, LOW);
  startDrive();
}

// ==========================================
//             MAIN EXECUTION LOOP
// ==========================================
void loop() {
  handleBluetoothCommands();   // service the Bluetooth tuning console every loop
  static unsigned long turnCooldownUntilMs  = 0;   // no NEW turn allowed until this millis()
  static bool leftGapWasOpen  = false;
  static bool rightGapWasOpen = false;

  float currentHeading = getCurrentHeading();                      // always read heading, always steer

  int distL = -1, distC = -1, distR = -1;
  bool inTurnCooldown = (millis() < turnCooldownUntilMs);

  distL = distLeft();
  distC = distCenter();
  distR = distRight();

  bool leftGapOpen  = (distL > Spike_Threshold);
  bool rightGapOpen = (distR > Spike_Threshold);

  bool didTurn = false;
  if (!inTurnCooldown) {
    if (!hasTurnedOnce) {
      // First turn ever — whichever side opens first sets the permanent lock
      if (leftGapOpen && !leftGapWasOpen)   { headingTarget -= TurnAngle; didTurn = true; hasTurnedOnce = true; lockedDirectionLeft = true;  }
      if (rightGapOpen && !rightGapWasOpen) { headingTarget += TurnAngle; didTurn = true; hasTurnedOnce = true; lockedDirectionLeft = false; }
    } else if (lockedDirectionLeft) {
      // Locked left — only ever react to the left gap from now on
      if (leftGapOpen && !leftGapWasOpen)   { headingTarget -= TurnAngle; didTurn = true; }
    } else {
      // Locked right — only ever react to the right gap from now on
      if (rightGapOpen && !rightGapWasOpen) { headingTarget += TurnAngle; didTurn = true; }
    }

    leftGapWasOpen  = leftGapOpen;
    rightGapWasOpen = rightGapOpen;

    if (didTurn) {
      turnCooldownUntilMs = millis() + TURN_COOLDOWN_MS;
      currentMode = "TURNING";
    }
  }

  int servoAngleOut = steerToHeading(currentHeading, headingTarget);

  if (currentMode == "TURNING" && fabsf(wrapAngle180(currentHeading - headingTarget)) < RA) {
    currentMode = "STRAIGHT";
  }

  btPrint("Mode: "); btPrint(currentMode);
  btPrint(" | Left: "); btPrint(distL);
  btPrint(" | Center: "); btPrint(distC);
  btPrint(" | Right: "); btPrint(distR);
  if (currentMode == "STRAIGHT") {
    btPrint(" | Target: "); btPrint(headingTarget);
    btPrint(" | Current: "); btPrint(currentHeading);
    btPrint(" | Error: "); btPrint(headingError);
  } else {
    btPrint(" | Target: "); btPrint(headingTarget);
    btPrint(" | Current: "); btPrint(currentHeading);
    btPrint(" | Delta: "); btPrint(fabsf(headingTarget - currentHeading));
  }
  btPrint(" | Servo: "); btPrint(servoAngleOut);
  btPrint(" | Turns: "); btPrint(turnsCompleted);
  if (!hasTurnedOnce) {
    btPrint(" | Lock: NONE");
  } else {
    btPrint(" | Lock: "); btPrint(lockedDirectionLeft ? "LEFT_ONLY" : "RIGHT_ONLY");
  }
  if (inTurnCooldown) btPrint(" | Turn Cooldown");
  btPrintln("");

  trackTurnsAndStop();
  delay(Code_Loop_Delay);
}

void trackTurnsAndStop() {
  static float prevHeadingTarget = NAN;             
  static unsigned long finalStopAtMs = 0;
  static unsigned long lastTurnAtMs = 0;

  if (isnan(prevHeadingTarget)) prevHeadingTarget = headingTarget;

  if (headingTarget != prevHeadingTarget) {           // loop() changed the target -> corner(s) taken
    turnsCompleted += (int)(fabsf(headingTarget - prevHeadingTarget) / TurnAngle + 0.5f);
    prevHeadingTarget = headingTarget;
    btPrint("[turn] "); btPrint(turnsCompleted); btPrint("/"); btPrintln(Max_Turns);
  }

  if (turnsCompleted >= Max_Turns && finalStopAtMs == 0) {
    if (lastTurnAtMs == 0) lastTurnAtMs = millis();
    float settleErr = fabsf(wrapAngle180(getCurrentHeading() - headingTarget));   // extra IMU read, this phase only
    if (settleErr < RA || millis() - lastTurnAtMs > Calibrate_Max) {
      finalStopAtMs = millis() + Stop_Condition;                        // last turn done -> timed run-on starts NOW
      btPrintln("[turn] last turn settled - final run-on");
    }
  }
  if (finalStopAtMs != 0 && millis() >= finalStopAtMs) { // steering stayed live through the run-on
  motorBrake();
  digitalWrite(2, HIGH);   // blue LED on — all rounds complete
  btPrintln("Run complete - motor braked.");
  while (1) delay(100);
  }
}

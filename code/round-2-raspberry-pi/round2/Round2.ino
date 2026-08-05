#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>

// ==========================================
//   THIS FIRMWARE DOES NO THINKING.
//   It reads the three LiDARs + BNO heading and reports them every loop
//   as "T,<left>,<center>,<right>,<heading>\n" over USB Serial, and obeys
//   whatever drive command the Pi sends back:
//     "M,<servo_angle>,<motor_speed>\n"  -- set servo + motor directly
//     "S\n"                              -- stop
//   All decisions (when to turn, how far, when to dodge, when to stop)
//   are made on the Pi. This board just senses and actuates.
// ==========================================

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
#define MUX_CH_CENTER 1
#define MUX_CH_RIGHT  2
#define MUX_CH_BNO    4

#define TFLUNA_I2C_ADDR 0x10

// ==========================================
//          SERVO / MOTOR SAFETY LIMITS
//      (the Pi should already send safe values -- never trust the wire)
// ==========================================
const int SERVO_CENTER    = 90;
const int SERVO_MIN       = 45;   // physical mechanical limit for right turn
const int SERVO_MAX       = 135;  // physical mechanical limit for left turn

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Servo steeringServo;

int16_t currentLeftDist   = -1;
int16_t currentCenterDist = -1;
int16_t currentRightDist  = -1;

unsigned long lastCommandMillis = 0;
const unsigned long COMMAND_TIMEOUT_MS = 500; // Failsafe: stop if the Pi goes silent

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
    speed = -speed;
  }
  analogWrite(MOTOR_PWM, constrain(speed, 0, 255));
}

void stopEverything() {
  setMotorOutput(0);
  steeringServo.write(SERVO_CENTER);
}

// ==========================================
//        PI COMMAND HANDLING
// ==========================================
void checkForPiCommands() {
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    if (line.startsWith("M,")) {
      // Format: M,<servo_angle>,<motor_speed>
      int c1 = line.indexOf(',');
      int c2 = line.indexOf(',', c1 + 1);
      if (c2 == -1) continue; // malformed, ignore this line

      int servoAngle = line.substring(c1 + 1, c2).toInt();
      int motorSpeed = line.substring(c2 + 1).toInt();

      servoAngle = constrain(servoAngle, SERVO_MIN, SERVO_MAX);
      motorSpeed = constrain(motorSpeed, -255, 255);

      steeringServo.write(servoAngle);
      setMotorOutput(motorSpeed);
      lastCommandMillis = millis();
    }
    else if (line == "S") {
      stopEverything();
      lastCommandMillis = millis();
    }
  }
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

  lastCommandMillis = millis();
}

// ==========================================
//             MAIN EXECUTION LOOP
// ==========================================
void loop() {
  checkForPiCommands();

  // Failsafe: if the Pi stops sending commands (crash, cable issue, USB
  // hiccup) stop the motor instead of coasting on the last command forever.
  if (millis() - lastCommandMillis > COMMAND_TIMEOUT_MS) {
    stopEverything();
  }

  currentLeftDist   = getLunaDistance(MUX_CH_LEFT);
  currentCenterDist = getLunaDistance(MUX_CH_CENTER);
  currentRightDist  = getLunaDistance(MUX_CH_RIGHT);
  float currentHeading = getCurrentHeading();

  Serial.print("T,");
  Serial.print(currentLeftDist);
  Serial.print(",");
  Serial.print(currentCenterDist);
  Serial.print(",");
  Serial.print(currentRightDist);
  Serial.print(",");
  Serial.println(currentHeading);

  delay(20);
}
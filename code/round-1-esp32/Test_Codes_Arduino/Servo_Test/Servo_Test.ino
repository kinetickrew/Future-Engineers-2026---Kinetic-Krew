#include <ESP32Servo.h>


#define SERVO_PIN 13


Servo testServo;


void setup() {
 Serial.begin(115200);
  // Attach the servo to pin 13
 testServo.attach(SERVO_PIN);
  // Start the servo at your dead-center position (90 degrees)
 testServo.write(90);
  Serial.println("=========================================");
 Serial.println("      ESP32 SERVO ANGLE TESTER           ");
 Serial.println("=========================================");
 Serial.println("Enter an angle between 45 and 135:");
}


void loop() {
 // Check if there is data waiting in the Serial buffer
 if (Serial.available() > 0) {
  
   // Read the incoming string until the user hits Enter
   String inputString = Serial.readStringUntil('\n');
  
   // Trim any accidental whitespaces or newline characters
   inputString.trim();
  
   // Convert the string to an integer
   int targetAngle = inputString.toInt();
  
   // Validate if the input is within your precise safe limits
   if (targetAngle >= 0 && targetAngle <= 180) {
     Serial.print("Moving to valid angle: ");
     Serial.print(targetAngle);
     Serial.println("°");
    
     testServo.write(targetAngle);
   }
   else {
     // Reject any values out of bounds (like 0, 180, or random letters)
     Serial.print("REJECTED: ");
     Serial.print(inputString);
     Serial.println(" is out of bounds! Please enter an angle strictly between 45 and 135.");
   }
 }
}


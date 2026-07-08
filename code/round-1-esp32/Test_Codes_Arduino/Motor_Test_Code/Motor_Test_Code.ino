// Motor Pin Definitions
const int IN1 = 25;
const int IN2 = 26;
const int PWM_PIN = 33;


// Timing variable
unsigned long lastUpdate = 0;
int currentSpeed = 255;
int stepAmount = -50; // Decrease by 50 each second


void setup() {
  // Initialize motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
    
  
  Serial.begin(115200);
  Serial.println("Motor Test Initialized. Starting at 255...");
}


void loop() {
  // Check if 1 second (1000 milliseconds) has passed
  if (millis() - lastUpdate >= 1000) {
    lastUpdate = millis();


    // Print current status to Serial Monitor
    Serial.print("Target Speed: ");
    Serial.println(currentSpeed);


    // Set motor direction and speed
    setMotor(currentSpeed);


    // Update speed for the next second
    currentSpeed += stepAmount;


    // Bounce back logic: if it goes past -255 or +255, reverse the direction of the step
    if (currentSpeed <= -255) {
      currentSpeed = -255;
      stepAmount = 50; // Start increasing
    } else if (currentSpeed >= 255) {
      currentSpeed = 255;
      stepAmount = -50; // Start decreasing
    }
  }
}


// Helper function to drive the motor
void setMotor(int speed) {
  // Absolute value for PWM duty cycle
  int pwmValue = abs(speed); 


  if (speed > 0) {
    // Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(PWM_PIN, pwmValue);
  } else if (speed < 0) {
    // Reverse
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(PWM_PIN, pwmValue);
  } else {
    // Brake/Stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(PWM_PIN, 0);
  }
}




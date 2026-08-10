#include <Wire.h>

// =========================================================================
// HW-617 (TCA9548A) CHANNEL CONFIGURATION
// Change these channel numbers if you plug the sensors into different slots.
// =========================================================================
const uint8_t LUNA_CH_1 = 0;  // Tied to SD0 / SC0
const uint8_t LUNA_CH_2 = 1;  // Tied to SD1 / SC1
const uint8_t LUNA_CH_3 = 2;  // Tied to SD2 / SC2

// =========================================================================
// FIXED CONSTANTS
// =========================================================================
const uint8_t MUX_ADDR = 0x70;   // Default factory I2C address of HW-617 MUX
const uint8_t LUNA_ADDR = 0x10;  // Predefined I2C address for all TF-Lunas
const int BAUDRATE = 115200;     // Serial communication speed
const int DATA_LENGTH = 9;       // TF-Luna data packet size

void setup() {
  Serial.begin(BAUDRATE);
  Wire.begin(); // Initialize microcontroller I2C bus
  
  Serial.println("HW-617 I2C Multiplexer + Multi-TF-Luna Initialized.");
  Serial.println("--------------------------------------------------\n");
}

void loop() {
  // Read Distance from Sensor 1 (Channel 0)
  tcaSelect(LUNA_CH_1);
  int dist1 = getLunaDistance();
  
  // Read Distance from Sensor 2 (Channel 1)
  tcaSelect(LUNA_CH_2);
  int dist2 = getLunaDistance();
  
  // Read Distance from Sensor 3 (Channel 2)
  tcaSelect(LUNA_CH_3);
  int dist3 = getLunaDistance();

  // Print formatted data to Serial Monitor
  printData(1, dist1);
  Serial.print(" | ");
  printData(2, dist2);
  Serial.print(" | ");
  printData(3, dist3);
  Serial.println();

  delay(50); // Small interval loop delay
}

// =========================================================================
// HELPER FUNCTION: Switch active channel on the HW-617 Multiplexer
// =========================================================================
void tcaSelect(uint8_t channel) {
  if (channel > 7) return; // HW-617 only has channels 0-7
  
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel); // Bitwise shift to open the specific channel gate
  Wire.endTransmission();
}

// =========================================================================
// HELPER FUNCTION: Read Distance from the currently active channel
// =========================================================================
int getLunaDistance() {
  // Command packet to trigger a data frame from TF-Luna
  unsigned char triggerCmd[] = {0x5A, 0x05, 0x00, 0x01, 0x60};
  
  Wire.beginTransmission(LUNA_ADDR);
  Wire.write(triggerCmd, 5);
  if (Wire.endTransmission() != 0) {
    return -1; // Connection error on this channel
  }
  
  Wire.requestFrom(LUNA_ADDR, (uint8_t)DATA_LENGTH);
  
  uint8_t data[DATA_LENGTH] = {0};
  int index = 0;
  
  while (Wire.available() > 0 && index < DATA_LENGTH) {
    data[index++] = Wire.read();
  }
  
  // Verify TF-Luna data header frame (0x59 0x59)
  if (index == DATA_LENGTH && data[0] == 0x59 && data[1] == 0x59) {
    int distance = data[2] + (data[3] << 8); // Combine Low and High bytes
    return distance; // Units in cm
  }
  
  return -2; // Data processing error
}

// Format printing utility
void printData(int id, int distance) {
  Serial.print("Lidar ");
  Serial.print(id);
  Serial.print(": ");
  if (distance > 0) {
    Serial.print(distance);
    Serial.print(" cm");
  } else if (distance == -1) {
    Serial.print("CONN_ERR");
  } else {
    Serial.print("DATA_ERR");
  }
}


#include <Wire.h>

// =========================================================================
// CONFIGURABLE SETTINGS (Change these as needed)
// =========================================================================
const uint8_t LUNA_CH_1 = 0;    // HW-617 channel for Sensor 1 (SD0/SC0)
const uint8_t LUNA_CH_2 = 1;    // HW-617 channel for Sensor 2 (SD1/SC1)
const uint8_t LUNA_CH_3 = 2;    // HW-617 channel for Sensor 3 (SD2/SC2)

const int NUM_SAMPLES = 10;     // Number of consecutive samples to average
const int SAMPLE_DELAY = 10;    // Delay (ms) between consecutive samples 

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
  
  Serial.println("HW-617 Multiplexer + Multi-TF-Luna (Averaging Mode) Initialized.");
  Serial.print("Sampling Rate: ");
  Serial.print(NUM_SAMPLES);
  Serial.println(" samples per reading.\n--------------------------------------------------\n");
}

void loop() {
  // Read and Average Distance from Sensor 1
  tcaSelect(LUNA_CH_1);
  float dist1 = getAverageLunaDistance();
  
  // Read and Average Distance from Sensor 2
  tcaSelect(LUNA_CH_2);
  float dist2 = getAverageLunaDistance();
  
  // Read and Average Distance from Sensor 3
  tcaSelect(LUNA_CH_3);
  float dist3 = getAverageLunaDistance();

  // Print formatted data to Serial Monitor
  printData(1, dist1);
  Serial.print(" | ");
  printData(2, dist2);
  Serial.print(" | ");
  printData(3, dist3);
  Serial.println();

  delay(100); // Main loop delay between full array scans
}

// =========================================================================
// HELPER FUNCTION: Take multiple samples and calculate the average
// =========================================================================
float getAverageLunaDistance() {
  long sum = 0;
  int validSamplesCount = 0;
  
  for (int i = 0; i < NUM_SAMPLES; i++) {
    int rawDistance = getSingleLunaDistance();
    
    // Only include actual physical distances (greater than 0) in the average
    if (rawDistance > 0) {
      sum += rawDistance;
      validSamplesCount++;
    }
    
    // Tiny delay between consecutive laser pulses to let the sensor refresh
    delay(SAMPLE_DELAY); 
  }
  
  // If we got at least one valid reading, return the calculated average
  if (validSamplesCount > 0) {
    return (float)sum / validSamplesCount;
  }
  
  // If every single sample failed, return a fallback error code
  return -1.0; 
}

// =========================================================================
// HELPER FUNCTION: Switch active channel on the HW-617 Multiplexer
// =========================================================================
void tcaSelect(uint8_t channel) {
  if (channel > 7) return; 
  
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel); 
  Wire.endTransmission();
}

// =========================================================================
// HELPER FUNCTION: Read a single raw distance frame from active channel
// =========================================================================
int getSingleLunaDistance() {
  unsigned char triggerCmd[] = {0x5A, 0x05, 0x00, 0x01, 0x60};
  
  Wire.beginTransmission(LUNA_ADDR);
  Wire.write(triggerCmd, 5);
  if (Wire.endTransmission() != 0) {
    return -1; // Connection error
  }
  
  Wire.requestFrom(LUNA_ADDR, (uint8_t)DATA_LENGTH);
  
  uint8_t data[DATA_LENGTH] = {0};
  int index = 0;
  
  while (Wire.available() > 0 && index < DATA_LENGTH) {
    data[index++] = Wire.read();
  }
  
  if (index == DATA_LENGTH && data[0] == 0x59 && data[1] == 0x59) {
    int distance = data[2] + (data[3] << 8); 
    return distance; 
  }
  
  return -2; // Data/packet error
}

// Format printing utility (Now accepts float values for precision averages)
void printData(int id, float distance) {
  Serial.print("Lidar ");
  Serial.print(id);
  Serial.print(": ");
  if (distance > 0) {
    Serial.print(distance, 1); // Prints with 1 decimal place
    Serial.print(" cm");
  } else {
    Serial.print("ERR");
  }
}
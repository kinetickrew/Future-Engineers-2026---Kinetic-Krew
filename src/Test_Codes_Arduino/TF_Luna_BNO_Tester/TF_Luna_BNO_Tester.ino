#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>


// --- I2C Multiplexer Channels (HW-617) ---
#define MUX_CH_LEFT   0
#define MUX_CH_CENTER 1
#define MUX_CH_RIGHT  2
#define MUX_CH_BNO    4


// --- TF Luna I2C Address ---
#define TFLUNA_I2C_ADDR 0x10


// --- Object Declarations ---
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


// --- Custom Structure to Hold Sensor Data ---
struct SensorData {
 int16_t leftDist;
 int16_t centerDist;
 int16_t rightDist;
 float heading;
};


// --- Non-blocking Timing Variable ---
unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 0; // Sample every 200ms (5Hz)


// --- Function Prototypes ---
void selectMuxChannel(uint8_t channel);
int16_t getLunaDistance(uint8_t channel);
SensorData fetchAllData();
void printData(const SensorData& data);


void setup() {
 Serial.begin(115200);
 while(!Serial); // Wait for Serial Monitor to open
  Serial.println("=========================================");
 Serial.println("   ESP32 I2C MULTIPLEXER DIAGNOSTIC      ");
 Serial.println("=========================================");


 // Initialize ESP32 I2C Bus (SDA = Pin 21, SCL = Pin 22)
 Wire.begin(21, 22);
  // FIX 1: Set a short I2C timeout (approx 50ms) so failed reads don't hang the CPU
 Wire.setTimeOut(50);


 // Initialize BNO055 on Channel 4
 selectMuxChannel(MUX_CH_BNO);
 Serial.print("Initializing BNO055 on Channel 4... ");
 if (!bno.begin()) {
   Serial.println("FAILED! Check HW-617 Channel 4 wiring.");
 } else {
   Serial.println("SUCCESS!");
   delay(500);
   bno.setExtCrystalUse(true);
 }
}


void loop() {
 // FIX 2: Non-blocking timer using millis() instead of delay(200)
 if (millis() - lastUpdate >= UPDATE_INTERVAL) {
   lastUpdate = millis();


   // 1. Collect all data
   SensorData currentData = fetchAllData();
  
   // 2. Print the collected data
   printData(currentData);
 }
}


// =========================================================================
// --- HELPER FUNCTIONS ---
// =========================================================================


// --- Switch HW-617 Channels ---
void selectMuxChannel(uint8_t channel) {
 if (channel > 7) return;
 Wire.beginTransmission(0x70); // Default TCA9548A Mux I2C Address
 Wire.write(1 << channel);
 Wire.endTransmission();
}


// --- Direct I2C Reader for TF Luna Distance ---
int16_t getLunaDistance(uint8_t channel) {
 selectMuxChannel(channel);
  // Point to distance registers
 Wire.beginTransmission(TFLUNA_I2C_ADDR);
 Wire.write(0x00);
  // If the initial handshake fails, exit immediately instead of waiting
 if (Wire.endTransmission() != 0) {
   return -1;
 }
  // Request Distance Low and High bytes
 Wire.requestFrom(TFLUNA_I2C_ADDR, 2);
 if (Wire.available() >= 2) {
   uint8_t lowByte = Wire.read();
   uint8_t highByte = Wire.read();
   return (lowByte + (highByte << 8)); // Combine into cm distance
 }
 return -1;
}


// --- Collect All Sensor Readings ---
SensorData fetchAllData() {
 SensorData data;
  // Read TF Luna Distances
 data.leftDist   = getLunaDistance(MUX_CH_LEFT);
 data.centerDist = getLunaDistance(MUX_CH_CENTER);
 data.rightDist  = getLunaDistance(MUX_CH_RIGHT);
  // Read BNO055 Orientation
 data.heading = -1.0;
 selectMuxChannel(MUX_CH_BNO);
  sensors_event_t event;
 if (bno.getEvent(&event)) {
   data.heading = event.orientation.x; // Yaw / Compass heading
 }
  return data;
}


// --- Print Clean Formatted Data ---
void printData(const SensorData& data) {
 Serial.print("LiDAR [L]: ");
 if(data.leftDist == -1) Serial.print("ERR\t"); else { Serial.print(data.leftDist); Serial.print("cm\t"); }
  Serial.print("[C]: ");
 if(data.centerDist == -1) Serial.print("ERR\t"); else { Serial.print(data.centerDist); Serial.print("cm\t"); }
  Serial.print("[R]: ");
 if(data.rightDist == -1) Serial.print("ERR\t"); else { Serial.print(data.rightDist); Serial.print("cm\t"); }


 Serial.print(" | IMU -> H: ");
 if(data.heading == -1.0) Serial.print("ERR"); else Serial.print(data.heading, 1);
 Serial.println("°");
}





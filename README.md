# WRO Sensor & Actuator Test Repository

This repository is a mixed Arduino + Python test workspace for debugging and validating individual sensors, actuators, and I2C devices before they are integrated into the main project.

## Repository Structure

- `Test Codes/`
  - `3_TF_Luna/`
    - `3_TF_Luna.ino` - Arduino test sketch for reading three TF-Luna lidar sensors through an HW-617 I2C multiplexer.
  - `3_TF_Luna_with_Filter/`
    - `3_TF_Luna_with_Filter.ino` - Arduino test sketch that reads three TF-Luna sensors and averages multiple samples to reduce noise.
  - `Servo_Test.ino` - ESP32 servo tester sketch that accepts a serial angle input and moves a servo between 45° and 135°.
  - `TF_Luna_BNO_Tester.ino` - ESP32 diagnostic sketch for TF-Luna lidars and a BNO055 IMU on an HW-617 I2C multiplexer.
- `Python/`
  - Placeholder folder for future Python test scripts and debugging utilities.

## Purpose

This repository is intended for:

- verifying individual hardware components
- testing sensor connections and I2C multiplexer wiring
- debugging TF-Luna lidar readings and BNO055 orientation data
- validating servo movement and serial command handling
- isolating errors before integration into the main robotics codebase

## Included Test Codes

### `3_TF_Luna.ino`
- Reads three TF-Luna lidar sensors using HW-617 I2C multiplexer channels 0, 1, and 2.
- Prints distance readings to the serial monitor.
- Useful for verifying each lidar sensor and multiplexer channel.

### `3_TF_Luna_with_Filter.ino`
- Similar to `3_TF_Luna.ino`, but averages multiple samples per channel.
- Useful when testing noisy distance readings and verifying stable output.
- Configurable sample count and delay between samples.

### `Servo_Test.ino`
- ESP32-based servo test sketch using `ESP32Servo`.
- Accepts serial angle input between 45 and 135 degrees.
- Prints validation messages and moves the servo accordingly.
- Useful for checking servo response and servo wiring.

### `TF_Luna_BNO_Tester.ino`
- ESP32 diagnostic sketch for reading up to three TF-Luna lidar sensors and a BNO055 IMU.
- Uses HW-617 I2C multiplexer channels for the lidars and IMU.
- Prints combined distance and heading data to the serial monitor.
- Useful for verifying both distance sensors and orientation sensor together.

## Usage

1. Open the sketch folder or `.ino` file in the Arduino IDE.
2. Select the correct board and serial port.
3. Install any required libraries (`ESP32Servo`, `Adafruit_BNO055`, `Adafruit_Sensor`) if needed.
4. Upload the sketch.
5. Open the Serial Monitor at `115200` baud to view sensor output.

## Notes

- This repo is focused on test/debug code, not on final control or navigation logic.
- Keep every hardware-specific diagnostic sketch in `Test Codes/`.
- Add Python test tools under `Python/` as they are developed.
- Update this README with new test code names and usage notes when you add new sketches.

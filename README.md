# KineticKrew-FutureEngineer-2026 Skeleton Repository

This repository is a skeleton clone of the structure used by the KineticKrew Future Engineers project. It contains the same top-level folders and placeholders, but no actual project data, images, CAD files, or compiled code.

## Table of Contents

1. Overview
2. Architecture
3. Repository Structure
4. Missing TODO Items
5. Notes

## 1. Overview

This workspace is prepared as an empty repository layout for the Future Engineers project. Use this repo as the starting point for:

- adding robotics design files
- populating code for Round 1 ESP32 and Round 2 Raspberry Pi
- adding documentation, images, and build instructions
- tracking progress with TODO notes in the README

## 2. Architecture

The robot architecture is split into two rounds, with dedicated code folders for each stage:

- Round 1: `code/round-1-esp32/`
  - Contains ESP32 Arduino firmware and sensor/control code for:
    - distance sensor
    - IMU
    - motor driver
    - DC motors
    - steering servo
  - Uses a TCA9548A I2C MUX module to connect the TF-LUNA and BNO055 to the ESP32 on shared I2C lines.
- Round 2: `code/round-2-raspberry-pi/`
  - Contains Raspberry Pi image processing code and control logic that sends commands to the ESP32.
- Shared code and utilities: `code/shared/`
  - Contains any common modules, communication helpers, or libraries used by both rounds.

### 3. Hardware Specifications

* **Distance Sensor – Benewake TF-LUNA Micro LiDAR**

  * **Power Requirement:** 5 V DC, approximately 180 mA
  * **Communication Interface:** UART
  * **Operating Range:** 0.2–8 m
  * **Degrees of Freedom:** 1 (distance measurement)
  * **Description:** Used for real-time obstacle detection by measuring the distance between the vehicle and surrounding objects.

* **Inertial Measurement Unit (IMU) – BNO055**

  * **Power Requirement:** 3.3 V DC, approximately 12 mA
  * **Communication Interface:** I²C
  * **Degrees of Freedom:** 9 DoF
  * **Description:** Provides fused orientation, acceleration, gyroscope, and magnetometer data for vehicle localization and motion estimation.

* **Motor Driver – TB6612FNG**

  * **Power Requirement:** 5 V logic, 4.5–13.5 V motor supply, up to 1.2 A per channel
  * **Communication Interface:** PWM
  * **Motor Outputs:** 2 channels
  * **Description:** Controls the speed and direction of the DC drive motors using PWM signals.

* **DC Drive Motor – BO Motor**

  * **Operating Voltage:** 6–12 V DC
  * **Control Method:** PWM
  * **Typical Speed:** 200–300 RPM
  * **Degrees of Freedom:** 1 rotational axis
  * **Description:** Provides propulsion for the autonomous vehicle. The actual speed depends on the supplied voltage and mechanical load.

* **Steering Servo Motor – MG90S**

  * **Power Requirement:** 5 V DC, 500–900 mA (stall current)
  * **Communication Interface:** PWM
  * **Operating Speed:** Approximately 0.2 s per 60°
  * **Degrees of Freedom:** 1 rotational axis
  * **Description:** A metal-gear servo motor with approximately 180° rotation, used for precise front-wheel steering.

* **I²C Multiplexer – TCA9548A**

  * **Power Requirement:** 3.3 V / 5 V logic
  * **Communication Interface:** I²C
  * **Channels:** 8 independent I²C channels
  * **Description:** Enables multiple I²C devices to communicate with the ESP32 by eliminating address conflicts on the shared I²C bus.


## 4. Repository Structure

The current skeleton includes these directories:

- `.github/workflows/`
- `FreeCAD-Files/`
- `Slicer-Files/`
- `code/`
  - `round-1-esp32/`
    - `Test_Codes_Arduino/`
  - `round-2-raspberry-pi/`
    - `raspberry-pi-5/`
  - `shared/`
- `docs/resources/`
- `other/`
- `t-photos/`
- `v-photos/`
- `video/`

The structure now uses round-specific folders for ESP32 and Raspberry Pi development, with a shared folder for any common code or utilities.

## 3. Missing TODO Items

This repository is intentionally empty. The following content still needs to be added:

- Add GitHub Actions workflows in `.github/workflows/`
- Add FreeCAD assembly and part files in `FreeCAD-Files/`
- Add slicer project files and print settings in `Slicer-Files/`
- Add firmware and application source code in `code/raspberry-pi-5/` and `code/raspberry-pi-pico-2/`
- Add shared library/source code in `code/shared/`
- Add documentation images and resource files in `docs/resources/`
- Add `other/dhcp-server-on-ethernet-port.md` and `other/image-drive-linux.md`
- Add team photos and presentation media in `t-photos/`, `v-photos/`, and `video/`
- Add a project changelog in `CHANGELOG.md`
- Add an appropriate license in `LICENSE`
- Add `.gitignore`, `.gitattributes`, `.clang-format`, and `.gitmodules` if needed
- Fill in the major README sections:
  - Overview
  - Mobility Management
  - Power and Sense Management
  - Obstacle Management
  - Source Code
  - List of Components
  - 3D Model Files
  - Building Instructions
  - Extras

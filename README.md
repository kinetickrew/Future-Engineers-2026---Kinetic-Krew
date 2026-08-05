# Kinetic Krew - Future Engineers 2026

This repository contains the hardware design, firmware, vision pipeline, and documentation for the Kinetic Krew robot built for the Future Engineers 2026 competition. The project is organized around two main layers:

- an ESP32-based control layer for low-level sensing and actuation
- a Raspberry Pi based perception and decision layer for vision and high-level behavior

## Project overview

The repository now groups files by function so the robot stack is easier to navigate:

- Mechanical and CAD assets live in CAD-Files and Slicer-Files
- Firmware and embedded control code live in code/round-1-esp32
- Raspberry Pi code and round-two software live in code/round-2-raspberry-pi
- Vision datasets, annotations, scripts, and trained model assets live in data/ and tools/vision
- Photos, videos, and supporting documentation live in media/ and docs/

## Repository structure

```text
.
├── CAD-Files/              # CAD and mechanical design files
├── Slicer-Files/           # 3D print project files and slicer settings
├── code/
│   ├── round-1-esp32/      # ESP32 firmware and experiments
│   ├── round-2-raspberry-pi/  # Raspberry Pi applications and round-2 code
│   └── shared/             # Shared helpers and reusable modules
├── data/
│   ├── datasets/           # exported datasets and training bundles
│   └── vision/             # images and annotations for object detection
├── docs/                   # project documentation and resources
├── media/                  # photos and videos for documentation and presentations
├── other/                  # miscellaneous notes and utility references
├── tools/vision/           # vision scripts, notebooks, and model assets
└── README.md
```

## Hardware and software focus

### Embedded control layer

The ESP32 side is centered around the round-1 firmware in code/round-1-esp32. It handles:

- sensor polling and telemetry reporting
- motor and steering actuation
- serial communication with the Raspberry Pi

### Vision and autonomy layer

The Raspberry Pi side in code/round-2-raspberry-pi contains the round-two software stack for:

- camera input and image processing
- object detection and pillar tracking
- high-level navigation decisions

The vision assets in data/vision and tools/vision support training and testing for the detection pipeline.

## Key folders

- CAD-Files/: mechanical design files
- Slicer-Files/: printable parts and print preparation assets
- code/round-1-esp32/: firmware and Arduino experiments
- code/round-2-raspberry-pi/: Raspberry Pi application code
- data/vision/: image and annotation data for vision training
- tools/vision/: scripts, notebooks, and model files
- docs/: documentation and supporting resources
- media/: photos and videos

## Getting started

1. Open the relevant folder for the subsystem you are working on.
2. Use the ESP32 code under code/round-1-esp32 for embedded behavior.
3. Use the Raspberry Pi code under code/round-2-raspberry-pi for higher-level control.
4. For vision work, use the scripts and model assets in tools/vision with the data in data/vision.

## Notes

- This repository is actively being organized and expanded.
- If you add new source files, keep them in the most relevant subsystem folder.
- When adding data or media, prefer placing it in data/ or media/ rather than keeping it at the repository root.

## Suggested next steps

- add a LICENSE file
- add a CHANGELOG file
- add CI or automation workflows under .github/
- document the hardware wiring and software setup in docs/


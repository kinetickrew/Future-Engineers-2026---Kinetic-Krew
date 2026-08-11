# WRO 2026 Future Engineers — Round 2: Obstacle Challenge

## Same Car, Harder Job

This is Steve's second event of the season. He's the same physical car documented in the Round 1 writeup — same Ackermann-steered LEGO EV3 chassis, same rear differential, same servo and DC motor — but the Obstacle Challenge asks a lot more of him. Instead of just holding a lane around randomized walls, he now has to _see_ the track: spot red and green pillars, decide which side of the lane each one requires him to take, route around them, and — in later rounds — find and reverse into a parking bay, all without any pre-programmed path.

This document focuses on what's new and specific to Round 2: the computer vision pipeline that turns a camera feed into steering decisions, and how it hands off to the same mechanical and PID systems described in the Round 1 document.

**Note on pipeline history:** an earlier version of this document covered a different detection approach — HSV color thresholding with Kalman filtering and hysteresis-based decisions. That pipeline has since been fully replaced with a trained YOLO object detector running via ONNX Runtime, described below. The old approach is preserved as a historical record in the main engineering README's "Design Evolution" and "Engineering Decisions" sections, not repeated here.

---

## Table of Contents

1. [What the Obstacle Challenge Adds](#what-the-obstacle-challenge-adds)
2. [Shared Hardware Recap](#shared-hardware-recap)
3. [Vision Pipeline Overview](#vision-pipeline-overview)
4. [Step 1 — Model & Inference](#step-1--model--inference)
5. [Step 2 — Preprocessing: Letterboxing](#step-2--preprocessing-letterboxing)
6. [Step 3 — Postprocessing: Decoding & Confidence Filtering](#step-3--postprocessing-decoding--confidence-filtering)
7. [Step 4 — Temporal Confirmation (Voting)](#step-4--temporal-confirmation-voting)
8. [Step 5 — Deciding: CLEAR / STOP / REVERSE](#step-5--deciding-clear--stop--reverse)
9. [Step 6 — Waypoint Geometry: Computing a Pass Point](#step-6--waypoint-geometry-computing-a-pass-point)
10. [Step 7 — From Vision Decision to Motion: The Serial Link](#step-7--from-vision-decision-to-motion-the-serial-link)
11. [Wall-Following Still Applies](#wall-following-still-applies)
12. [Shared PID Steering Loop](#shared-pid-steering-loop)
13. [Vision Detection Accuracy — Testing Results](#vision-detection-accuracy--testing-results)
14. [Glossary](#glossary)

---

## What the Obstacle Challenge Adds

Like the Open Challenge, this is a three-lap, single-car, timed run — no head-to-head racing, just Steve against the clock. The difference is what's on the track: red and green pillars are placed randomly before each run, and each color is a rule, not just an obstacle to dodge blindly.

- A **red** pillar means: stay to the **right** of the lane at that point.
- A **green** pillar means: stay to the **left** of the lane at that point.

In later rounds, the run isn't over after three laps — Steve also has to locate a marked parking space and complete a parallel-parking maneuver to finish.

Because pillar placement is random every run, there's no way to memorize "turn left here, right there." The entire pillar-avoidance system has to work live, frame by frame, off the camera feed — which is why most of this document is about the vision pipeline rather than the mechanical platform (that part is identical to Round 1).

---

## Shared Hardware Recap

Everything mechanical from the Open Challenge carries over unchanged: the LEGO EV3 Technic chassis, Ackermann front steering, rear differential, DC drive motor, and the layered chassis layout that keeps the Raspberry Pi and battery low for a stable center of gravity. See the Round 1 document for the full breakdown of that system.

What's new for this round is primarily software running on top of that same hardware, plus the camera itself — a Lenovo 300 FHD USB webcam, mounted via a custom 3D-printed bracket positioned at a height and angle tuned specifically for reliable pillar detection. Round 1's Raspberry Pi 5 is left unpowered, since Round 1 doesn't need the vision pipeline at all; Round 2 is where the Pi 5 and its ONNX detector come online.

---

## Vision Pipeline Overview

Turning a raw camera frame into a routing decision happens in a fixed sequence of stages:

1. Run the frame through a trained YOLO object detector via ONNX Runtime.
2. Fit the frame into the model's expected input size without distorting pillar proportions (letterboxing).
3. Decode the model's raw output into candidate boxes and discard low-confidence ones.
4. Confirm a detection is real by requiring it to persist across several frames (temporal voting), not just one.
5. Turn a confirmed detection's size into a CLEAR / STOP / REVERSE decision.
6. On STOP, compute a one-time real-world pass point and arc around the pillar.
7. Send the resulting command to the ESP32 over serial.

Each stage is detailed below.

---

## Step 1 — Model & Inference

The detector is a YOLO-family object detection model, fine-tuned from a pretrained `yolo26s.pt` checkpoint on a custom red/green pillar dataset (`dataset.yaml`: 2 classes — `green` = 0, `red` = 1) and exported to ONNX format (`best.onnx` / `best_ncnn.onnx`). Inference runs via **ONNX Runtime**, using the `CPUExecutionProvider`, since the Raspberry Pi 5 has no CUDA-capable GPU.

**Training data pipeline:**

- **Capture (`capture.py`):** saves raw red/green photos to separate folders under manual exposure and white-balance control. This stage produces clean, consistently-lit source images only — no bounding boxes are created here.
- **Annotation:** each captured image was manually annotated with bounding boxes in Pascal VOC XML format using an external labeling tool (e.g. labelImg), which is not part of this repository.
- **Preparation (`prepare.py`):** converts the XML annotations into YOLO's normalized `[class, x_center, y_center, width, height]`label format, maps class-name variants (`red`/`red_b` → class 1, `green`/`green_b` → class 0) to absorb inconsistent labeling, performs an 80/20 train/validation split, writes `dataset.yaml`, and zips the resulting `dataset/` folder for upload to Colab.
- **Training (`google_colab_trainer.ipynb`):** run on Google Colab with a GPU runtime. `dataset.zip` is uploaded and extracted, image/label counts are sanity-checked, and `ultralytics` fine-tunes the pretrained `yolo26s.pt` checkpoint for 200 epochs at 224×224 image size, batch size 16, with early stopping (`patience=20`). Standard HSV-space augmentation is applied during training (`hsv_h=0.015`, `hsv_s=0.7`, `hsv_v=0.4`) specifically to improve robustness to lighting shifts. The best checkpoint is validated (`model.val()`, reporting mAP50 / mAP50-95) and exported to ONNX with a dynamic input shape (`imgsz=224, dynamic=True`). `best.onnx` and a `labelmap.txt` (`green`, `red`) are downloaded from Colab for deployment onto the Raspberry Pi.

This replaces the earlier fixed-threshold HSV color pipeline entirely — instead of hand-tuned Hue/Saturation/Value bounds that had to be recalibrated per lighting condition, the model learns what a pillar looks like directly from labeled examples, which is inherently more robust to lighting shifts than a fixed color threshold. See "Vision Detection Accuracy — Testing Results" below for a direct before/after comparison against the retired HSV pipeline.

---

## Step 2 — Preprocessing: Letterboxing

The camera captures at 640×480 (MJPEG), which is resized to a 240×240 working frame for display and coordinate math. The model itself expects a fixed 224×224 input — a separate, further-resized frame.

Rather than stretching the 240×240 working frame to 224×224 — which would distort pillar proportions — the frame is **letterboxed**: scaled to fit within 224×224 while preserving aspect ratio, then padded with neutral gray (114, 114, 114) to fill the remaining space:

```
scale = 224 / max(frame_height, frame_width)
new_h, new_w = frame_height × scale, frame_width × scale
canvas = 224×224 image filled with gray (114)
paste the resized frame into canvas, centered
```

The scale factor and padding offsets are retained so that detection boxes — computed in the 224×224 letterboxed space — can be mapped back to the original 240×240 working frame's coordinate system after inference.

Letterboxing avoids the aspect-ratio distortion that naive stretching would introduce, which matters for a shape-sensitive detector trained on real pillar proportions.

---

## Step 3 — Postprocessing: Decoding & Confidence Filtering

The model outputs up to 300 candidate detections per frame, each as `[x1, y1, x2, y2, confidence, class_id]`. For each frame:

1. Every candidate below `CONF_THRESHOLD = 0.55` is discarded.
2. Among the remaining candidates, only the **single highest-confidence detection per class** (red, green) is kept — so at most one red and one green box survive per frame, rather than running full NMS across overlapping boxes of the same class.
3. Surviving boxes are mapped back out of letterbox-space into the working 240×240 frame's pixel coordinates using the inverse of the preprocessing scale/offset.

**Update:** this threshold has since been independently validated with a dedicated accuracy-testing pass — 230 trials across three lighting conditions, averaging 92.6% overall accuracy, a meaningful improvement over the retired HSV pipeline's 83.7% overall accuracy across a comparable test structure. See "Vision Detection Accuracy — Testing Results" below for the full breakdown.

---

## Step 4 — Temporal Confirmation (Voting)

Camera frames vibrate with the chassis, and a single frame's detection can be noisy. Rather than tracking a smoothed pixel position across frames (as the old Kalman-filter approach did), the current pipeline holds detections in a rolling history and requires them to appear consistently before being trusted:

|Parameter|Value|
|---|---|
|Vote history length|7 frames|
|Minimum votes to confirm|5 of last 7|

A color is only "confirmed" once it's appeared in at least 5 of the last 7 frames. If both red and green are confirmed simultaneously, the **taller bounding box** (i.e. the closer object) is treated as primary.

No temporal smoothing filter is applied to box position beyond this frame-voting scheme — detections feed directly into the decision logic below once confirmed.

---

## Step 5 — Deciding: CLEAR / STOP / REVERSE

The primary detection's bounding-box height (a proxy for distance — taller box = closer object) drives a three-way decision:

|Condition|Decision|
|---|---|
|No confirmed detection|`CLEAR`|
|Confirmed detection, height < `STOP_HEIGHT_PX` (45px)|`CLEAR` (too far to act on yet)|
|Confirmed detection, height ≥ 45px and ≤ `REVERSE_HEIGHT_PX` (80px)|`STOP`|
|Confirmed detection, height > 80px|`REVERSE`|

This replaces the old pipeline's separate trigger/release hysteresis thresholds and minimum-hold-time logic. Once a `STOP`is triggered, the robot's position and the pillar's position are **frozen** as a one-time waypoint calculation (Step 6) rather than recalculated every frame — the lock persists until 10 consecutive `CLEAR` frames are seen (`CLEAR_HISTORY = 10`), which then releases the waypoint.

---

## Step 6 — Waypoint Geometry: Computing a Pass Point

This stage has no equivalent in the old pipeline, which swerved reactively based on live pixel position rather than computing a target point. Once a pillar triggers `STOP`, the system computes a one-time geometric waypoint describing how the robot should route around it, using a robot-centered coordinate frame frozen at the moment of the stop:

- **B** = the robot's own position at the moment of freeze, defined as the origin (0, 0), with +Y as "straight ahead" (camera axis) and +X as "robot's right."
- **A** = the detected pillar's position, computed from the frozen bounding box via similar-triangles depth/lateral estimation:

```
depth (y_a)    = AB_DISTANCE_CM × (STOP_HEIGHT_PX / box_height)
lateral (x_a)  = (box_center_x − frame_center_x) × aspect_ratio × (REAL_BLOCK_HEIGHT_CM / box_height)
```

where `AB_DISTANCE_CM = 40cm` is a one-time hand-measured calibration constant (the real forward distance from robot to pillar when the box height equals `STOP_HEIGHT_PX`), and `REAL_BLOCK_HEIGHT_CM = 10cm` is the pillar's known physical height, used to convert the box's pixel size into a real-world scale for the lateral estimate.

- **C** = the intended pass point, offset `AC_OFFSET_CM = 25cm` laterally from A — to the robot's right if the pillar is red (drive right of it), to the left if green:

```
x_c = x_a + (25cm if red else -25cm)
y_c = y_a
```

- **Arc from B to C:** a constant-curvature circular arc connecting the robot's current position/heading (B) to the pass point (C) is computed geometrically, giving a signed turn radius (positive = right turn, negative = left, infinite = drive straight), a turn angle, and an arc length.

Rather than reactively swerving based on the pillar's live pixel position every frame, this computes a single target point to route around, in real-world centimeters, using one hand-calibrated distance measurement and the pillar's known physical size.

---

## Step 7 — From Vision Decision to Motion: The Serial Link

Once a decision is made, it's sent to the ESP32 over UART at 115200 baud (`/dev/ttyUSB1`) as a newline-terminated ASCII command:

|ASCII Command|Trigger|Consumed by firmware?|
|---|---|---|
|`CLEAR\n`|No confirmed detection close enough to act on|Yes|
|`STOP,cx,cy,w,h\n`|Pillar height reaches `STOP_HEIGHT_PX`|Yes|
|`WAYPOINT,color,x_a,y_a,x_c,y_c,radius,theta,arc_len\n`|Sent immediately after STOP, once per lock|Yes|
|`REVERSE,cx,cy,w,h\n`|Pillar height exceeds `REVERSE_HEIGHT_PX`|Yes|

This is a richer protocol than the old pipeline's three bare commands (`RED` / `GREEN` / `REVERSE`) — the ESP32 now receives full geometric waypoint data rather than just a color, and computes its own arc from it, converting the received turn radius into a servo angle using its own wheelbase constant (`WHEELBASE_CM = 9.0`, matching the robot's actual physical wheelbase) and the same `atan(wheelbase / radius)` bicycle-model relationship documented in the Round 1 Ackermann sections. The ESP32 still parses the legacy `RED`/`GREEN` commands if received (they enter an `OBSTACLE_AVOIDING` state that swerves hard to one side), but the current vision pipeline no longer sends them.

The vision system's entire job ends here — everything past this point (turning a waypoint into an actual servo angle and drive sequence) runs through the ESP32's state machine and PID loop, detailed in the Round 1 document.

---

## Wall-Following Still Applies

Pillar avoidance doesn't replace the maze/wall-following systems from Round 1 — it layers on top of them. The Obstacle Challenge track still has walls to follow between pillar encounters, so the same ESP32 subsystem — BNO055 heading smoothing and cardinal snapping, the three-LiDAR front-distance turn trigger with locked turn direction, and the debounced multi-sensor stop condition — runs continuously in the background. A confirmed pillar detection interrupts that baseline behavior (`PI_HOLD` → `WAYPOINT_ARC`), then hands control back to straight-line driving once the arc completes or a `CLEAR` is received. Full detail on that subsystem is in the Round 1 document's "ESP32 Maze-Navigating & Self-Stabilizing Subsystem" section.

The same Bluetooth Serial calibration console described there (`KP=`, `KI=`, `KD=`, `CENTER=`, `DIFF=`, `STRAIGHT=`, `BACK=`, `FRONT=`, `ARCANGLE=`, and related turn-arc timing parameters) is used to tune Round 2 behavior too, and is likewise disabled before competition runs. Note that the firmware's `DIFF=25°` setting is the servo-horn command, not the actual front wheel angle — the steering linkage's mechanical leverage ratio reduces this to a measured 14.5° at the wheels themselves (see the Round 1 document's "Steering Mechanism — Ackermann Steering" section for the full turning-radius derivation using this corrected figure).

**Update — WiFi tuning console:** the ESP32 firmware has since also gained a WiFi-based tuning console (`setupWifiTuner()`, a self-hosted access point serving a browser-based slider UI with values persisted to flash) covering a wider set of tunables, including servo limits, drive speeds, and turn-arc timing constants. This console is currently disabled in code (commented out in `setup()`); Bluetooth Serial remains the active tuning method for both rounds. See the Round 1 document's "Live Parameter Tuning Interface" engineering decision for the history behind this.

---

## Shared PID Steering Loop

Whether a steering correction originates from a pillar's computed waypoint arc or from wall-following heading error, straight-line driving ultimately drives the same servo through the same PID controller:

```
u(t) = Kp·e(t)  +  Ki·∫e(t)dt  +  Kd·(de(t)/dt)
```

- **Kp** governs the strength of the immediate correction.
- **Ki** cleans up small persistent bias (e.g. a slightly off-center servo neutral point), and is clamped and deadband-gated to prevent windup.
- **Kd** damps the response to avoid overshoot as the target heading is approached.

Waypoint arcs and turn arcs themselves are not driven by this PID loop — they use a fixed servo-crank angle or a geometrically-computed angle from a turn radius, applied directly. The PID loop governs straight-line heading hold only. Full derivation and background on this shared control loop is covered in the Round 1 document's "PID Control — Background Theory" section.

---

## Vision Detection Accuracy — Testing Results

Following the same test procedure used to originally validate the retired HSV pipeline — camera pointed at a known set of pillars (mix of red/green, at varying distances/angles) per lighting condition, scoring each frame as a true positive, false positive, or missed detection — an equivalent accuracy-testing pass has now been run on the current YOLO/ONNX detector, closing the validation gap noted in Step 3 above.

|Lighting Condition|Trials (N)|True Positives|False Positives|Missed Detections|Accuracy (%)|
|---|--:|--:|--:|--:|--:|
|Orange (incandescent, ~2700K)|40|37|1|2|92.5%|
|White (daylight LED, ~6500K)|60|55|2|3|91.7%|
|Mixed / Venue Overhead|130|121|3|6|93.1%|
|**Overall**|**230**|**213**|**6**|**11**|**92.6%**|

Accuracy (%) = True Positives / Trials × 100

**Result:** accuracy ranged from 91.7% (White lighting) to 93.1% (Mixed/Venue Overhead) across tested conditions, averaging 92.6% overall — a meaningful improvement over the retired HSV pipeline's 83.7% overall accuracy across a comparable test structure. Unlike the HSV pipeline, the YOLO detector did not show a pronounced accuracy drop under Mixed/Venue Overhead lighting, consistent with a learned detector being less sensitive to lighting-driven color shifts than a fixed HSV threshold. Full detail, including the retired pipeline's original comparison table, is in the main README's "Engineering Decisions" section.

---

## Glossary

|Term|Meaning|
|---|---|
|**Confidence threshold**|The minimum model confidence (0.55) a candidate detection must clear before being considered.|
|**Letterboxing**|Resizing an image to fit a target size while preserving aspect ratio, padding the remaining space rather than stretching.|
|**NMS**|Non-Maximum Suppression — a technique for removing duplicate/overlapping detections around the same object; this pipeline uses a simplified per-class best-detection selection instead of full NMS.|
|**ONNX**|Open Neural Network Exchange — a portable format for trained machine learning models, allowing them to run across different runtimes/hardware.|
|**PID controller**|Proportional–Integral–Derivative feedback controller used to drive a measured value toward a setpoint.|
|**PWM**|Pulse Width Modulation — a technique for encoding an analog-like signal (servo angle, motor speed) as a digital pulse train.|
|**Temporal voting**|Requiring a detection to appear in several of the last N frames (5 of 7) before it's trusted, to reject single-frame noise.|
|**UART**|Universal Asynchronous Receiver/Transmitter — the serial protocol linking the vision system to the ESP32.|
|**Waypoint**|A one-time computed real-world pass point (and arc) the robot routes through to get around a confirmed pillar.|
|**YOLO**|"You Only Look Once" — a family of real-time object detection neural network architectures that predict bounding boxes and classes in a single pass.|

---

_This document covers the Obstacle Challenge (Round 2) vision pipeline and decision logic for Steve, our WRO 2026 Future Engineers entry. For the mechanical platform and shared PID/wall-following subsystem, see the companion Round 1 document._
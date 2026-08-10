# WRO 2026 Future Engineers — Round 2: Obstacle Challenge

## Same Car, Harder Job

This is Steve's second event of the season. He's the same physical car documented in the Round 1 writeup — same Ackermann-steered LEGO EV3 chassis, same rear differential, same servo and DC motor — but the Obstacle Challenge asks a lot more of him. Instead of just holding a lane around randomized walls, he now has to _see_ the track: spot red and green pillars, decide which side of the lane each one requires him to take, swerve accordingly, and — in later rounds — find and reverse into a parking bay, all without any pre-programmed path.

This document focuses on what's new and specific to Round 2: the computer vision pipeline that turns a camera feed into steering decisions, and how it hands off to the same mechanical and PID systems described in the Round 1 document.

---

## Table of Contents

1. [What the Obstacle Challenge Adds](#what-the-obstacle-challenge-adds)
2. [Shared Hardware Recap](#shared-hardware-recap)
3. [Vision Pipeline Overview](#vision-pipeline-overview)
4. [Step 1 — Fixing the Lighting: Gray-World White Balance](#step-1--fixing-the-lighting-gray-world-white-balance)
5. [Step 2 — Recovering Shadow Detail: CLAHE](#step-2--recovering-shadow-detail-clahe)
6. [Step 3 — Color Isolation: HSV Hue Topology & Thresholding](#step-3--color-isolation-hsv-hue-topology--thresholding)
7. [Step 4 — Cleaning Up the Mask: Morphological Operations](#step-4--cleaning-up-the-mask-morphological-operations)
8. [Step 5 — Is It Actually a Pillar? Shape & Confidence Scoring](#step-5--is-it-actually-a-pillar-shape--confidence-scoring)
9. [Step 6 — Smoothing the Signal: Kalman Filtering](#step-6--smoothing-the-signal-kalman-filtering)
10. [Step 7 — Deciding & Committing: Hysteresis and Hold Logic](#step-7--deciding--committing-hysteresis-and-hold-logic)
11. [From Vision Decision to Motion: The Serial Link](#from-vision-decision-to-motion-the-serial-link)
12. [Wall-Following Still Applies](#wall-following-still-applies)
13. [Shared PID Steering Loop](#shared-pid-steering-loop)
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

Everything mechanical from the Open Challenge carries over unchanged: the LEGO EV3 Technic chassis, Ackermann front steering, rear differential, DC drive motor, and the two-tier layout that keeps the Raspberry Pi and battery low for a stable center of gravity. See the Round 1 document for the full breakdown of that system.

What's new for this round is primarily software running on top of that same hardware, plus the camera itself — mounted via a custom 3D-printed bracket positioned at a height and angle tuned specifically for reliable pillar detection.

---

## Vision Pipeline Overview

Turning a raw camera frame into a steering decision happens in a fixed sequence of stages, each one solving a specific failure we actually ran into on the physical track — not a generic "best practice" checklist, but a pipeline shaped by real testing:

1. Fix inconsistent competition-venue lighting (white balance).
2. Recover detail lost in shadows without blowing out highlights (CLAHE).
3. Isolate red and green pixels reliably, including red's awkward position at the hue wraparound (HSV thresholding).
4. Clean up noise and fill gaps in the resulting mask (morphological operations).
5. Confirm that a surviving blob is actually shaped like a pillar, not some other red or green object in the arena (shape and confidence scoring).
6. Smooth the pillar's tracked position across frames, even through dropped or occluded frames (Kalman filtering).
7. Convert the smoothed position into a stable, non-flickering steering decision (hysteresis and minimum hold time).

Each stage is detailed below.

---

## Step 1 — Fixing the Lighting: Gray-World White Balance

Competition venues aren't lit consistently — we've seen anything from warm 2700K incandescent bulbs to cool 6500K daylight LEDs, sometimes in the same room. Left uncorrected, that shift skews the Hue channel enough to break a fixed color threshold that worked fine in testing but fails under different venue lighting.

The Gray-World algorithm assumes that, on average, a natural scene's reflectance is roughly neutral gray. We compute the mean intensity of each color channel across the frame:

<p align="center"><img src="images/cv_01_mean_channel_intensity.png" width="220"></p>

average those three channel means into a single target gray value:

<p align="center"><img src="images/cv_02_grayworld_target.png" width="260"></p>

and then scale every pixel in each channel by a per-channel gain factor (with a tiny ε = 10⁻⁶ floor to avoid dividing by zero in near-total darkness):

<p align="center"><img src="images/cv_03_grayworld_gain.png" width="320"></p>

Doing this correction in linear RGB space _before_ converting to HSV means our downstream color thresholds stay valid even as the venue's lighting color temperature drifts over the course of a competition day.

---

## Step 2 — Recovering Shadow Detail: CLAHE

Plain histogram equalization stretches contrast across an entire image using its global cumulative distribution — which sounds good until you apply it to a mostly-flat region and it amplifies noise instead of detail.

**CLAHE (Contrast Limited Adaptive Histogram Equalization)** instead works locally, tile by tile — in our implementation, an 8×8 grid of contextual regions. Each tile's histogram is clipped at a set limit (`clipLimit = 2.0`):

<p align="center"><img src="images/cv_04_clahe_clip_limit.png" width="360"></p>

Pixels that would have exceeded that clip limit are redistributed evenly across the rest of the histogram before the local contrast transform is computed:

<p align="center"><img src="images/cv_05_clahe_cdf_transform.png" width="300"></p>

Neighboring tiles are blended with bilinear interpolation so there's no visible seam at tile boundaries.

We apply this only to the **Value** channel in HSV space. That's a deliberate choice: strong directional shadows across the arena floor hide real detail in dark regions, and CLAHE on the V channel pulls that detail back out — and suppresses blown-out highlights — without touching Hue or Saturation, which is where our actual color classification happens.

---

## Step 3 — Color Isolation: HSV Hue Topology & Thresholding

Hue in HSV space isn't a line — it's a circle. H runs from 0° to just under 360° (or 0–179 in OpenCV's byte-scaled representation) and wraps back around to 0°:

<p align="center"><img src="images/cv_06_hue_circle_topology.png" width="220"></p>

That circular structure is exactly why red is annoying to threshold: pure red sits right at the 0° seam, so its natural variation spills across _both_ sides of the wraparound. A single contiguous hue range either misses part of the red spectrum or has to be widened enough to start catching oranges and magentas. Our fix is to split red into two separate ranges — one near the top of the hue scale, one near the bottom — and treat a pixel as red if it falls in _either_ range:

<p align="center"><img src="images/cv_07_hsv_red_mask.png" width="340"></p>

Green doesn't have this problem — it sits comfortably away from the wraparound (around H ≈ 60°, or 30 in OpenCV units) — so a single contiguous range is enough:

<p align="center"><img src="images/cv_08_hsv_green_mask.png" width="340"></p>

Handling the wraparound explicitly, rather than just widening a single range and hoping, is what keeps us from picking up false positives off orange or magenta arena markers.

---

## Step 4 — Cleaning Up the Mask: Morphological Operations

Even after color and lighting correction, a raw color mask is noisy — stray single pixels pass the threshold from sensor noise or specular reflections, and legitimate pillar blobs sometimes come out with small internal holes. We clean this up with two standard morphological operations using a 5×5 structuring element:

- **Opening** (erosion then dilation) — strips out small isolated noise pixels that are smaller than the structuring element.

<p align="center"><img src="images/cv_09_morphological_opening.png" width="340"></p>

- **Closing** (dilation then erosion) — fills small holes inside an otherwise-solid detected region, welding fragmented pieces of the same blob back together.

<p align="center"><img src="images/cv_10_morphological_closing.png" width="340"></p>

By the end of this stage, what's left is a clean set of solid, contiguous blobs — ready to be evaluated as pillar candidates.

---

## Step 5 — Is It Actually a Pillar? Shape & Confidence Scoring

Color alone isn't enough to confirm something is a pillar — plenty of things in an arena might be red or green. Any surviving contour with an area above 150 px gets scored on three independent measures:

- **Aspect ratio (S_aspect):** computed from `a = width/height`, then normalized as `S_aspect = clip((a − 1.0)/(b − 1.0), 0, 1)` with `b = 2.0`. Pillars have a predictable, narrow shape, and this rewards contours that match it.
- **Solidity (S_solidity):** `A_contour / A_hull`, where `A_hull` is the area of the contour's convex hull. A pillar should be close to fully solid; a low solidity score usually means the mask is fragmented or the shape doesn't match a pillar.
- **Color centrality (S_color):** how close the contour's mean Saturation and Value sit to the center of our calibrated color range, rather than just barely clearing the threshold edge.

<p align="center"> <img src="images/cv_11_color_centrality_sat.png" width="300"> </p> <p align="center"> <img src="images/cv_12_color_centrality_cont.png" width="300"> <img src="images/cv_13_color_centrality_val.png" width="300"></p>

These three combine into a single confidence score:

```
Confidence = round(100 × (0.4 · S_solidity + 0.3 · S_aspect + 0.3 · S_color))
```

Anything scoring below 55%, or failing hard bounds (`width/height < 1.2` or `solidity < 0.80`), gets thrown out before it ever reaches the steering logic. This is what stops background clutter that happens to be red or green from being mistaken for a real pillar.

---

## Step 6 — Smoothing the Signal: Kalman Filtering

Camera frames vibrate with the chassis, and occasionally drop or get briefly occluded. Feeding raw, jittery per-frame pillar positions straight into steering would produce visibly twitchy corrections.

We track each pillar with a discrete linear Kalman filter over a 4-element state vector — 2D position plus 2D velocity in image coordinates — following the standard predict/correct cycle: predict the next state from the motion model, then correct it against whatever measurement (if any) came in that frame. When a frame is dropped or the pillar is briefly occluded, the filter simply predicts forward using the motion model instead of correcting, so tracking doesn't fall apart during a momentary gap — it picks back up smoothly once real measurements resume.

---

## Step 7 — Deciding & Committing: Hysteresis and Hold Logic

A pillar's tracked x-position naturally jitters a little even after Kalman smoothing, and if the steering decision is allowed to flip every time that position crosses a single trigger line, the car oscillates instead of committing to a clean avoidance maneuver.

We solve this with separate trigger and release thresholds (hysteresis) plus a minimum hold time, so a decision, once made, sticks around long enough to actually be useful:

```
      LEFT TRIGGER (x < 75)      LEFT RELEASE (x > 90)
◄───────────────┼───────────────────────┼──────►
 [TRIGGER RED]  │   HYSTERESIS REGION   │      [CLEAR ZONE]
                │  (Retains RED state)  │
```

|Parameter|Value|
|---|---|
|Left Trigger|x ≤ 90 px|
|Right Trigger|x ≥ 150 px|
|Release Margin|15 px|
|Left Release|x ≤ 75 px|
|Right Release|x ≥ 165 px|
|Minimum Hold Time|≥ 3.0 s|
|Release Confirmation|6 continuous frames|

That translates into a simple state machine:

- **Red** — x-center > 90 and height between 30–80 px.
- **Green** — x-center < 150 and height between 30–80 px.
- **Reverse** — height exceeds 80 px (the pillar has gotten close enough to demand an emergency reverse instead of a swerve).
- **Clear** — only once at least 3.0 seconds have passed, the position is back within release bounds, _and_ that's been confirmed for 6 straight frames.

Requiring several confirming frames before releasing a decision is what keeps a single noisy frame from prematurely canceling an avoidance maneuver that's still in progress.

---

## From Vision Decision to Motion: The Serial Link

Once a decision is confirmed, it's sent to the ESP32 over UART at 115200 baud (`/dev/ttyUSB0`) as a simple newline-terminated ASCII command:

|Command|Trigger|Resulting Behavior|
|---|---|---|
|`RED\n`|Confirmed red pillar, right side|Swerve left|
|`GREEN\n`|Confirmed green pillar, left side|Swerve right|
|`REVERSE\n`|Pillar height > 80 px|Emergency reverse|

The vision system's entire job ends here — everything past this point (turning a swerve command into an actual servo angle) runs through the same PID steering loop used for wall-following in Round 1.

---

## Wall-Following Still Applies

Pillar avoidance doesn't replace the maze/wall-following systems from Round 1 — it layers on top of them. The Obstacle Challenge track still has walls to follow between pillar encounters, so the same ESP32 subsystem — BNO055 heading smoothing and cardinal snapping, the three-LiDAR wall-spike detection, layout locking, and the debounced multi-sensor stop condition — runs continuously in the background. Pillar detections interrupt that baseline behavior with a temporary swerve, then hand control back once the hold period clears. Full detail on that subsystem is in the Round 1 document.

The same Bluetooth Serial calibration console described there (`KP=`, `KI=`, `KD=`, `CENTER=`, `DIFF=`, `STRAIGHT=`, `TURN=`) was used to tune Round 2 behavior too, and was likewise disabled before competition runs.

---

## Shared PID Steering Loop

Whether a steering correction originates from a pillar's Kalman-filtered pixel offset or from the wall-following heading error, both ultimately drive the same servo through the same PID controller:

```
u(t) = Kp·e(t)  +  Ki·∫e(t)dt  +  Kd·(de(t)/dt)
```

- **Kp** governs the strength of the immediate correction.
- **Ki** cleans up small persistent bias (e.g. a slightly off-center servo neutral point).
- **Kd** damps the response to avoid overshoot as the target is approached.

The controller's output becomes a servo pulse-width offset from `SERVO_CENTER`, clamped to the mechanical `DIFF` limit so a pillar-avoidance swerve can never command more angle than the Ackermann linkage can physically deliver. Full derivation and background on this shared control loop is covered in the Round 1 document.

---

## Glossary

|Term|Meaning|
|---|---|
|**CLAHE**|Contrast Limited Adaptive Histogram Equalization — local, noise-controlled contrast enhancement.|
|**HSV**|Hue-Saturation-Value color space, generally more lighting-robust than RGB for color-based detection.|
|**Hysteresis**|Using separate trigger/release thresholds to stop rapid flip-flopping near a decision boundary.|
|**Kalman filter**|A recursive predict-correct algorithm for estimating a system's true state from noisy measurements.|
|**PID controller**|Proportional–Integral–Derivative feedback loop used to drive a measured value toward a target setpoint.|
|**PWM**|Pulse Width Modulation — encodes an analog-like signal as a digital pulse train.|
|**UART**|Universal Asynchronous Receiver/Transmitter — the serial protocol linking the vision system to the ESP32.|

---

_This document covers the Obstacle Challenge (Round 2) vision pipeline and decision logic for Steve, our WRO 2026 Future Engineers entry. For the mechanical platform and wall-following subsystem shared with the Open Challenge, see the companion Round 1 document._
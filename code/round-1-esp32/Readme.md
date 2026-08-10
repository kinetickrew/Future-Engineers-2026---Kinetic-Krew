# WRO 2026 Future Engineers — Round 1: Open Challenge

## Meet Steve

Our car doesn't have a clever acronym for a name — we just call him **Steve**. Between tuning a Kalman filter, wiring up a three-way LiDAR array, and chasing down heading drift near the 0°/360° wraparound, we figured the robot had earned a name with a bit more personality than "Unit 7." Steve seemed to fit — and given how cleanly he snaps back to a heading and holds a line down a straight, he wears it well.

This document covers everything relevant to the **Open Challenge** round specifically: the mechanical platform, the steering and drivetrain systems, and the heading-holding subsystem that keeps Steve driving straight and turning cleanly through three laps of randomized wall placements — with no pillars and no parking involved.

---

## Table of Contents

1. [What the Open Challenge Actually Tests](#what-the-open-challenge-actually-tests)
2. [Big-Picture Architecture](#big-picture-architecture)
3. [Build Materials — Why LEGO EV3 Over a Pure 3D Print](#build-materials--why-lego-ev3-over-a-pure-3d-print)
4. [Two-Tier Chassis Layout](#two-tier-chassis-layout)
5. [Ackermann Steering Geometry](#ackermann-steering-geometry)
6. [Turning Ackermann Geometry Into Servo Motion](#turning-ackermann-geometry-into-servo-motion)
7. [The Rear Differential](#the-rear-differential)
8. [DC Motor & Power Delivery](#dc-motor--power-delivery)
9. [Heading-Holding & Wall-Following Subsystem (ESP32)](#heading-holding--wall-following-subsystem-esp32)
10. [PID Control, Explained](#pid-control-explained)
11. [Glossary](#glossary)

---

## What the Open Challenge Actually Tests

The Open Challenge is the "pure driving" round of WRO Future Engineers: three autonomous laps around a track where only the _inside_ wall positions are randomized before the run. There are no red/green pillars to react to and no parking maneuver at the end — it's purely a test of how accurately the car can hold a lane, read the track boundaries, and execute smooth, repeatable turns lap after lap.

Because we can't memorize a fixed path (the inner wall layout changes every run), Round 1 lives or dies on two things working together:

- **Mechanical precision** — steering geometry and drivetrain behavior that translate a commanded heading into predictable, low-slip motion.
- **Closed-loop heading correction** — sensor-driven logic that continuously compares where the car is pointed against where it should be pointed, and nudges the steering accordingly.

Everything below is the hardware and firmware stack that makes that possible.

---

## Big-Picture Architecture

For this round, Steve's systems break down into three cooperating pieces:

1. **Mechanical base** — a LEGO EV3 Technic chassis with Ackermann-geometry front steering and a rear differential, supplemented by 3D-printed mounts for sensors and actuators the LEGO system can't hold on its own.
2. **Sensing** — a three-sensor TF-Luna LiDAR array reading distances to the left, right, and forward, plus a BNO055 orientation sensor giving an absolute compass heading.
3. **Control** — an ESP32 running a finite-state machine and a PID steering loop that turns "where are the walls, and which way am I facing" into servo commands.

---

## Build Materials — Why LEGO EV3 Over a Pure 3D Print

Steve's frame is built primarily from LEGO EV3 Technic beams, plates, gears, and connectors, with a handful of custom 3D-printed brackets filling in where the LEGO system runs out of options.

We leaned on EV3 for the structural backbone for a few concrete reasons:

- **Manufacturing precision.** EV3 parts are injection-molded to tight, consistent tolerances, which keeps axles, gears, and bearings aligned run after run.
- **Rigidity without extra weight.** The beam-and-connector system is stiff enough to resist flex under load while staying light.
- **Fast iteration.** Because everything snaps together, we could restructure a section of the chassis in minutes rather than reprinting and waiting.
- **Durability under repeated stress.** A 3D-printed frame of comparable size is more prone to layer delamination, warping, and slow dimensional drift under repeated mechanical loading — all of which quietly degrade gear mesh and steering accuracy over a season of testing. EV3's consistency avoids that failure mode.

That said, LEGO alone can't mount modern electronics cleanly, so we designed and printed four custom parts to bridge the gap:

<p align="center"> <img src="images/photo_ev3_mindstorms_kit.png" width="520"><br> <em>The core EV3 Mindstorms set that supplies the Technic beams and gearing Steve's frame is built around.</em> </p>

- **LiDAR mount** — holds the TF-Luna array with a clear, unobstructed scanning field.
- **Camera mount** — positioned at a height and angle tuned for downstream vision work (relevant mainly in Round 2, but physically present on the same chassis).
- **DC motor mount** — locks the drive motor rigidly to the drivetrain to cut down on wasted motion.
- **Servo mount** — the most steering-critical part on the car: it keeps the servo horn and steering linkage in fixed alignment, which directly controls how consistent our Ackermann angles are from run to run.

The result is a chassis that gets EV3's manufacturing consistency for the load-bearing structure, with 3D printing filling in exactly where standardized parts fall short.

---

## Two-Tier Chassis Layout

Rather than bolting every component onto a single flat deck, we split the chassis into two levels using LEGO Technic panel plates as the dividing floor.

- **Top tier** — wiring and electronics that benefit from being accessible for debugging, and from being kept clear of the drivetrain and steering linkage below.
- **Bottom tier** — the heaviest components, namely the Raspberry Pi and battery pack, mounted as low as the frame allows.

Putting the heavy mass down low isn't just tidy packaging — it directly changes how the car behaves in corners. A lower center of gravity means less weight transfer during a turn, which keeps body roll down and helps all four wheels stay evenly loaded against the track surface. More even wheel loading means more usable grip, which in turn means Steve can carry more speed into a corner without losing the front end or unsettling the rear. It also makes the whole platform noticeably calmer during quick direction changes, which matters a lot when the wall layout — and therefore the exact turn sequence — is different every run.

---

## Ackermann Steering Geometry

### The Problem It Solves

When any four-wheeled vehicle turns, the inside front wheel is tracing a tighter circle than the outside front wheel. If both wheels are steered to the exact same angle, one of them is geometrically wrong for the turn it's actually making — and the tire scrubs sideways instead of rolling cleanly.

Ackermann geometry fixes this by deliberately steering the two front wheels to _different_ angles: the inner wheel turns more sharply than the outer wheel, and both are angled so their steering axes converge on the same point — the vehicle's instantaneous center of rotation.

<p align="center"> <img src="images/diagram_ackermann_linkage.png" width="420"><br> <em>The Ackermann linkage: tie-rod geometry (A, B) angled so both wheels' steering axes meet at a common point (C) on the extended rear-axle line.</em> </p>

### The Governing Equation

For a chassis with wheelbase **L** (front-to-rear axle distance) and track width **W** (side-to-side distance between the steering pivots), the low-speed, no-slip relationship between the inner angle δᵢ and correct outer angle δₒ is:

```
cot(δₒ) = cot(δᵢ) + W / L
```

In plain terms: the outer wheel should always sit at a slightly shallower angle than the inner wheel — never the same. As a concrete example, on a chassis with a 100 mm wheelbase and 60 mm track width, steering the inner wheel to 20° calls for an outer angle around 16–17°, not a matching 20°. Steer both wheels equally instead, and the outer tire is forced to scrub sideways, bleeding energy and wearing the tire unnecessarily.

Geometrically, this condition is equivalent to saying: if you extend a line through each kingpin axis and out through the tip of its steering arm, both lines should cross at one point on the rear axle's centerline. When our linkage satisfies that, both front tires roll cleanly around a shared turn center instead of skidding.

This effect is most important in sharp turns — at very shallow steering angles the inner/outer difference is small enough to barely matter, but the tighter the corner, the more a mismatched angle costs in scrub and wear. Since Round 1 is full of tight, back-to-back corners around randomized wall placements, this is exactly the regime where getting the geometry right pays off.

It's also worth noting that full-size road cars often run _less_ than textbook-perfect Ackermann geometry at speed, because tire slip angles at higher speeds naturally provide some of the correction that pure geometry provides at parking-lot speeds. Steve, running small and slow by comparison, doesn't get that free correction from tire slip — so we tuned our linkage toward closer-to-ideal Ackermann geometry rather than compromising for high-speed behavior we'll never see.

<p align="center"> <img src="images/diagram_davis_steering.png" width="480"><br> <em>Davis steering — a sliding-linkage alternative we evaluated and ultimately passed on in favor of the pivoted trapezoidal Ackermann linkage, for lower friction and simpler mounting on the EV3 frame.</em> </p>

### Why This Mattered for Round 1 Specifically

Open Challenge scoring is entirely about clean, repeatable lane-following with no obstacles to distract from steering quality. That makes cornering precision and low tire scrub disproportionately valuable — every degree of steering error compounds over three full laps. Ackermann geometry, paired with a rear differential (below), gives us smoother, more predictable trajectories than a differential-drive layout would, and produces vehicle dynamics that are much easier to reason about and tune in software.

### Simplified View

```
        Front

      /         \
 Inner Wheel   Outer Wheel
 (larger angle) (smaller angle)

        \       /
         \     /
   Instantaneous
 Rotation Center

        Rear Drive Wheels
```

---

## Turning Ackermann Geometry Into Servo Motion

The physical steering angle is actuated by a single high-torque servo — chosen over a plain DC motor specifically because a servo holds a _commanded position_, rather than just spinning, which is what precise steering requires.

<p align="center"> <img src="images/photo_servo_gearbox_assembly.png" width="480"><br> <em>The servo gearbox mounted to the chassis, meshing directly into the steering linkage gear train.</em> </p>

The chain of motion looks like this: the control loop decides a target heading correction → sends a PWM position command to the servo → the servo rotates and pulls or pushes the steering rod → both steering arms rotate together → because of the Ackermann geometry built into the arm lengths and pivot points, the inner and outer wheels automatically land at their correct, _different_ angles without any extra computation.

This setup gives us a few practical advantages for Round 1's heading-holding task:

- **Positional accuracy** — the servo goes to a specific angle, not just "some rotation," which is what lets us make small continuous corrections rather than coarse on/off steering.
- **Fast response** — quick enough to make small mid-corridor corrections without visibly lagging the control loop.
- **Repeatability** — the same command produces the same wheel angle run after run, which matters when comparing lap times.
- **Simple integration** — standard PWM control works directly with the ESP32, no custom driver needed.

---

## The Rear Differential

### What It Does

A LEGO EV3 differential gear sits between the two rear wheels and lets them spin at different speeds — which is exactly what's needed any time the car isn't going perfectly straight, since the outer rear wheel travels a longer arc than the inner one during a turn.

- **Straight-line driving:** both rear wheels spin at the same speed, and the internal bevel gears inside the differential don't rotate relative to each other.
- **Cornering:** the outer wheel is allowed to spin faster than the inner wheel, while the differential keeps sending torque to both, so neither wheel loses traction or gets dragged.

### How It Pairs With Ackermann Steering

The front steering geometry and the rear differential are solving two halves of the same problem. The front wheels are pointed correctly thanks to Ackermann geometry; the rear differential then lets the rear wheels actually follow through at the different speeds that geometry implies. Neither system alone would be enough — Ackermann steering without a differential (or an equivalent) would still leave the rear wheels fighting each other through a turn.

```
            Drive Motor
                 │
                 ▼
        ┌─────────────────┐
        │  Differential   │
        └─────────────────┘
          │             │
          ▼             ▼
     Left Rear     Right Rear
       Wheel          Wheel

Straight:  Same wheel speed
Turning:   Outer wheel rotates faster than inner wheel
```

We picked the stock EV3 differential specifically because it's a fully mechanical solution — it needs zero lines of code to compensate for wheel-speed differences, which is one less thing to tune or get wrong during a competition run.

---

## DC Motor & Power Delivery

Propulsion comes from a single DC motor driving into the differential. The motor supplies continuous rotational power; the differential's job is purely to split that power correctly between the two rear wheels depending on whether the car is going straight or turning. From the motor's point of view, it just spins at whatever speed the drive logic commands — all the speed-splitting complexity during a turn is handled downstream, mechanically, by the differential.

---

## Heading-Holding & Wall-Following Subsystem (ESP32)

This is the part of the stack that does the actual "driving" during Round 1 — reading the walls, holding a heading, and deciding when and how sharply to turn.

### Architecture

A dedicated ESP32 runs a finite-state machine layered on top of a continuous PID control loop. The state machine handles discrete decisions (has a wall ended? has the car completed enough turns to stop?), while the PID loop continuously fine-tunes steering to hold whatever heading is currently targeted.

### Sensors & Wiring

|Peripheral|Pin|Mux Channel|Protocol / Address|
|---|---|---|---|
|I2C SDA|GPIO 21|–|I2C Data|
|I2C SCL|GPIO 22|–|I2C Clock|
|Steering Servo|GPIO 13|–|PWM|
|Left Motor IN1|GPIO 25|–|GPIO Output|
|Left Motor IN2|GPIO 26|–|GPIO Output|
|Motor PWM|GPIO 33|–|PWM Duty Cycle|
|LiDAR Left|–|Ch 0|0x10 (I2C)|
|LiDAR Center|–|Ch 1|0x10 (I2C)|
|LiDAR Right|–|Ch 2|0x10 (I2C)|
|BNO055 IMU|–|Ch 4|0x28 (I2C)|

Since all three TF-Luna LiDAR units share the same default I2C address (`0x10`), we route them through a TCA9548A I2C multiplexer and select each one with a simple bit-shift: `1 << channel` — writing `1 << 0`, for instance, isolates the bus to just the left sensor.

### Getting a Clean Heading Signal

Raw gyroscope/magnetometer data from the BNO055 is noisy and, worse, discontinuous at the 0°/360° wraparound — a naive average of 1° and 359° comes out near 180°, which is exactly backwards. We fix this in two layers:

1. **Shortest-path angle wrapping.** Any raw angular delta is first normalized into the range [−180°, 180°] before it's used, so the filter always corrects across the _short_ way around the circle, never the long way.
2. **Exponential moving average smoothing.** The wrapped delta is blended into a running heading estimate with a smoothing factor of α = 0.2, which knocks down high-frequency jitter without introducing much lag.

<p align="center"><img src="images/esp_01_heading_wrap_delta.png" width="280"></p> <p align="center"><img src="images/esp_02_heading_ema_update.png" width="280"></p>

### Cardinal Snapping

Over a long straight, small heading errors accumulate — the car might drift toward aiming at 86.4° instead of a clean 90°. To stop that drift from compounding, we snap the target heading to whichever of {0°, 90°, 180°, 270°} is closest, guaranteeing every straight segment is aligned to the track's actual grid rather than to whatever error has quietly built up.

<p align="center"><img src="images/esp_03_cardinal_snap_distance.png" width="260"></p>

### Reading the Walls: Spike Detection & Layout Locking

The three LiDAR sensors continuously report distance to the left and right walls. We watch the _difference_ between those two readings:

```
Δ_dist = dist_left − dist_right
```

- Δ_dist jumping above +100 cm means the left wall has opened up → trigger a left turn.
- Δ_dist dropping below −100 cm means the right wall has opened up → trigger a right turn.

The first time a valid turn triggers, we lock in a `lockedDirectionLeft` flag and stop evaluating the opposite-direction condition entirely. This "layout locking" step turned out to matter a lot in practice — without it, a noisy reading right after a real turn could occasionally trigger a false second turn in the wrong direction inside a complex maze-like layout.

### Knowing When to Stop

The run only counts as finished when _all_ of these hold at once:

- 0 < left distance < 100 cm
- 0 < right distance < 100 cm
- 0 < center distance < 150 cm
- Turn count ≥ 12

...and they have to hold continuously for at least 1000 ms (`STOP_CONFIRM_MS`) — any break in that window resets the timer. This debounced, multi-sensor check exists specifically because a single bad LiDAR frame (signal dropout, reflection glitch) shouldn't be able to end the run early or falsely.

### The Steering PID Loop

The heading error driving the steering PID loop is wrapped and computed the same shortest-path way described above:

<p align="center"> <img src="images/esp_04_heading_error.png" width="260"></p> <p align="center"><img src="images/esp_05_heading_error_wrap.png" width="240"></p>

The proportional, integral, and derivative terms accumulate as:

<p align="center"><img src="images/esp_06_pid_accum_terms.png" width="320"></p>

before the final output is mapped onto the physical servo range, with a gentler ramp near the setpoint and a hard clamp at the mechanical steering limit so the loop can never command an angle the linkage can't physically reach:

<p align="center"><img src="images/esp_07_servo_angle_clamp.png" width="320"></p>

### Why We Built a Bluetooth Tuning Console

Getting the PID gains and other constants dialed in required a _lot_ of trial and error, and our first two approaches to iterating on those constants both turned out to be dead ends:

1. **Straight USB re-flashing.** Every single constant change — even something as small as nudging a servo bound — meant a full compile-and-flash cycle, roughly a minute each time. Across dozens of small parameter tweaks during a testing session, that added up to a huge amount of dead time.
2. **A Wi-Fi dashboard.** To skip re-flashing, we first tried an onboard Wi-Fi access point with a browser-based control page. This worked, but pulling in the full Wi-Fi stack alongside our IMU, servo, and Bluetooth drivers bloated the compiled binary badly enough to cause intermittent crashes and sensor communication delays — an unacceptable trade for a competition robot.
3. **Bluetooth Serial (what we shipped).** We settled on lightweight `key=value` text commands (e.g. `KP=1.2`, `TURN=150`) sent over Bluetooth Serial and parsed live in RAM — no re-flash, no heavy network stack, and updates apply instantly. This let us change a gain and watch Steve respond in real time without ever taking him off the table.

**Note:** this Bluetooth console was disabled once tuning was finished — a live parameter-override channel isn't something we want active during an official run.

|Command|Variable|Meaning|Default|
|---|---|---|---|
|`KP=val`|`STEERING_KP`|Proportional gain|1.2|
|`KI=val`|`STEERING_KI`|Integral gain|0.02|
|`KD=val`|`STEERING_KD`|Derivative gain|0.15|
|`CENTER=val`|`SERVO_CENTER`|Neutral steering angle|70|
|`DIFF=val`|`DIFF`|Max steering offset|25|
|`STRAIGHT=val`|`STRAIGHT_SPEED`|Cruise PWM|150|
|`TURN=val`|`TURN_SPEED`|Turning PWM|150|

---

## PID Control, Explained

The steering servo is the single actuator that both the wall-following logic (this round) and the vision-based pillar avoidance (Round 2) ultimately drive. In both cases, that servo is controlled by a **PID (Proportional–Integral–Derivative) controller** — a standard feedback algorithm that continuously nudges an actuator toward a target based on the current error.

```
u(t) = Kp·e(t)  +  Ki·∫e(t)dt  +  Kd·(de(t)/dt)
```

- **Proportional (Kp):** reacts to how far off the current heading is _right now_. Push this too high and the car overshoots and oscillates; too low and corrections feel sluggish.
- **Integral (Ki):** accumulates small, persistent errors over time — useful for correcting a subtle miscalibration (say, a `SERVO_CENTER` that's a couple degrees off) that a proportional term alone would never fully cancel out.
- **Derivative (Kd):** reacts to how _fast_ the error is changing, which damps the response and helps prevent overshoot as the heading approaches its target.

For Round 1, the error term feeding this loop is the shortest-path heading error described above — the difference between Steve's current EMA-smoothed compass heading and whichever cardinal direction he's currently targeting. The PID output becomes a servo offset from `SERVO_CENTER`, clamped to the mechanical `DIFF` limit so the loop physically can't command more steering angle than the linkage allows.

---

## Glossary

|Term|Meaning|
|---|---|
|**Ackermann steering**|Steering geometry where inner and outer front wheels turn at different angles so both roll around a shared center, avoiding tire scrub.|
|**EMA**|Exponential Moving Average — a recursive low-pass filter that weights recent samples more heavily.|
|**FSM**|Finite State Machine — a control architecture that moves between a fixed set of discrete states based on input conditions.|
|**IMU**|Inertial Measurement Unit — combines accelerometer, gyroscope, and often magnetometer data to estimate orientation.|
|**PID controller**|Proportional–Integral–Derivative feedback loop used to drive a measured value toward a target setpoint.|
|**PWM**|Pulse Width Modulation — encodes an analog-like signal (servo angle, motor speed) as a digital pulse train.|
|**UART**|Universal Asynchronous Receiver/Transmitter — the serial protocol used for onboard command links.|

---

_This document covers the Open Challenge (Round 1) build and control stack for Steve, our WRO 2026 Future Engineers entry. For the pillar-avoidance and parking systems used in the Obstacle Challenge, see the companion Round 2 document._
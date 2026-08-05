import cv2
import numpy as np
import serial
import time

# ==========================================
#   THIS SCRIPT DOES ALL THE THINKING.
#   The ESP32 is a dumb actuator: it reports raw sensor telemetry
#   ("T,<left>,<center>,<right>,<heading>") and executes drive
#   commands ("M,<servo_angle>,<motor_speed>" / "S" to stop).
#   Every decision -- when to reverse-arc, when to dodge a
#   pillar, when to stop -- happens here.
#
#   REVERSE-ARC NOTE: when a LiDAR spike (SPIKE_THRESHOLD) fires, the
#   L/R LiDARs are, by definition, in a moment where they can't be
#   trusted for fine steering. So the realignment maneuver steers on
#   BNO heading error only. The robot reverses while easing the servo
#   from full lock toward center as heading closes in on the new
#   cardinal target -- a diagonal arc that straightens out, landing the
#   robot centered and squared up in the new corridor, no separate
#   pivot-in-place step needed. LiDARs are still read and used purely
#   as a coarse "something is way too close, pause" safety floor.
# ==========================================

# ==========================================
#      ESP32 SERIAL CONNECTION
# ==========================================
ESP32_PORT = "/dev/ttyUSB0"
ESP32_BAUD = 115200

esp32 = None
try:
    esp32 = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=0.02)
    time.sleep(2)  # let the ESP32 finish its reset-on-serial-open cycle
    print(f"Connected to ESP32 on {ESP32_PORT}")
except serial.SerialException as e:
    print(f"Could not open {ESP32_PORT}: {e}")
    print("Continuing in vision-only mode (no commands will be sent).")

# ==========================================
#      STEERING / DRIVE CONFIG
#      (must stay inside the ESP32's SERVO_MIN/SERVO_MAX clamp)
# ==========================================
SERVO_CENTER = 87
DIFF = 25
SERVO_MAX_LEFT = SERVO_CENTER + DIFF
SERVO_MAX_RIGHT = SERVO_CENTER - DIFF
INVERT_STEERING = False  # flip if corrections steer the wrong way on your build

STRAIGHT_SPEED = 150
TURN_SPEED = 150
DODGE_SPEED = 150

# ==========================================
#      STRAIGHT-LINE HEADING-HOLD PID
# ==========================================
STEERING_KP = 1.2
STEERING_KI = 0.02
STEERING_KD = 0.15
HEADING_DEADBAND = 1.5        # degrees -- ignore jitter smaller than this
MAX_STEER_CORRECTION = 20     # clamp -- no single correction can swing the servo too hard
MAX_INTEGRAL = 50.0           # anti-windup clamp, degrees*seconds

# ==========================================
#      OBSTACLE TRIGGER CONFIG
# ==========================================
SPIKE_THRESHOLD = 100          # cm difference between L/R LiDAR that triggers a reverse-arc
CARDINAL_HEADINGS = [0.0, 90.0, 180.0, 270.0]

# ==========================================
#      REVERSE-ARC CONFIG
#      (replaces pivot-in-place turning -- steers on BNO heading only,
#       since a LiDAR spike is exactly the moment those readings can't
#       be trusted for fine correction)
# ==========================================
REVERSE_SPEED = -120            # constant reverse motor speed while arcing
SLOW_STEER_THRESHOLD = 50.0     # degrees remaining where steering eases back toward center
TURN_DONE_THRESHOLD = 6.0       # degrees remaining considered "arc complete"
REVERSE_TIMEOUT_S = 2.5         # hard safety cap -- no rear sensor exists, never trust heading alone forever
TURN_COOLDOWN_S = 1.0           # ignore obstacle checks for this long right after an arc completes
MIN_TURN_CLEARANCE = 25         # cm -- coarse emergency-pause floor only, NOT a steering input
MAX_TURNS = 12

# ==========================================
#      HEADING SMOOTHING
#      (complementary filter, wrap-safe around 0/360)
# ==========================================
HEADING_SMOOTH_ALPHA = 0.2

# ==========================================
#      STOP CONDITION
#      (L & R both close, front close, held continuously before acting)
# ==========================================
STOP_LEFT_RIGHT_DIST = 100
STOP_CENTER_DIST = 150
STOP_CONFIRM_S = 1.0

# ==========================================
#      VISION / DODGE CONFIG
# ==========================================
RESUME_AFTER_MISSES = 5   # consecutive empty frames before resuming normal driving
DODGE_CONFIDENCE_MIN = 60


# ==========================================
#      SERIAL HELPERS
# ==========================================
def send_move_command(servo_angle, motor_speed):
    if esp32 is None:
        return
    try:
        esp32.write(f"M,{int(servo_angle)},{int(motor_speed)}\n".encode())
    except serial.SerialException as e:
        print(f"Serial write failed: {e}")


def send_stop_command():
    if esp32 is None:
        return
    try:
        esp32.write(b"S\n")
    except serial.SerialException as e:
        print(f"Serial write failed: {e}")


def drain_telemetry(brain):
    """Read every buffered telemetry line from the ESP32, keeping only the latest."""
    if esp32 is None:
        return
    while esp32.in_waiting:
        try:
            raw = esp32.readline().decode(errors="ignore").strip()
        except serial.SerialException:
            break
        if raw:
            brain.ingest_telemetry_line(raw)


# ==========================================
#      VISION: PILLAR DETECTION
# ==========================================
def white_balance_gray_world(img):
    result = img.astype(np.float32)
    avg_b = np.mean(result[:, :, 0])
    avg_g = np.mean(result[:, :, 1])
    avg_r = np.mean(result[:, :, 2])
    avg_gray = (avg_b + avg_g + avg_r) / 3.0
    result[:, :, 0] *= (avg_gray / max(avg_b, 1e-6))
    result[:, :, 1] *= (avg_gray / max(avg_g, 1e-6))
    result[:, :, 2] *= (avg_gray / max(avg_r, 1e-6))
    return np.clip(result, 0, 255).astype(np.uint8)


def is_pillar_shaped(contour, min_aspect_ratio=1.2, min_solidity=0.85):
    """Rejects noisy blobs that pass the color mask but aren't pillar-shaped."""
    x, y, w, h = cv2.boundingRect(contour)
    if w == 0:
        return False
    aspect_ratio = h / float(w)
    contour_area = cv2.contourArea(contour)
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    solidity = contour_area / float(hull_area) if hull_area > 0 else 0
    return aspect_ratio >= min_aspect_ratio and solidity >= min_solidity


def compute_confidence(contour, hsv_frame, low_arr, high_arr, ideal_aspect_ratio=2.0):
    """
    Composite 0-100% confidence score combining:
    - Solidity (40%): how solid/convex the blob is vs. ragged noise
    - Aspect ratio fit (30%): how close to an ideal elongated pillar shape
    - Color centrality (30%): how deep into the S/V threshold range the
      object's pixels sit, vs. just barely scraping past the edge
    """
    x, y, w, h = cv2.boundingRect(contour)
    aspect_ratio = h / float(w) if w > 0 else 0
    aspect_score = np.clip((aspect_ratio - 1.0) / (ideal_aspect_ratio - 1.0), 0, 1)

    contour_area = cv2.contourArea(contour)
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    solidity = contour_area / float(hull_area) if hull_area > 0 else 0

    obj_mask = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
    cv2.drawContours(obj_mask, [contour], -1, 255, -1)
    mean_h, mean_s, mean_v = cv2.mean(hsv_frame, mask=obj_mask)[:3]

    s_center = (int(low_arr[1]) + int(high_arr[1])) / 2.0
    v_center = (int(low_arr[2]) + int(high_arr[2])) / 2.0
    s_half_range = (int(high_arr[1]) - int(low_arr[1])) / 2.0 + 1e-6
    v_half_range = (int(high_arr[2]) - int(low_arr[2])) / 2.0 + 1e-6
    s_score = 1 - min(abs(mean_s - s_center) / s_half_range, 1.0)
    v_score = 1 - min(abs(mean_v - v_center) / v_half_range, 1.0)
    color_score = (s_score + v_score) / 2.0

    confidence = (0.4 * solidity + 0.3 * aspect_score + 0.3 * color_score) * 100
    return int(round(confidence))


def compute_dodge_servo_angle(color, box_x, box_w, box_h, frame_w, frame_h):
    """
    - RED: keep to the RIGHT side of the lane -> steer right, pillar passes on the left.
    - GREEN: keep to the LEFT side of the lane -> steer left, pillar passes on the right.
    Offset scales with how close the object looks (taller box = closer).
    """
    closeness = min(box_h / (frame_h * 0.6), 1.0)  # tune the 0.6 divisor to your camera/track distance
    max_offset = SERVO_MAX_LEFT - SERVO_CENTER  # assumes symmetric limits
    offset = int(max_offset * closeness)

    if color == "red":
        servo_angle = SERVO_CENTER - offset
    else:  # green
        servo_angle = SERVO_CENTER + offset

    return max(SERVO_MAX_RIGHT, min(SERVO_MAX_LEFT, servo_angle))


def detect_pillars(frame, clahe):
    """Runs the full color-detection pipeline on one frame and returns the
    best (confidence, color, x, y, w, h) detection, or None. Also draws
    boxes/labels onto `frame` in place."""
    balanced_frame = white_balance_gray_world(frame)
    blurred_frame = cv2.GaussianBlur(balanced_frame, (5, 5), 0)
    hsv_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)

    h_ch, s_ch, v_ch = cv2.split(hsv_frame)
    v_ch = clahe.apply(v_ch)
    hsv_frame = cv2.merge([h_ch, s_ch, v_ch])

    low_red1 = np.array([0, 140, 100])
    high_red1 = np.array([8, 255, 255])
    low_red2 = np.array([172, 140, 100])
    high_red2 = np.array([180, 255, 255])

    # Calibrated on the white-balance + CLAHE corrected feed --
    # don't reuse numbers found on the raw uncorrected feed.
    low_green = np.array([57, 0, 0])
    high_green = np.array([79, 255, 255])

    mask_red = cv2.inRange(hsv_frame, low_red1, high_red1) + cv2.inRange(hsv_frame, low_red2, high_red2)
    mask_green = cv2.inRange(hsv_frame, low_green, high_green)

    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)

    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    best_detection = None  # (confidence, color, x, y, w, h) -- highest confidence wins

    for contour in contours_red:
        if cv2.contourArea(contour) > 1500 and is_pillar_shaped(contour):
            conf = compute_confidence(contour, hsv_frame, low_red1, high_red1)
            if conf < DODGE_CONFIDENCE_MIN:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, f"RED PILLAR {conf}%", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if best_detection is None or conf > best_detection[0]:
                best_detection = (conf, "red", x, y, w, h)

    for contour in contours_green:
        if cv2.contourArea(contour) > 1500 and is_pillar_shaped(contour):
            conf = compute_confidence(contour, hsv_frame, low_green, high_green)
            if conf < DODGE_CONFIDENCE_MIN:
                continue
            hull = cv2.convexHull(contour)
            x, y, w, h = cv2.boundingRect(hull)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"GREEN PILLAR {conf}%", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if best_detection is None or conf > best_detection[0]:
                best_detection = (conf, "green", x, y, w, h)

    return best_detection


# ==========================================
#      ROBOT BRAIN -- the full state machine
# ==========================================
class RobotBrain:
    def __init__(self):
        self.state = "DRIVING_STRAIGHT"  # DRIVING_STRAIGHT | REVERSE_ARC | DODGING | STOPPED

        # Latest raw sensor telemetry from the ESP32
        self.left_dist = -1
        self.center_dist = -1
        self.right_dist = -1
        self.raw_heading = 0.0

        # Smoothed heading (complementary filter, wrap-safe)
        self.smoothed_heading = 0.0
        self.heading_initialized = False

        # Straight-line PID state
        self.straight_target_heading = 0.0
        self.last_heading_error = 0.0
        self.integral_error = 0.0
        self.last_heading_time = time.time()

        # Reverse-arc state
        self.turn_target_heading = 0.0
        self.is_turning_left = False
        self.has_turned_once = False    # becomes True permanently on the first arc
        self.locked_direction_left = False
        self.total_turns = 0
        self.turn_cooldown_until = 0.0
        self.reverse_start_time = 0.0

        # Stop-condition confirmation
        self.stop_condition_active = False
        self.stop_condition_start = 0.0

        # Dodge state
        self.consecutive_misses = 0
        self.last_camera_status = "NO DETECTION"

        self.final_servo_angle = SERVO_CENTER
        self.final_motor_speed = 0

    # ---------------- Telemetry ingestion ----------------
    def ingest_telemetry_line(self, line):
        # Format: T,<left>,<center>,<right>,<heading>
        parts = line.split(",")
        if len(parts) != 5 or parts[0] != "T":
            return
        try:
            self.left_dist = int(parts[1])
            self.center_dist = int(parts[2])
            self.right_dist = int(parts[3])
            self.raw_heading = float(parts[4])
        except ValueError:
            return
        self._update_smoothed_heading()

    def _update_smoothed_heading(self):
        if not self.heading_initialized:
            self.smoothed_heading = self.raw_heading
            self.heading_initialized = True
            self.straight_target_heading = self.raw_heading
            return
        diff = self.raw_heading - self.smoothed_heading
        if diff > 180.0:
            diff -= 360.0
        if diff < -180.0:
            diff += 360.0
        self.smoothed_heading = (self.smoothed_heading + HEADING_SMOOTH_ALPHA * diff) % 360.0

    # ---------------- Cardinal snap ----------------
    @staticmethod
    def snap_to_cardinal(angle):
        """Forces a target onto the nearest of 0/90/180/270 so sensor
        drift can never leave the robot aiming at something like 253 degrees."""
        angle = angle % 360.0
        best = CARDINAL_HEADINGS[0]
        best_diff = 999.0
        for c in CARDINAL_HEADINGS:
            diff = abs(angle - c)
            diff = min(diff, 360.0 - diff)
            if diff < best_diff:
                best_diff = diff
                best = c
        return best

    def compute_turn_target(self, current_heading, turning_left):
        raw = (current_heading - 90.0) if turning_left else (current_heading + 90.0)
        raw %= 360.0
        return self.snap_to_cardinal(raw)

    # ---------------- Obstacle -> reverse-arc trigger ----------------
    def check_obstacles(self):
        if time.time() < self.turn_cooldown_until:
            return
        if self.left_dist == -1 or self.right_dist == -1:
            return

        difference = self.left_dist - self.right_dist

        # Once a direction is locked, only ever evaluate that one branch.
        if self.has_turned_once:
            if self.locked_direction_left and difference > SPIKE_THRESHOLD:
                self._start_reverse_arc(turning_left=True)
            elif (not self.locked_direction_left) and difference < -SPIKE_THRESHOLD:
                self._start_reverse_arc(turning_left=False)
            return

        if difference > SPIKE_THRESHOLD:
            self.has_turned_once = True
            self.locked_direction_left = True
            print("!!! LAYOUT INITIALIZED: PERMANENT LEFT TURN ONLY LOCK ACTIVATED !!!")
            self._start_reverse_arc(turning_left=True)
        elif difference < -SPIKE_THRESHOLD:
            self.has_turned_once = True
            self.locked_direction_left = False
            print("!!! LAYOUT INITIALIZED: PERMANENT RIGHT TURN ONLY LOCK ACTIVATED !!!")
            self._start_reverse_arc(turning_left=False)

    def _start_reverse_arc(self, turning_left):
        self.is_turning_left = turning_left
        # Target computed off heading at the moment of trigger -- LiDARs are
        # spiking right now, so heading is the only trustworthy reference.
        self.turn_target_heading = self.compute_turn_target(self.smoothed_heading, turning_left)
        self.reverse_start_time = time.time()
        self.state = "REVERSE_ARC"

    # ---------------- Straight-line PID ----------------
    def drive_straight_command(self):
        now = time.time()
        raw_error = self.straight_target_heading - self.smoothed_heading
        if raw_error > 180.0:
            raw_error -= 360.0
        if raw_error < -180.0:
            raw_error += 360.0

        dt = max(now - self.last_heading_time, 0.001)
        error_rate = (raw_error - self.last_heading_error) / dt

        p_term_input = 0.0 if abs(raw_error) < HEADING_DEADBAND else raw_error

        # Only accumulate the integral on a real (non-deadbanded) error, so it
        # doesn't slowly drift from sensor noise while the robot is on target.
        if abs(raw_error) >= HEADING_DEADBAND:
            self.integral_error += raw_error * dt
            self.integral_error = max(-MAX_INTEGRAL, min(MAX_INTEGRAL, self.integral_error))

        correction = (p_term_input * STEERING_KP +
                      self.integral_error * STEERING_KI +
                      error_rate * STEERING_KD)
        correction = int(round(max(-MAX_STEER_CORRECTION, min(MAX_STEER_CORRECTION, correction))))

        self.last_heading_error = raw_error
        self.last_heading_time = now

        if INVERT_STEERING:
            angle = SERVO_CENTER + correction
        else:
            angle = SERVO_CENTER - correction
        angle = max(SERVO_MAX_RIGHT, min(SERVO_MAX_LEFT, angle))

        return angle, STRAIGHT_SPEED

    # ---------------- Reverse-arc ----------------
    def reverse_arc_command(self):
        """Reverses while steering purely on BNO heading error toward the
        target cardinal heading. Full servo lock while heading error is
        large, easing back toward center as it closes in -- producing a
        diagonal-then-straight arc that lands centered and squared up,
        with no separate pivot-in-place step. LiDARs are NOT used for
        steering here (they're the thing that's spiking); they're only
        checked as a coarse emergency-pause floor below."""
        angle_difference = self.smoothed_heading - self.turn_target_heading
        if angle_difference > 180.0:
            angle_difference -= 360.0
        if angle_difference < -180.0:
            angle_difference += 360.0
        remaining = abs(angle_difference)

        left_extreme = SERVO_MAX_RIGHT if INVERT_STEERING else SERVO_MAX_LEFT
        right_extreme = SERVO_MAX_LEFT if INVERT_STEERING else SERVO_MAX_RIGHT

        if remaining < SLOW_STEER_THRESHOLD:
            progress = remaining / SLOW_STEER_THRESHOLD
            if self.is_turning_left:
                max_offset = left_extreme - SERVO_CENTER
                angle = SERVO_CENTER + int(round(max_offset * progress))
            else:
                max_offset = SERVO_CENTER - right_extreme
                angle = SERVO_CENTER - int(round(max_offset * progress))
        else:
            angle = left_extreme if self.is_turning_left else right_extreme

        speed = REVERSE_SPEED

        # Coarse emergency floor only -- spiking LiDARs can't be trusted for
        # fine correction, but a sudden very-small reading is still worth
        # pausing the reverse for a frame rather than trusting heading blindly.
        closest = min(
            d for d in (self.left_dist, self.right_dist) if d > 0
        ) if (self.left_dist > 0 or self.right_dist > 0) else 9999
        if closest < MIN_TURN_CLEARANCE:
            speed = 0

        timed_out = (time.time() - self.reverse_start_time) > REVERSE_TIMEOUT_S

        if remaining < TURN_DONE_THRESHOLD or timed_out:
            self.total_turns += 1
            self.straight_target_heading = self.turn_target_heading
            self.state = "DRIVING_STRAIGHT"
            self.turn_cooldown_until = time.time() + TURN_COOLDOWN_S
            self.integral_error = 0.0
            angle = SERVO_CENTER
            speed = 0

        return angle, speed

    # ---------------- Stop condition ----------------
    def check_stop_condition(self):
        condition_met = (
            0 < self.left_dist < STOP_LEFT_RIGHT_DIST and
            0 < self.right_dist < STOP_LEFT_RIGHT_DIST and
            0 < self.center_dist < STOP_CENTER_DIST and
            self.total_turns >= MAX_TURNS
        )
        if condition_met:
            if not self.stop_condition_active:
                self.stop_condition_active = True
                self.stop_condition_start = time.time()
            elif time.time() - self.stop_condition_start >= STOP_CONFIRM_S:
                self.state = "STOPPED"
        else:
            self.stop_condition_active = False

    # ---------------- Top-level decision per frame ----------------
    def step(self, best_detection, frame_w, frame_h):
        """Runs one control cycle and returns (servo_angle, motor_speed)."""
        if self.state == "STOPPED":
            self.final_servo_angle, self.final_motor_speed = SERVO_CENTER, 0
            return self.final_servo_angle, self.final_motor_speed

        self.check_stop_condition()
        if self.state == "STOPPED":
            self.final_servo_angle, self.final_motor_speed = SERVO_CENTER, 0
            return self.final_servo_angle, self.final_motor_speed

        # Vision may only take over while driving straight (or while already
        # dodging) -- never interrupt an active reverse-arc to dodge.
        if best_detection is not None and self.state in ("DRIVING_STRAIGHT", "DODGING"):
            conf, color, x, y, w, h = best_detection
            angle = compute_dodge_servo_angle(color, x, w, h, frame_w, frame_h)
            self.state = "DODGING"
            self.consecutive_misses = 0
            self.last_camera_status = f"{color.upper()} DETECTED ({conf}%)"
            self.final_servo_angle, self.final_motor_speed = angle, DODGE_SPEED
            return self.final_servo_angle, self.final_motor_speed

        if self.state == "DODGING":
            self.consecutive_misses += 1
            if self.consecutive_misses >= RESUME_AFTER_MISSES:
                self.state = "DRIVING_STRAIGHT"
                self.last_camera_status = "NO DETECTION"
                # Re-anchor heading so the robot doesn't try to correct back
                # to whatever direction it was facing before the dodge.
                self.straight_target_heading = self.smoothed_heading
            else:
                self.final_servo_angle, self.final_motor_speed = SERVO_CENTER, DODGE_SPEED
                return self.final_servo_angle, self.final_motor_speed

        if self.state == "DRIVING_STRAIGHT":
            self.check_obstacles()
            if self.state == "REVERSE_ARC":
                angle, speed = self.reverse_arc_command()
            else:
                angle, speed = self.drive_straight_command()
        elif self.state == "REVERSE_ARC":
            angle, speed = self.reverse_arc_command()
        else:
            angle, speed = SERVO_CENTER, 0

        self.final_servo_angle, self.final_motor_speed = angle, speed
        return angle, speed

    # ---------------- Debug ----------------
    def telemetry_line(self):
        """Human-readable status line, printed from the Pi since it's the
        only side that knows the full state."""
        lock = "NONE"
        if self.has_turned_once:
            lock = "LEFT_ONLY" if self.locked_direction_left else "RIGHT_ONLY"

        parts = [
            f"MODE: {self.state:16s}",
            f"| L: {self.left_dist}cm",
            f"| C: {self.center_dist}cm",
            f"| R: {self.right_dist}cm",
            f"| Turns: {self.total_turns}",
            f"| CAM: {self.last_camera_status}",
            f"| LOCK: {lock}",
        ]

        if self.state == "DRIVING_STRAIGHT":
            raw_error = self.straight_target_heading - self.smoothed_heading
            if raw_error > 180.0:
                raw_error -= 360.0
            if raw_error < -180.0:
                raw_error += 360.0
            parts.append(f"| Target: {self.straight_target_heading:.1f}\u00b0")
            parts.append(f"| Current: {self.smoothed_heading:.1f}\u00b0")
            parts.append(f"| Error: {raw_error:.1f}\u00b0")
        elif self.state == "REVERSE_ARC":
            angle_difference = self.smoothed_heading - self.turn_target_heading
            if angle_difference > 180.0:
                angle_difference -= 360.0
            if angle_difference < -180.0:
                angle_difference += 360.0
            parts.append(f"| Target: {self.turn_target_heading:.1f}\u00b0")
            parts.append(f"| Current: {self.smoothed_heading:.1f}\u00b0")
            parts.append(f"| Delta: {abs(angle_difference):.1f}\u00b0")
        elif self.state == "DODGING":
            parts.append(f"| Current: {self.smoothed_heading:.1f}\u00b0")

        parts.append(f"| Servo Angle: {self.final_servo_angle}\u00b0")
        parts.append(f"| Motor: {self.final_motor_speed}")

        return " ".join(parts)


# ==========================================
#             MAIN LOOP
# ==========================================
def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    brain = RobotBrain()

    print("Press 'q' to close the camera window.")
    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        drain_telemetry(brain)

        best_detection = detect_pillars(frame, clahe)
        frame_h, frame_w = frame.shape[:2]

        servo_angle, motor_speed = brain.step(best_detection, frame_w, frame_h)
        send_move_command(servo_angle, motor_speed)

        cv2.putText(frame, f"{brain.state} | servo={servo_angle} speed={motor_speed}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow("Live Object Detection", frame)

        frame_count += 1
        print(brain.telemetry_line())

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    send_stop_command()
    cap.release()
    cv2.destroyAllWindows()
    if esp32 is not None:
        esp32.close()


if __name__ == "__main__":
    main()
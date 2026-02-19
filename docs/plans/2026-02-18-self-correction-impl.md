# Self-Correction Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add hybrid multi-signal self-correction to the Freenove car so it compensates for wheel drift and servo misalignment using sensor fusion of visual odometry, ultrasonic reference, and dead reckoning.

**Architecture:** A new `calibration.py` module contains five classes (VisualOdometry, UltrasonicReference, MotorBiasTable, ServoAlignment, SensorFusion). These integrate into the existing `AutonomyModule.execute_step()` cycle — pre/post photos + 3-point ultrasonic sweeps feed into fusion, which corrects the pose estimate and updates a persistent motor bias table. The system degrades gracefully: without OpenCV, visual odometry returns confidence=0; without nearby walls, ultrasonic does the same.

**Tech Stack:** Python 3 stdlib + OpenCV (optional, `cv2`). No new pip dependencies required on the Pi (OpenCV is pre-installed on Raspberry Pi OS).

**Design doc:** `docs/plans/2026-02-18-self-correction-design.md`

---

## Prerequisites

Before starting: commit existing uncommitted changes (battery LED monitor, autonomy endpoints, speed defaults) from the previous session. These are staged modifications to `server.py`, `autonomy.py`, and `API_SPEC.md`.

---

### Task 1: Create calibration.py — VisualOdometry

**Files:**
- Create: `calibration.py`

**What this does:** Compares before/after camera images to estimate how the car actually moved. Uses OpenCV sparse optical flow (Shi-Tomasi corner detection → Lucas-Kanade tracking). Returns pixel displacement + rotation estimate with a confidence score.

**Step 1: Create calibration.py with OpenCV guard and VisualOdometry class**

```python
"""Self-correction calibration module for the Freenove 4WD Smart Car.

Provides sensor fusion, visual odometry, ultrasonic reference correction,
motor bias learning, and servo alignment detection. All components are
optional and degrade gracefully.

Dependencies: OpenCV (optional — visual odometry returns confidence=0 without it)
"""

import json
import math
import os
import time

try:
    import cv2
    import numpy as np
    _cv2_available = True
except ImportError:
    _cv2_available = False


class VisualOdometry:
    """Estimate car displacement by comparing before/after camera images.

    Uses sparse optical flow (Shi-Tomasi corners + Lucas-Kanade tracking)
    to compute translation and rotation between two frames.

    Without OpenCV installed, all methods return zero displacement with
    confidence=0 — no errors, no special handling needed by callers.
    """

    # Shi-Tomasi corner detection params
    FEATURE_PARAMS = dict(
        maxCorners=50,
        qualityLevel=0.3,
        minDistance=7,
        blockSize=7,
    )

    # Lucas-Kanade optical flow params
    LK_PARAMS = dict(
        winSize=(15, 15),
        maxLevel=2,
        criteria=(3, 10, 0.03),  # cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT
    )

    # Calibration: pixels per cm of car movement at 1 meter distance
    # Must be tuned empirically during startup calibration
    PX_PER_CM_AT_1M = 3.0

    def __init__(self):
        self._before_frame = None
        self._before_gray = None

    def capture_before(self, jpeg_bytes):
        """Store the 'before' frame for later comparison.

        Args:
            jpeg_bytes: Raw JPEG bytes from rpicam-still.
        """
        if not _cv2_available or jpeg_bytes is None:
            self._before_frame = None
            self._before_gray = None
            return

        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            self._before_frame = None
            self._before_gray = None
            return

        self._before_frame = frame
        self._before_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def compute_displacement(self, after_jpeg_bytes, distance_cm=100.0):
        """Compare after image to stored before image.

        Args:
            after_jpeg_bytes: Raw JPEG bytes of the 'after' frame.
            distance_cm: Ultrasonic distance for depth scaling.

        Returns:
            dict with keys:
                dx_cm: Forward displacement (positive = forward).
                dy_cm: Lateral displacement (positive = left).
                d_theta_deg: Rotation (positive = CCW).
                confidence: 0.0-1.0 reliability score.
        """
        zero = {'dx_cm': 0.0, 'dy_cm': 0.0, 'd_theta_deg': 0.0, 'confidence': 0.0}

        if not _cv2_available or self._before_gray is None or after_jpeg_bytes is None:
            return zero

        arr = np.frombuffer(after_jpeg_bytes, dtype=np.uint8)
        after_frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if after_frame is None:
            return zero

        after_gray = cv2.cvtColor(after_frame, cv2.COLOR_BGR2GRAY)

        # Detect features in the before image
        corners = cv2.goodFeaturesToTrack(self._before_gray, **self.FEATURE_PARAMS)
        if corners is None or len(corners) < 5:
            return zero

        # Track features in the after image
        tracked, status, err = cv2.calcOpticalFlowPyrLK(
            self._before_gray, after_gray, corners, None, **self.LK_PARAMS
        )

        if tracked is None:
            return zero

        # Filter: keep only successfully tracked points
        good_mask = status.ravel() == 1
        before_pts = corners[good_mask]
        after_pts = tracked[good_mask]

        if len(before_pts) < 5:
            return zero

        # Compute displacements
        displacements = after_pts - before_pts  # shape: (N, 1, 2) or (N, 2)
        if displacements.ndim == 3:
            displacements = displacements.reshape(-1, 2)

        # Outlier rejection: remove points > 2 std devs from median
        dx_all = displacements[:, 0]
        dy_all = displacements[:, 1]

        med_dx, med_dy = np.median(dx_all), np.median(dy_all)
        std_dx, std_dy = np.std(dx_all), np.std(dy_all)

        if std_dx < 0.01:
            std_dx = 1.0
        if std_dy < 0.01:
            std_dy = 1.0

        inlier_mask = (
            (np.abs(dx_all - med_dx) < 2 * std_dx) &
            (np.abs(dy_all - med_dy) < 2 * std_dy)
        )
        dx_inliers = dx_all[inlier_mask]
        dy_inliers = dy_all[inlier_mask]

        n_inliers = len(dx_inliers)
        if n_inliers < 5:
            return zero

        # Mean pixel displacement
        mean_dx_px = float(np.mean(dx_inliers))
        mean_dy_px = float(np.mean(dy_inliers))

        # Estimate rotation from the flow field
        # Use before/after point pairs relative to image center
        h, w = self._before_gray.shape
        cx, cy = w / 2.0, h / 2.0

        before_centered = before_pts.reshape(-1, 2)[inlier_mask] - np.array([cx, cy])
        after_centered = after_pts.reshape(-1, 2)[inlier_mask] - np.array([cx, cy])

        # Rotation = average angular change of vectors from center
        angles_before = np.arctan2(before_centered[:, 1], before_centered[:, 0])
        angles_after = np.arctan2(after_centered[:, 1], after_centered[:, 0])
        d_angles = angles_after - angles_before
        # Wrap to [-pi, pi]
        d_angles = (d_angles + np.pi) % (2 * np.pi) - np.pi
        mean_rotation_rad = float(np.median(d_angles))

        # Convert pixel displacement to cm using depth scaling
        # Closer objects move more pixels, so scale by distance
        depth_scale = max(distance_cm, 10.0) / 100.0
        px_per_cm = self.PX_PER_CM_AT_1M / depth_scale

        if px_per_cm < 0.01:
            px_per_cm = 0.01

        # Camera axes: image x = car's lateral, image y = car's forward (inverted)
        # Positive image dx = feature moved right = car moved left
        # Positive image dy = feature moved down = car moved forward
        dx_cm = -mean_dy_px / px_per_cm  # forward/backward
        dy_cm = -mean_dx_px / px_per_cm  # lateral (positive = left)
        d_theta_deg = math.degrees(mean_rotation_rad)

        # Confidence based on number of inliers and reprojection consistency
        confidence = min(1.0, n_inliers / 30.0)
        # Reduce confidence if displacement std is high (noisy tracking)
        flow_std = float(np.std(dx_inliers) + np.std(dy_inliers))
        if flow_std > 20:
            confidence *= 0.5

        return {
            'dx_cm': round(dx_cm, 2),
            'dy_cm': round(dy_cm, 2),
            'd_theta_deg': round(d_theta_deg, 2),
            'confidence': round(confidence, 3),
        }
```

**Step 2: Verify OpenCV is available on the Pi**

```bash
ssh freenove-car "python3 -c 'import cv2; print(cv2.__version__)'"
```

Expected: prints a version string (e.g., `4.6.0`). If it fails, visual odometry will gracefully return confidence=0.

**Step 3: Commit**

```bash
git add calibration.py
git commit -m "feat: add VisualOdometry class with OpenCV optical flow"
```

---

### Task 2: Add UltrasonicReference to calibration.py

**Files:**
- Modify: `calibration.py`

**What this does:** Takes 3-point ultrasonic sweeps (left 45°, center 90°, right 135°) before and after movement. Compares the distance changes to estimate lateral drift and rotation. Provides absolute position reference when walls are nearby.

**Step 1: Add UltrasonicReference class**

```python
class UltrasonicReference:
    """Estimate displacement from 3-point ultrasonic sweep before/after movement.

    Sweeps at 45° (left), 90° (center), 135° (right) relative to pan servo.
    Compares pre/post readings to detect forward motion, lateral drift, and rotation.

    Only reliable when walls are within ~150cm. Returns confidence=0 in open space.
    """

    SWEEP_ANGLES = (45, 90, 135)  # left, center, right
    SETTLE_TIME = 0.15  # seconds for servo to reach position
    MAX_RELIABLE_DISTANCE = 150  # cm — beyond this, confidence drops

    def __init__(self, get_ultrasonic_fn, get_servo_fn, move_servo_smooth_fn):
        self._get_ultrasonic = get_ultrasonic_fn
        self._get_servo = get_servo_fn
        self._move_servo_smooth = move_servo_smooth_fn

        self._before_readings = None
        self._after_readings = None

    def sweep(self):
        """Perform a 3-point ultrasonic sweep at SWEEP_ANGLES.

        Returns:
            dict mapping angle -> distance_cm, e.g. {45: 120.3, 90: 85.1, 135: 110.7}
        """
        readings = {}
        for angle in self.SWEEP_ANGLES:
            self._move_servo_smooth(1, angle, 200)  # pan channel = 1
            time.sleep(self.SETTLE_TIME)
            try:
                dist = self._get_ultrasonic().get_distance()
                readings[angle] = dist
            except Exception:
                readings[angle] = -1

        # Return pan to center
        self._move_servo_smooth(1, 90, 200)
        return readings

    def capture_before(self):
        """Store pre-movement 3-point sweep."""
        self._before_readings = self.sweep()

    def capture_after(self):
        """Store post-movement 3-point sweep."""
        self._after_readings = self.sweep()

    def compute_correction(self):
        """Compare before/after sweeps to estimate actual displacement.

        Returns:
            dict with keys:
                dx_cm: Forward displacement from ultrasonic delta.
                dy_cm: Lateral drift estimate.
                d_theta_deg: Rotation estimate (crude, use with visual odometry).
                confidence: 0.0-1.0 reliability score.
                readings_before: Raw before readings.
                readings_after: Raw after readings.
        """
        zero = {
            'dx_cm': 0.0, 'dy_cm': 0.0, 'd_theta_deg': 0.0, 'confidence': 0.0,
            'readings_before': self._before_readings,
            'readings_after': self._after_readings,
        }

        if self._before_readings is None or self._after_readings is None:
            return zero

        # Check all readings are valid
        for angle in self.SWEEP_ANGLES:
            if (self._before_readings.get(angle, -1) <= 0 or
                    self._after_readings.get(angle, -1) <= 0):
                return zero

        b_left = self._before_readings[45]
        b_center = self._before_readings[90]
        b_right = self._before_readings[135]
        a_left = self._after_readings[45]
        a_center = self._after_readings[90]
        a_right = self._after_readings[135]

        # Deltas (negative = got closer)
        d_left = a_left - b_left
        d_center = a_center - b_center
        d_right = a_right - b_right

        # Forward motion: primarily from center reading change
        dx_cm = -d_center

        # Lateral drift: asymmetry between left and right changes
        # If left dropped more than right → drifted left
        lateral_asymmetry = d_right - d_left  # positive = drifted left
        dy_cm = lateral_asymmetry * 0.5  # rough geometric scaling

        # Rotation: if left got closer and right got farther (or vice versa)
        # while center stayed roughly the same
        rotation_signal = (d_right - d_left) * 0.3  # degrees, very rough
        d_theta_deg = rotation_signal

        # Confidence based on distance — close walls give better data
        max_dist = max(b_left, b_center, b_right, a_left, a_center, a_right)
        if max_dist > self.MAX_RELIABLE_DISTANCE:
            confidence = max(0.0, 1.0 - (max_dist - self.MAX_RELIABLE_DISTANCE) / 100.0)
        else:
            confidence = 1.0

        # Reduce confidence if readings are noisy (large inconsistencies)
        spread = max(abs(d_left), abs(d_center), abs(d_right))
        if spread > 50:
            confidence *= 0.5

        return {
            'dx_cm': round(dx_cm, 2),
            'dy_cm': round(dy_cm, 2),
            'd_theta_deg': round(d_theta_deg, 2),
            'confidence': round(min(1.0, confidence), 3),
            'readings_before': self._before_readings,
            'readings_after': self._after_readings,
        }
```

**Step 2: Commit**

```bash
git add calibration.py
git commit -m "feat: add UltrasonicReference 3-point sweep correction"
```

---

### Task 3: Add MotorBiasTable to calibration.py

**Files:**
- Modify: `calibration.py`

**What this does:** Learns per-command correction factors (lateral drift, rotation bias, speed scaling) using exponential moving average. Persists to `~/freenove-car-api/motor_bias.json` so corrections survive reboots. Applies corrections to motor PWM values before movement.

**Step 1: Add MotorBiasTable class**

```python
class MotorBiasTable:
    """Per-command motor correction table that learns from sensor fusion.

    Tracks lateral drift, rotation bias, and speed scaling factor per command.
    Uses exponential moving average for smooth adaptation. Persists to JSON.

    Corrections are applied by adjusting per-wheel motor speeds to compensate
    for observed drift and rotation errors.
    """

    DEFAULT_BIAS = {'lateral': 0.0, 'rotation': 0.0, 'speed_scale': 1.0}

    # Guardrails — max correction values
    MAX_WHEEL_ADJUST_PCT = 0.15   # +/- 15% of commanded speed
    MAX_ROTATION_BIAS = 10.0      # deg/sec
    SPEED_SCALE_RANGE = (0.5, 1.5)

    COMMANDS = [
        'forward', 'backward', 'strafe_left', 'strafe_right',
        'rotate_cw', 'rotate_ccw', 'diagonal_fl', 'diagonal_fr',
        'diagonal_bl', 'diagonal_br', 'left', 'right',
    ]

    def __init__(self, persist_path=None, alpha=0.3):
        self.alpha = alpha  # EMA learning rate
        self._stale_alpha = 0.6  # faster learning when battery drops

        if persist_path is None:
            persist_path = os.path.expanduser('~/freenove-car-api/motor_bias.json')
        self._persist_path = persist_path

        self.table = {}
        self._calibration_voltage = None
        self.stale = False

        self._load()

    def _load(self):
        """Load bias table from disk if it exists."""
        try:
            with open(self._persist_path, 'r') as f:
                data = json.load(f)
                self.table = data.get('biases', {})
                self._calibration_voltage = data.get('calibration_voltage')
        except (FileNotFoundError, json.JSONDecodeError, KeyError):
            self.table = {}

    def save(self):
        """Persist current bias table to disk."""
        data = {
            'biases': self.table,
            'calibration_voltage': self._calibration_voltage,
            'updated_at': time.time(),
        }
        try:
            os.makedirs(os.path.dirname(self._persist_path), exist_ok=True)
            with open(self._persist_path, 'w') as f:
                json.dump(data, f, indent=2)
        except OSError:
            pass

    def get_bias(self, command):
        """Get the current bias entry for a command."""
        if command not in self.table:
            self.table[command] = dict(self.DEFAULT_BIAS)
        return self.table[command]

    def update(self, command, observed_lateral, observed_rotation, expected_speed,
               actual_speed, duration):
        """Update bias table with a new observation.

        Args:
            command: Movement command name.
            observed_lateral: Lateral drift in cm/sec (positive = left).
            observed_rotation: Rotation in deg/sec (positive = CCW).
            expected_speed: Commanded PWM speed.
            actual_speed: Estimated actual speed from fusion (cm/sec).
            duration: How long the movement lasted.
        """
        if command not in self.COMMANDS or duration < 0.1:
            return

        bias = self.get_bias(command)
        alpha = self._stale_alpha if self.stale else self.alpha

        # Update lateral drift (cm/sec)
        bias['lateral'] = (1 - alpha) * bias['lateral'] + alpha * observed_lateral

        # Update rotation bias (deg/sec) — only for non-rotation commands
        if command not in ('rotate_cw', 'rotate_ccw'):
            bias['rotation'] = (1 - alpha) * bias['rotation'] + alpha * observed_rotation
            # Clamp
            bias['rotation'] = max(-self.MAX_ROTATION_BIAS,
                                   min(self.MAX_ROTATION_BIAS, bias['rotation']))

        # Update speed scale
        if expected_speed > 0 and actual_speed > 0:
            observed_scale = actual_speed / expected_speed
            observed_scale = max(self.SPEED_SCALE_RANGE[0],
                                min(self.SPEED_SCALE_RANGE[1], observed_scale))
            bias['speed_scale'] = (1 - alpha) * bias['speed_scale'] + alpha * observed_scale

        self.table[command] = bias

    def check_battery_staleness(self, current_voltage):
        """Check if battery has dropped enough to mark table stale."""
        if self._calibration_voltage is None:
            self._calibration_voltage = current_voltage
            return

        if abs(current_voltage - self._calibration_voltage) > 0.5:
            self.stale = True
        else:
            self.stale = False

    def compute_motor_correction(self, command, speed):
        """Compute per-wheel speed adjustments to compensate for known bias.

        Args:
            command: Movement command name.
            speed: Base PWM speed.

        Returns:
            tuple of 4 ints: (left_upper_adj, left_lower_adj, right_upper_adj, right_lower_adj)
            These are additive adjustments to the base motor values.
        """
        bias = self.get_bias(command)
        max_adj = int(speed * self.MAX_WHEEL_ADJUST_PCT)

        # Rotation compensation: add differential between left and right
        # If car rotates CW (positive heading decrease), left wheels spin
        # slightly more than right → reduce left, increase right
        rotation_bias = bias['rotation']
        # Scale rotation to a PWM adjustment (rough: 1 deg/sec ≈ 20 PWM units)
        rotation_adj = int(rotation_bias * 20)
        rotation_adj = max(-max_adj, min(max_adj, rotation_adj))

        # Lateral compensation: adjust front/rear differential
        # (less effective for mecanum, but helps)
        lateral_adj = int(bias['lateral'] * 15)
        lateral_adj = max(-max_adj, min(max_adj, lateral_adj))

        # Apply: counter the observed bias
        # If car drifts CW, we increase right-side speed (or decrease left)
        left_adj = rotation_adj // 2
        right_adj = -(rotation_adj // 2)

        return (left_adj, left_adj, right_adj, right_adj)

    @property
    def calibration_warning(self):
        """True if any bias exceeds guardrail soft limits."""
        for cmd, bias in self.table.items():
            if abs(bias.get('rotation', 0)) > self.MAX_ROTATION_BIAS * 0.8:
                return True
            scale = bias.get('speed_scale', 1.0)
            if scale < 0.6 or scale > 1.4:
                return True
        return False
```

**Step 2: Commit**

```bash
git add calibration.py
git commit -m "feat: add MotorBiasTable with EMA learning and persistence"
```

---

### Task 4: Add ServoAlignment to calibration.py

**Files:**
- Modify: `calibration.py`

**What this does:** Startup routine that finds the true pan center by sweeping across a wall and finding the minimum distance angle (perpendicular to wall). The offset from 90° becomes the correction applied to all pan commands.

**Step 1: Add ServoAlignment class**

```python
class ServoAlignment:
    """Detect true pan servo center by finding perpendicular to nearest wall.

    Startup routine:
    1. Full sweep to find nearest wall
    2. Fine sweep (60-120° in 1° steps) against that wall
    3. Minimum distance angle = perpendicular = true straight ahead
    4. Offset from 90° = pan_offset_deg (applied to all future pan commands)
    """

    def __init__(self, get_ultrasonic_fn, move_servo_smooth_fn):
        self._get_ultrasonic = get_ultrasonic_fn
        self._move_servo_smooth = move_servo_smooth_fn

        self.pan_offset_deg = 0.0
        self.servo_slop_deg = 0.0  # range of angles with same min distance
        self.calibrated = False

    def calibrate(self):
        """Run the full servo alignment routine.

        Returns:
            dict with offset, slop, and raw sweep data.
        """
        # Step 1: Coarse sweep to find nearest wall direction
        coarse = {}
        for angle in range(30, 151, 10):
            self._move_servo_smooth(1, angle, 200)
            time.sleep(0.15)
            try:
                dist = self._get_ultrasonic().get_distance()
                coarse[angle] = dist
            except Exception:
                coarse[angle] = 999

        # Find the angle with minimum distance (nearest wall)
        min_angle = min(coarse, key=coarse.get)
        min_dist = coarse[min_angle]

        if min_dist > 200:
            # No wall close enough for calibration
            self._move_servo_smooth(1, 90, 200)
            return {
                'status': 'no_wall',
                'message': 'No wall within 200cm for calibration',
                'pan_offset_deg': 0.0,
            }

        # Step 2: Fine sweep around the minimum
        fine_start = max(60, min_angle - 30)
        fine_end = min(120, min_angle + 30)

        fine = {}
        for angle in range(fine_start, fine_end + 1, 1):
            self._move_servo_smooth(1, angle, 200)
            time.sleep(0.15)
            try:
                dist = self._get_ultrasonic().get_distance()
                fine[angle] = dist
            except Exception:
                fine[angle] = 999

        # Find true minimum
        true_min_angle = min(fine, key=fine.get)
        true_min_dist = fine[true_min_angle]

        # Detect slop: range of angles within 0.5cm of minimum
        slop_angles = [a for a, d in fine.items() if abs(d - true_min_dist) < 0.5]
        self.servo_slop_deg = max(slop_angles) - min(slop_angles) if slop_angles else 0

        # Offset from 90°
        self.pan_offset_deg = true_min_angle - 90.0
        self.calibrated = True

        # Return to corrected center
        self._move_servo_smooth(1, 90, 200)

        return {
            'status': 'ok',
            'pan_offset_deg': self.pan_offset_deg,
            'servo_slop_deg': self.servo_slop_deg,
            'true_center_angle': true_min_angle,
            'wall_distance_cm': true_min_dist,
            'coarse_sweep': coarse,
            'fine_sweep': fine,
        }

    def correct_pan_angle(self, requested_angle):
        """Apply offset correction to a pan angle.

        Args:
            requested_angle: Desired pan angle (0-180).

        Returns:
            Corrected pan angle (0-180), clamped to valid range.
        """
        corrected = requested_angle + self.pan_offset_deg
        return max(0, min(180, int(round(corrected))))
```

**Step 2: Commit**

```bash
git add calibration.py
git commit -m "feat: add ServoAlignment startup calibration routine"
```

---

### Task 5: Add SensorFusion to calibration.py

**Files:**
- Modify: `calibration.py`

**What this does:** Combines the three displacement estimates (visual odometry, ultrasonic reference, dead reckoning) using confidence-weighted averaging. Dynamic weights — if a signal has low confidence, its weight drops and the others rebalance.

**Step 1: Add SensorFusion class**

```python
class SensorFusion:
    """Combine multiple displacement estimates with confidence-based weighting.

    Base weights:
        visual odometry: 0.5 (high precision when features available)
        ultrasonic reference: 0.3 (absolute but requires walls)
        dead reckoning: 0.2 (always available, drifts over time)

    Weights are scaled by each source's confidence score and renormalized.
    If all confidences are 0, falls back to pure dead reckoning.
    """

    BASE_WEIGHTS = {
        'visual': 0.5,
        'ultrasonic': 0.3,
        'dead_reckoning': 0.2,
    }

    def fuse(self, visual_estimate, ultrasonic_estimate, dead_reckoning_estimate):
        """Combine three displacement estimates into one corrected estimate.

        Each estimate is a dict with keys:
            dx_cm, dy_cm, d_theta_deg, confidence (0.0-1.0)

        Args:
            visual_estimate: From VisualOdometry.compute_displacement()
            ultrasonic_estimate: From UltrasonicReference.compute_correction()
            dead_reckoning_estimate: From PoseTracker (always confidence=1.0)

        Returns:
            dict with fused dx_cm, dy_cm, d_theta_deg, and per-source weights used.
        """
        estimates = {
            'visual': visual_estimate,
            'ultrasonic': ultrasonic_estimate,
            'dead_reckoning': dead_reckoning_estimate,
        }

        # Compute confidence-scaled weights
        raw_weights = {}
        for source, est in estimates.items():
            confidence = est.get('confidence', 0.0)
            raw_weights[source] = self.BASE_WEIGHTS[source] * confidence

        total = sum(raw_weights.values())

        if total < 0.001:
            # All confidences near zero — pure dead reckoning fallback
            dr = dead_reckoning_estimate
            return {
                'dx_cm': dr.get('dx_cm', 0.0),
                'dy_cm': dr.get('dy_cm', 0.0),
                'd_theta_deg': dr.get('d_theta_deg', 0.0),
                'weights': {'visual': 0.0, 'ultrasonic': 0.0, 'dead_reckoning': 1.0},
                'fused': False,
            }

        weights = {k: v / total for k, v in raw_weights.items()}

        # Weighted average of each component
        fused_dx = sum(weights[s] * estimates[s].get('dx_cm', 0.0) for s in estimates)
        fused_dy = sum(weights[s] * estimates[s].get('dy_cm', 0.0) for s in estimates)
        fused_theta = sum(weights[s] * estimates[s].get('d_theta_deg', 0.0) for s in estimates)

        return {
            'dx_cm': round(fused_dx, 2),
            'dy_cm': round(fused_dy, 2),
            'd_theta_deg': round(fused_theta, 2),
            'weights': {k: round(v, 3) for k, v in weights.items()},
            'fused': True,
        }
```

**Step 2: Commit**

```bash
git add calibration.py
git commit -m "feat: add SensorFusion confidence-weighted estimator"
```

---

### Task 6: Integrate calibration into AutonomyModule

**Files:**
- Modify: `autonomy.py:419-624` — AutonomyModule.__init__() and execute_step()

**What this does:** Wire the calibration classes into the existing step cycle. Before movement: capture photo + 3-point ultrasonic sweep. After movement: capture again, run fusion, use fused estimate to update pose instead of pure dead reckoning. Apply motor bias corrections before movement.

**Step 1: Update AutonomyModule.__init__() to initialize calibration objects**

Add to `autonomy.py` imports at the top:

```python
from calibration import (
    VisualOdometry, UltrasonicReference, MotorBiasTable,
    ServoAlignment, SensorFusion,
)
```

Add a `_capture_jpeg` import from server (or accept as dependency injection). Since `autonomy.py` shouldn't depend on `server.py`, add a `get_camera_fn` parameter:

Update `__init__` to accept `get_camera_fn` and initialize calibration objects:

```python
def __init__(self, get_motor_fn, get_servo_fn, get_ultrasonic_fn,
             get_infrared_fn, get_adc_fn, move_servo_smooth_fn,
             get_camera_fn=None):
    self._get_motor = get_motor_fn
    self._get_servo = get_servo_fn
    self._get_ultrasonic = get_ultrasonic_fn
    self._get_infrared = get_infrared_fn
    self._get_adc = get_adc_fn
    self._move_servo_smooth = move_servo_smooth_fn
    self._get_camera = get_camera_fn  # returns JPEG bytes or None

    self.grid = OccupancyGrid()
    self.pose = PoseTracker()
    self.safety = SafetyMonitor(get_ultrasonic_fn, get_motor_fn)

    # Calibration components
    self.visual_odom = VisualOdometry()
    self.ultrasonic_ref = UltrasonicReference(
        get_ultrasonic_fn, get_servo_fn, move_servo_smooth_fn
    )
    self.bias_table = MotorBiasTable()
    self.servo_align = ServoAlignment(get_ultrasonic_fn, move_servo_smooth_fn)
    self.fusion = SensorFusion()

    self._correction_enabled = True  # can be toggled via configure()

    self._movement_lock = threading.Lock()
```

**Step 2: Update execute_step() default movement branch**

Replace the movement section of `execute_step()` (after the action handlers, starting at `# -- Default: movement command --`) with the corrected version that includes pre/post capture and fusion:

```python
# -- Default: movement command --
if command is None:
    command = 'forward'
if command not in self.COMMANDS:
    return {
        'error': f'Unknown command: {command}',
        'valid': list(self.COMMANDS.keys()),
    }

speed = max(0, min(4095, speed))
duration = max(0.1, min(10.0, duration))

# Pre-movement sensors
pre_distance = self._read_distance()
ir_data = self._read_infrared()
battery = self._read_battery()

# Pre-movement calibration captures
if self._correction_enabled:
    if self._get_camera:
        try:
            before_jpeg = self._get_camera()
            self.visual_odom.capture_before(before_jpeg)
        except Exception:
            pass
    self.ultrasonic_ref.capture_before()
    self.bias_table.check_battery_staleness(battery)

# Apply motor bias correction
correction = (0, 0, 0, 0)
if self._correction_enabled:
    correction = self.bias_table.compute_motor_correction(command, speed)

# Start safety monitor
self.safety.start_monitoring(command)

# Execute movement with bias-corrected motors
multipliers = self.COMMANDS[command]
base_vals = tuple(int(m * speed) for m in multipliers)
corrected_vals = tuple(
    max(-4095, min(4095, b + c))
    for b, c in zip(base_vals, correction)
)
self._get_motor().set_motor_model(*corrected_vals)

# Wait for duration or safety interrupt
start = time.time()
while time.time() - start < duration:
    if self.safety.collision_flag or self.safety.stuck_flag:
        break
    time.sleep(0.05)

actual_duration = time.time() - start

# Stop motors
self._get_motor().set_motor_model(0, 0, 0, 0)
self.safety.stop_monitoring()
time.sleep(0.1)

# Post-movement sensors
post_distance = self._read_distance()

# Post-movement calibration captures and fusion
fusion_result = None
if self._correction_enabled and not self.safety.collision_flag:
    # Visual odometry
    visual_est = {'dx_cm': 0, 'dy_cm': 0, 'd_theta_deg': 0, 'confidence': 0}
    if self._get_camera:
        try:
            after_jpeg = self._get_camera()
            visual_est = self.visual_odom.compute_displacement(
                after_jpeg, distance_cm=max(pre_distance, 10)
            )
        except Exception:
            pass

    # Ultrasonic reference
    self.ultrasonic_ref.capture_after()
    ultra_est = self.ultrasonic_ref.compute_correction()

    # Dead reckoning estimate
    dr_dist = speed * self.pose.SPEED_CALIBRATION_FACTOR * actual_duration
    dr_heading = self.pose.heading_deg
    dr_est = {
        'dx_cm': dr_dist * math.cos(math.radians(dr_heading)) if command == 'forward' else 0,
        'dy_cm': 0,
        'd_theta_deg': 0,
        'confidence': 1.0,
    }
    # Compute dead reckoning based on command type
    if command in ('forward', 'backward'):
        sign = 1 if command == 'forward' else -1
        dr_est['dx_cm'] = sign * dr_dist
    elif command in ('strafe_left', 'strafe_right'):
        sign = 1 if command == 'strafe_left' else -1
        dr_est['dy_cm'] = sign * dr_dist
    elif command in ('rotate_cw', 'rotate_ccw'):
        rot = speed * self.pose.ROTATION_CALIBRATION_FACTOR * actual_duration
        sign = -1 if command == 'rotate_cw' else 1
        dr_est['d_theta_deg'] = sign * rot

    # Fuse all three
    fusion_result = self.fusion.fuse(visual_est, ultra_est, dr_est)

    # Update pose with fused estimate instead of pure dead reckoning
    with self.pose._lock:
        rad = math.radians(self.pose.heading_deg)
        self.pose.x_cm += (fusion_result['dx_cm'] * math.cos(rad)
                          - fusion_result['dy_cm'] * math.sin(rad))
        self.pose.y_cm += (fusion_result['dx_cm'] * math.sin(rad)
                          + fusion_result['dy_cm'] * math.cos(rad))
        self.pose.heading_deg = (self.pose.heading_deg
                                + fusion_result['d_theta_deg']) % 360

    # Update motor bias table with observed errors
    if actual_duration > 0.1 and command not in ('stop',):
        observed_lateral = fusion_result['dy_cm'] / actual_duration
        observed_rotation = fusion_result['d_theta_deg'] / actual_duration
        actual_speed_cm = fusion_result['dx_cm'] / actual_duration if actual_duration > 0 else 0
        self.bias_table.update(
            command, observed_lateral, observed_rotation,
            speed * self.pose.SPEED_CALIBRATION_FACTOR, actual_speed_cm,
            actual_duration,
        )
        self.bias_table.save()

else:
    # No correction — use pure dead reckoning (original behavior)
    if not self.safety.collision_flag:
        self.pose.update_from_movement(command, speed, actual_duration)

result = {
    'command': command,
    'speed': speed,
    'requested_duration': duration,
    'actual_duration': round(actual_duration, 3),
    'pre_distance_cm': pre_distance,
    'post_distance_cm': post_distance,
    'infrared': ir_data,
    'battery_voltage': battery,
    'collision': self.safety.collision_flag,
    'stuck': self.safety.stuck_flag,
    'pose': self.pose.get_pose(),
}

if fusion_result:
    result['fusion'] = fusion_result
    result['calibration_warning'] = self.bias_table.calibration_warning
    result['bias_stale'] = self.bias_table.stale

return result
```

**Step 3: Update configure() to support correction toggle**

Add to `AutonomyModule.configure()`:

```python
if 'correction_enabled' in kwargs:
    self._correction_enabled = bool(kwargs['correction_enabled'])
    updated['correction_enabled'] = self._correction_enabled
```

**Step 4: Commit**

```bash
git add autonomy.py
git commit -m "feat: integrate sensor fusion into execute_step cycle"
```

---

### Task 7: Add /auto/calibrate endpoint and camera dependency to server.py

**Files:**
- Modify: `server.py:105-118` — get_autonomy()
- Modify: `server.py:687-782` — autonomy routes section

**Step 1: Update get_autonomy() to pass get_camera_fn**

```python
def get_autonomy():
    global _autonomy
    if _autonomy is None:
        from autonomy import AutonomyModule
        _autonomy = AutonomyModule(
            get_motor_fn=get_motor,
            get_servo_fn=get_servo,
            get_ultrasonic_fn=get_ultrasonic,
            get_infrared_fn=get_infrared,
            get_adc_fn=get_adc,
            move_servo_smooth_fn=move_servo_smooth,
            get_camera_fn=_capture_jpeg,
        )
    return _autonomy
```

**Step 2: Add /auto/calibrate route**

Add after the `/auto/reset` route:

```python
@app.route("/auto/calibrate", methods=["POST"])
def auto_calibrate():
    """Run startup calibration: servo alignment + bias table seeding.

    This takes ~30 seconds. Run once per session or after battery swap.
    """
    auto = get_autonomy()

    # Step 1: Servo alignment
    servo_result = auto.servo_align.calibrate()

    # Step 2: Seed bias table with short movements
    seed_commands = ['forward', 'backward', 'strafe_left', 'strafe_right']
    seed_results = []
    for cmd in seed_commands:
        step = auto.execute_step(command=cmd, speed=500, duration=0.5)
        seed_results.append({
            'command': cmd,
            'fusion': step.get('fusion'),
        })
        time.sleep(0.5)

    # Save bias table after seeding
    auto.bias_table.save()

    return jsonify({
        'status': 'ok',
        'servo_alignment': servo_result,
        'bias_seed': seed_results,
        'bias_table': auto.bias_table.table,
    })
```

**Step 3: Update /auto/configure to expose correction_enabled**

Add `correction_enabled` to the `valid_keys` list in the no-change response:

```python
"valid_keys": [
    "collision_threshold_cm", "stuck_time_threshold",
    "stuck_spread_threshold", "speed_calibration",
    "rotation_calibration", "correction_enabled",
],
```

**Step 4: Update index route to include calibrate endpoint**

Add to the `"autonomy"` section of the index:

```python
"POST /auto/calibrate": "Startup calibration — servo alignment + bias seeding (~30s)",
```

**Step 5: Commit**

```bash
git add server.py
git commit -m "feat: add /auto/calibrate endpoint and camera DI for fusion"
```

---

### Task 8: Update API_SPEC.md with calibration endpoints

**Files:**
- Modify: `API_SPEC.md`

**Step 1: Add /auto/calibrate documentation**

Add after the `/auto/reset` section:

```markdown
### POST /auto/calibrate

Run startup calibration: servo alignment detection + motor bias table seeding. Takes ~30 seconds.

**Request Body:** `{}` (empty, or omit body)

**Response:**
```json
{
  "status": "ok",
  "servo_alignment": {
    "status": "ok",
    "pan_offset_deg": -2.0,
    "servo_slop_deg": 3.0,
    "true_center_angle": 88,
    "wall_distance_cm": 62.3
  },
  "bias_seed": [
    {"command": "forward", "fusion": {"dx_cm": 1.8, "dy_cm": 0.1, "d_theta_deg": 0.5, "weights": {...}}},
    {"command": "backward", "fusion": {...}}
  ],
  "bias_table": {
    "forward": {"lateral": 0.2, "rotation": 1.1, "speed_scale": 0.92},
    "backward": {"lateral": -0.1, "rotation": -0.5, "speed_scale": 0.88}
  }
}
```

> **When to run:** Once per session, or after battery swap. The car should be facing a wall within 200cm.
```

**Step 2: Update /auto/step response to include fusion fields**

Add to the `/auto/step` movement mode response documentation:

```markdown
When self-correction is enabled (default), movement responses include additional fields:

| Field | Type | Description |
|-------|------|-------------|
| `fusion` | object | Fused displacement estimate with per-source weights |
| `calibration_warning` | bool | True if any bias exceeds soft guardrails |
| `bias_stale` | bool | True if battery voltage dropped >0.5V from calibration |
```

**Step 3: Update /auto/configure to document correction_enabled**

Add `correction_enabled` to the configure parameters table:

```markdown
| `correction_enabled` | bool | true | Enable/disable self-correction (sensor fusion + bias) |
```

**Step 4: Commit**

```bash
git add API_SPEC.md
git commit -m "docs: add /auto/calibrate and fusion fields to API spec"
```

---

### Task 9: Deploy and verify on the car

**Files:**
- Deploy: `calibration.py`, `autonomy.py`, `server.py` to Pi

**Step 1: SCP files to the Pi**

```bash
scp calibration.py autonomy.py server.py freenove-car:~/freenove-car-api/
```

**Step 2: Check OpenCV availability**

```bash
ssh freenove-car "python3 -c 'import cv2; print(cv2.__version__)'"
```

If OpenCV is missing, install it:
```bash
ssh freenove-car "pip3 install opencv-python-headless"
```

**Step 3: Restart server**

```bash
ssh freenove-car "pkill -f 'python3 server.py' || true"
sleep 2
ssh freenove-car "cd ~/freenove-car-api && nohup python3 server.py > server.log 2>&1 &"
```

**Step 4: Verify server starts**

```bash
curl -s http://192.168.1.16:5000/ | python3 -m json.tool
```

Expected: JSON with `auto/calibrate` in the endpoints list.

**Step 5: Test calibration endpoint**

Place the car facing a wall within 100cm, then:

```bash
curl -s -X POST http://192.168.1.16:5000/auto/calibrate | python3 -m json.tool
```

Expected: `servo_alignment.status == "ok"`, `pan_offset_deg` near 0 (±5°), `bias_seed` with fusion results.

**Step 6: Test a step with fusion**

```bash
curl -s -X POST http://192.168.1.16:5000/auto/step \
  -H "Content-Type: application/json" \
  -d '{"command":"forward","speed":500,"duration":0.5}' | python3 -m json.tool
```

Expected: Response includes `fusion` object with `weights`, `dx_cm`, `dy_cm`, `d_theta_deg`.

**Step 7: Test with correction disabled**

```bash
curl -s -X POST http://192.168.1.16:5000/auto/configure \
  -H "Content-Type: application/json" \
  -d '{"correction_enabled": false}' | python3 -m json.tool
```

Then run a step — should NOT include `fusion` in the response (pure dead reckoning).

**Step 8: Verify bias table persists**

```bash
ssh freenove-car "cat ~/freenove-car-api/motor_bias.json"
```

Expected: JSON with biases for the commands tested during calibration.

---

## Timing Summary

| Task | Scope |
|------|-------|
| 1 | VisualOdometry (OpenCV optical flow) |
| 2 | UltrasonicReference (3-point sweep) |
| 3 | MotorBiasTable (EMA + persistence) |
| 4 | ServoAlignment (startup calibration) |
| 5 | SensorFusion (weighted combiner) |
| 6 | Integrate into autonomy.py execute_step() |
| 7 | Server routes + camera DI |
| 8 | API documentation |
| 9 | Deploy and hardware verification |

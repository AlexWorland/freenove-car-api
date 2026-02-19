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

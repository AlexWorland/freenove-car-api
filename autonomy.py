"""Autonomous navigation module for the Freenove 4WD Smart Car.

Provides occupancy grid mapping, dead-reckoning pose tracking,
real-time safety monitoring, and a high-level autonomy controller
that Claude (or any HTTP client) drives via /auto/* endpoints.

Architecture:
    Claude (brain)                    Pi (reflexes)
    POST /auto/step ───────────────►  Execute command
                                       SafetyMonitor (~12Hz)
                                       ├─ Collision avoidance
                                       └─ Stuck detection
    ◄────────────── JSON response     Update grid + pose

No external dependencies — stdlib only.
"""

import math
import threading
import time

from calibration import (
    VisualOdometry, UltrasonicReference, MotorBiasTable,
    ServoAlignment, SensorFusion,
)


# ---------------------------------------------------------------------------
# OccupancyGrid — 200x200 cells, 5cm resolution = 10m x 10m
# ---------------------------------------------------------------------------

class OccupancyGrid:
    """Probabilistic occupancy grid stored as a flat bytearray.

    Cell values: 0 = definitely free, 127 = unknown, 255 = definitely occupied.
    Asymmetric updates: +15 occupied, -5 free (obstacles stick, free space clears).
    """

    FREE_DELTA = -5
    OCCUPIED_DELTA = 15
    MAX_SENSOR_RANGE_CM = 400  # ultrasonic max reliable range

    def __init__(self, width=200, height=200, cell_size_cm=5):
        self.width = width
        self.height = height
        self.cell_size_cm = cell_size_cm
        self.grid = bytearray([127] * (width * height))
        # Car starts at grid center
        self.origin_x = width // 2
        self.origin_y = height // 2

    def world_to_grid(self, x_cm, y_cm):
        """Convert world coordinates (cm) to grid cell indices."""
        gx = self.origin_x + int(round(x_cm / self.cell_size_cm))
        gy = self.origin_y - int(round(y_cm / self.cell_size_cm))
        return gx, gy

    def _clamp_update(self, current, delta):
        return max(0, min(255, current + delta))

    def mark_ray(self, car_x_cm, car_y_cm, angle_deg, distance_cm):
        """Trace a ray from car through free space to obstacle, updating grid.

        Args:
            car_x_cm, car_y_cm: Car position in world coordinates.
            angle_deg: Ray direction in world frame (0=east, 90=north).
            distance_cm: Measured distance to obstacle.
        """
        gx0, gy0 = self.world_to_grid(car_x_cm, car_y_cm)

        rad = math.radians(angle_deg)
        end_x = car_x_cm + distance_cm * math.cos(rad)
        end_y = car_y_cm + distance_cm * math.sin(rad)
        gx1, gy1 = self.world_to_grid(end_x, end_y)

        points = self._bresenham(gx0, gy0, gx1, gy1)

        for i, (gx, gy) in enumerate(points):
            if not (0 <= gx < self.width and 0 <= gy < self.height):
                continue
            idx = gy * self.width + gx
            if i < len(points) - 1:
                # Free space along the ray
                self.grid[idx] = self._clamp_update(self.grid[idx], self.FREE_DELTA)
            else:
                # Endpoint — only mark occupied if within sensor range
                if distance_cm < self.MAX_SENSOR_RANGE_CM:
                    self.grid[idx] = self._clamp_update(self.grid[idx], self.OCCUPIED_DELTA)

    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        """Bresenham's line algorithm — returns list of (x, y) grid cells."""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return points

    def to_text_grid(self, car_x_cm, car_y_cm, car_heading_deg, radius=20):
        """Export an ASCII art view centered on the car.

        Characters: '#'=wall, '.'=free, '?'=unknown, heading arrow=car.
        Returns a (2*radius+1) x (2*radius+1) character grid.
        """
        car_gx, car_gy = self.world_to_grid(car_x_cm, car_y_cm)

        # Pick a car symbol based on heading
        heading_symbols = [
            (0, '>'), (45, '/'), (90, '^'), (135, '\\'),
            (180, '<'), (225, '/'), (270, 'v'), (315, '\\'),
        ]
        h = int(car_heading_deg) % 360
        car_char = min(heading_symbols, key=lambda hs: min(abs(h - hs[0]), 360 - abs(h - hs[0])))[1]

        lines = []
        for dy in range(-radius, radius + 1):
            row = []
            for dx in range(-radius, radius + 1):
                gx = car_gx + dx
                gy = car_gy + dy
                if dx == 0 and dy == 0:
                    row.append(car_char)
                elif 0 <= gx < self.width and 0 <= gy < self.height:
                    val = self.grid[gy * self.width + gx]
                    if val > 200:
                        row.append('#')
                    elif val < 60:
                        row.append('.')
                    else:
                        row.append('?')
                else:
                    row.append('?')
            lines.append(''.join(row))
        return '\n'.join(lines)

    def reset(self):
        """Clear the grid back to all-unknown."""
        for i in range(len(self.grid)):
            self.grid[i] = 127


# ---------------------------------------------------------------------------
# PoseTracker — Dead-reckoning position estimate
# ---------------------------------------------------------------------------

class PoseTracker:
    """Track car position and heading via dead reckoning.

    Coordinate system: x-axis = east, y-axis = north.
    heading_deg: 0=east, 90=north, 180=west, 270=south.
    Car starts at (0, 0) facing north (90 deg).

    Calibration factors convert (PWM speed * time) to physical displacement.
    These must be tuned empirically for your surface and battery level.
    """

    SPEED_CALIBRATION_FACTOR = 0.007   # cm per (PWM unit * second)
    ROTATION_CALIBRATION_FACTOR = 0.09  # degrees per (PWM unit * second)

    def __init__(self):
        self._lock = threading.Lock()
        self.x_cm = 0.0
        self.y_cm = 0.0
        self.heading_deg = 90.0  # facing north

    def update_from_movement(self, command, speed, duration_sec):
        """Update pose estimate after a movement command completes.

        Args:
            command: Movement command name (forward, backward, strafe_left, etc.)
            speed: PWM speed value (0-4095).
            duration_sec: How long the movement lasted.
        """
        with self._lock:
            dist = speed * self.SPEED_CALIBRATION_FACTOR * duration_sec

            if command == 'forward':
                rad = math.radians(self.heading_deg)
                self.x_cm += dist * math.cos(rad)
                self.y_cm += dist * math.sin(rad)

            elif command == 'backward':
                rad = math.radians(self.heading_deg)
                self.x_cm -= dist * math.cos(rad)
                self.y_cm -= dist * math.sin(rad)

            elif command == 'strafe_left':
                rad = math.radians(self.heading_deg + 90)
                self.x_cm += dist * math.cos(rad)
                self.y_cm += dist * math.sin(rad)

            elif command == 'strafe_right':
                rad = math.radians(self.heading_deg - 90)
                self.x_cm += dist * math.cos(rad)
                self.y_cm += dist * math.sin(rad)

            elif command == 'rotate_cw':
                angle = speed * self.ROTATION_CALIBRATION_FACTOR * duration_sec
                self.heading_deg = (self.heading_deg - angle) % 360

            elif command == 'rotate_ccw':
                angle = speed * self.ROTATION_CALIBRATION_FACTOR * duration_sec
                self.heading_deg = (self.heading_deg + angle) % 360

            elif command == 'diagonal_fl':
                rad = math.radians(self.heading_deg + 45)
                self.x_cm += dist * math.cos(rad)
                self.y_cm += dist * math.sin(rad)

            elif command == 'diagonal_fr':
                rad = math.radians(self.heading_deg - 45)
                self.x_cm += dist * math.cos(rad)
                self.y_cm += dist * math.sin(rad)

            elif command == 'diagonal_bl':
                rad = math.radians(self.heading_deg + 135)
                self.x_cm += dist * math.cos(rad)
                self.y_cm += dist * math.sin(rad)

            elif command == 'diagonal_br':
                rad = math.radians(self.heading_deg - 135)
                self.x_cm += dist * math.cos(rad)
                self.y_cm += dist * math.sin(rad)

            elif command in ('left', 'right'):
                # Turn commands: simplified as partial rotation
                angle = speed * self.ROTATION_CALIBRATION_FACTOR * duration_sec * 0.5
                if command == 'left':
                    self.heading_deg = (self.heading_deg + angle) % 360
                else:
                    self.heading_deg = (self.heading_deg - angle) % 360

    def get_pose(self):
        """Return current pose as a dict (thread-safe)."""
        with self._lock:
            return {
                'x_cm': round(self.x_cm, 1),
                'y_cm': round(self.y_cm, 1),
                'heading_deg': round(self.heading_deg, 1),
            }

    def reset(self, x=0.0, y=0.0, heading=90.0):
        """Reset pose to given values."""
        with self._lock:
            self.x_cm = float(x)
            self.y_cm = float(y)
            self.heading_deg = float(heading)


# ---------------------------------------------------------------------------
# SafetyMonitor — Background thread for real-time collision/stuck detection
# ---------------------------------------------------------------------------

class SafetyMonitor:
    """Daemon thread running at ~12Hz during active movement.

    - Collision avoidance: emergency stop if ultrasonic < threshold during
      forward-facing commands.
    - Stuck detection: if distance readings have < spread_threshold spread
      over time_threshold seconds of movement, flag stuck and stop motors.

    Uses GPIO-based ultrasonic (not I2C), so no bus contention with
    motor/servo commands.
    """

    FORWARD_COMMANDS = frozenset({'forward', 'diagonal_fl', 'diagonal_fr'})

    def __init__(self, get_ultrasonic_fn, get_motor_fn,
                 collision_threshold_cm=15,
                 stuck_time_threshold=1.5,
                 stuck_spread_threshold=2.0):
        self._get_ultrasonic = get_ultrasonic_fn
        self._get_motor = get_motor_fn
        self.collision_threshold_cm = collision_threshold_cm
        self.stuck_time_threshold = stuck_time_threshold
        self.stuck_spread_threshold = stuck_spread_threshold

        self.collision_flag = False
        self.stuck_flag = False

        self._active_command = None
        self._monitoring = False
        self._stop_event = threading.Event()
        self._thread = None

        # Stuck detection history
        self._distance_history = []
        self._movement_start_time = None

    def start_monitoring(self, command):
        """Begin monitoring for the given movement command."""
        self.collision_flag = False
        self.stuck_flag = False
        self._active_command = command
        self._distance_history = []
        self._movement_start_time = time.time()
        self._monitoring = True
        self._stop_event.clear()

        if self._thread is None or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self._thread.start()

    def stop_monitoring(self):
        """Stop the monitoring loop (motors may still be running)."""
        self._monitoring = False
        self._active_command = None
        self._stop_event.set()

    def _monitor_loop(self):
        while not self._stop_event.is_set():
            if not self._monitoring:
                self._stop_event.wait(0.1)
                continue

            try:
                dist = self._get_ultrasonic().get_distance()

                # -- Collision avoidance --
                if (self._active_command in self.FORWARD_COMMANDS
                        and dist < self.collision_threshold_cm):
                    self._get_motor().set_motor_model(0, 0, 0, 0)
                    self.collision_flag = True
                    self._monitoring = False
                    continue

                # -- Stuck detection --
                self._distance_history.append(dist)
                if len(self._distance_history) > 20:
                    self._distance_history = self._distance_history[-20:]

                elapsed = time.time() - self._movement_start_time
                if (elapsed >= self.stuck_time_threshold
                        and len(self._distance_history) >= 5):
                    recent = self._distance_history[-5:]
                    spread = max(recent) - min(recent)
                    if spread < self.stuck_spread_threshold:
                        self._get_motor().set_motor_model(0, 0, 0, 0)
                        self.stuck_flag = True
                        self._monitoring = False
                        continue

            except Exception:
                pass

            time.sleep(1.0 / 12)  # ~12Hz


# ---------------------------------------------------------------------------
# AutonomyModule — High-level controller tying everything together
# ---------------------------------------------------------------------------

class AutonomyModule:
    """Orchestrates grid mapping, pose tracking, safety, and recovery.

    Receives hardware getter functions from server.py via dependency injection.
    All methods are designed for one-call-per-decision-cycle usage from Claude.
    """

    # Motor command multipliers — identical to MECANUM_DIRECTIONS in server.py
    COMMANDS = {
        "forward":      (-1, -1, -1, -1),
        "backward":     ( 1,  1,  1,  1),
        "left":         ( 1,  1, -1, -1),
        "right":        (-1, -1,  1,  1),
        "strafe_left":  ( 1, -1, -1,  1),
        "strafe_right": (-1,  1,  1, -1),
        "rotate_cw":    (-1, -1,  1,  1),
        "rotate_ccw":   ( 1,  1, -1, -1),
        "diagonal_fl":  ( 0, -1, -1,  0),
        "diagonal_fr":  (-1,  0,  0, -1),
        "diagonal_bl":  ( 1,  0,  0,  1),
        "diagonal_br":  ( 0,  1,  1,  0),
        "stop":         ( 0,  0,  0,  0),
    }

    # Servo channels after the swap fix: ch0=tilt, ch1=pan
    PAN_CHANNEL = 1
    TILT_CHANNEL = 0

    # Pre-built recovery maneuvers: list of (command, speed, duration_sec)
    RECOVERY_MANEUVERS = {
        "back_up": [
            ("backward", 1500, 0.8),
        ],
        "back_and_rotate": [
            ("backward", 1500, 0.8),
            ("rotate_cw", 1500, 0.6),
        ],
        "wiggle_free": [
            ("strafe_left", 1500, 0.3),
            ("strafe_right", 1500, 0.3),
            ("backward", 1500, 0.5),
        ],
        "full_retreat": [
            ("backward", 1500, 1.5),
            ("rotate_cw", 1500, 1.0),
        ],
        "strafe_escape_left": [
            ("strafe_left", 1500, 0.8),
            ("forward", 1000, 0.3),
        ],
        "strafe_escape_right": [
            ("strafe_right", 1500, 0.8),
            ("forward", 1000, 0.3),
        ],
    }

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

    # -- Sensor helpers -------------------------------------------------------

    def _read_distance(self):
        try:
            return self._get_ultrasonic().get_distance()
        except Exception:
            return -1

    def _read_infrared(self):
        try:
            ir = self._get_infrared()
            return {
                'left': ir.read_one_infrared(1),
                'middle': ir.read_one_infrared(2),
                'right': ir.read_one_infrared(3),
            }
        except Exception:
            return {'left': -1, 'middle': -1, 'right': -1}

    def _read_battery(self):
        try:
            adc = self._get_adc()
            raw = adc.read_adc(2)
            multiplier = 3 if adc.pcb_version == 1 else 2
            return round(raw * multiplier, 2)
        except Exception:
            return -1

    # -- Scan -----------------------------------------------------------------

    def perform_scan(self, pan_min=30, pan_max=150, step=5, settle_ms=150):
        """Horizontal ultrasonic sweep: rotate pan servo, read distance at each angle.

        Updates the occupancy grid with ray traces from the current pose.

        Args:
            pan_min: Start pan angle (degrees, 0=right, 180=left).
            pan_max: End pan angle (degrees).
            step: Degrees between readings.
            settle_ms: Milliseconds to wait after each servo move.

        Returns:
            dict with readings list, pose, and count.
        """
        servo = self._get_servo()
        readings = []
        pose = self.pose.get_pose()

        # Level the tilt for a horizontal scan
        servo.set_servo_pwm(str(self.TILT_CHANNEL), 90)
        time.sleep(0.1)

        for pan_angle in range(pan_min, pan_max + 1, step):
            self._move_servo_smooth(self.PAN_CHANNEL, pan_angle, 200)
            time.sleep(settle_ms / 1000.0)

            dist = self._read_distance()
            readings.append({
                'pan_angle': pan_angle,
                'distance_cm': dist,
            })

            if dist > 0:
                # Pan servo: 90 = straight ahead, <90 = right, >90 = left
                relative_angle = pan_angle - 90
                world_angle = pose['heading_deg'] + relative_angle
                self.grid.mark_ray(
                    pose['x_cm'], pose['y_cm'], world_angle, dist
                )

        # Return pan to center
        self._move_servo_smooth(self.PAN_CHANNEL, 90, 200)

        return {
            'readings': readings,
            'pose': pose,
            'count': len(readings),
        }

    # -- Step (core decision cycle) -------------------------------------------

    def execute_step(self, command=None, speed=500, duration=0.5,
                     action=None, scan_params=None, recover_maneuver=None):
        """One complete sense-act cycle. This is the primary endpoint for Claude.

        Modes (via `action` parameter):
            None / omitted  — Execute a movement command (default).
            "scan"          — Perform ultrasonic sweep (no movement).
            "recover"       — Run a recovery maneuver.
            "stop"          — Emergency stop + clear flags.

        Args:
            command: Movement command name (forward, backward, etc.).
            speed: PWM speed 0-4095 (default 500).
            duration: Movement duration in seconds (default 0.5).
            action: Override action type.
            scan_params: Optional dict of scan parameters.
            recover_maneuver: Recovery maneuver name.

        Returns:
            dict with full state: distances, pose, flags, IR, battery.
        """
        if not self._movement_lock.acquire(blocking=False):
            return {'error': 'Movement already in progress'}

        try:
            # -- Action: scan --
            if action == 'scan':
                params = scan_params or {}
                result = {'scan': self.perform_scan(**params)}
                result['pose'] = self.pose.get_pose()
                return result

            # -- Action: recover --
            if action == 'recover':
                maneuver = recover_maneuver or 'back_and_rotate'
                result = {'recovery': self.execute_recovery(maneuver)}
                result['pose'] = self.pose.get_pose()
                return result

            # -- Action: stop --
            if action == 'stop':
                self._get_motor().set_motor_model(0, 0, 0, 0)
                self.safety.stop_monitoring()
                return {
                    'stopped': True,
                    'pose': self.pose.get_pose(),
                }

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
                    'dx_cm': 0,
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

        finally:
            self._movement_lock.release()

    # -- Recovery maneuvers ---------------------------------------------------

    def execute_recovery(self, maneuver='back_and_rotate'):
        """Run a pre-built recovery sequence to escape stuck situations.

        Recovery maneuvers intentionally bypass the collision monitor
        since they often involve reversing toward unknown space.

        Args:
            maneuver: Name of recovery maneuver.

        Returns:
            dict with maneuver name, executed steps, and final pose.
        """
        if maneuver not in self.RECOVERY_MANEUVERS:
            return {
                'error': f'Unknown maneuver: {maneuver}',
                'valid': list(self.RECOVERY_MANEUVERS.keys()),
            }

        motor = self._get_motor()
        steps = self.RECOVERY_MANEUVERS[maneuver]
        executed = []

        for cmd, spd, dur in steps:
            multipliers = self.COMMANDS[cmd]
            motor_vals = tuple(int(m * spd) for m in multipliers)
            motor.set_motor_model(*motor_vals)
            time.sleep(dur)
            motor.set_motor_model(0, 0, 0, 0)
            time.sleep(0.1)

            self.pose.update_from_movement(cmd, spd, dur)
            executed.append({'command': cmd, 'speed': spd, 'duration': dur})

        return {
            'maneuver': maneuver,
            'steps': executed,
            'pose': self.pose.get_pose(),
        }

    # -- State & configuration ------------------------------------------------

    def get_state(self, include_grid=False, grid_radius=20):
        """Return current state snapshot.

        Args:
            include_grid: If True, include ASCII art grid in response.
            grid_radius: Half-width of the text grid (default 20 = 41x41 chars).

        Returns:
            dict with pose, safety flags, and optional grid text.
        """
        pose = self.pose.get_pose()
        state = {
            'pose': pose,
            'collision_flag': self.safety.collision_flag,
            'stuck_flag': self.safety.stuck_flag,
        }
        if include_grid:
            state['grid'] = self.grid.to_text_grid(
                pose['x_cm'], pose['y_cm'], pose['heading_deg'], grid_radius
            )
        return state

    def configure(self, **kwargs):
        """Update tunable parameters at runtime.

        Accepted keys:
            collision_threshold_cm, stuck_time_threshold,
            stuck_spread_threshold, speed_calibration, rotation_calibration,
            correction_enabled.

        Returns:
            dict of parameters that were actually updated.
        """
        updated = {}

        if 'collision_threshold_cm' in kwargs:
            self.safety.collision_threshold_cm = float(kwargs['collision_threshold_cm'])
            updated['collision_threshold_cm'] = self.safety.collision_threshold_cm

        if 'stuck_time_threshold' in kwargs:
            self.safety.stuck_time_threshold = float(kwargs['stuck_time_threshold'])
            updated['stuck_time_threshold'] = self.safety.stuck_time_threshold

        if 'stuck_spread_threshold' in kwargs:
            self.safety.stuck_spread_threshold = float(kwargs['stuck_spread_threshold'])
            updated['stuck_spread_threshold'] = self.safety.stuck_spread_threshold

        if 'speed_calibration' in kwargs:
            self.pose.SPEED_CALIBRATION_FACTOR = float(kwargs['speed_calibration'])
            updated['speed_calibration'] = self.pose.SPEED_CALIBRATION_FACTOR

        if 'rotation_calibration' in kwargs:
            self.pose.ROTATION_CALIBRATION_FACTOR = float(kwargs['rotation_calibration'])
            updated['rotation_calibration'] = self.pose.ROTATION_CALIBRATION_FACTOR

        if 'correction_enabled' in kwargs:
            self._correction_enabled = bool(kwargs['correction_enabled'])
            updated['correction_enabled'] = self._correction_enabled

        return updated

    def reset(self, reset_grid=True, reset_pose=True):
        """Reset grid and/or pose for fresh exploration.

        Args:
            reset_grid: Clear occupancy grid to all-unknown.
            reset_pose: Reset dead reckoning to origin.

        Returns:
            dict describing what was reset.
        """
        result = {}

        if reset_grid:
            self.grid.reset()
            result['grid'] = 'reset'

        if reset_pose:
            self.pose.reset()
            result['pose'] = 'reset'

        self.safety.collision_flag = False
        self.safety.stuck_flag = False
        result['safety_flags'] = 'cleared'

        return result

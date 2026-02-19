#!/usr/bin/env python3
"""Freenove 4WD Smart Car REST API Server.

Exposes all car hardware (sensors, motors, servos, LEDs, buzzer, camera)
over a local-network Flask API running on the Raspberry Pi.
Includes a web-based control dashboard with live camera feed.
"""

import base64
import io
import sys
import time
import threading
import traceback
import subprocess

# Add Freenove SDK to path and set working directory so params.json is found
import os
FREENOVE_DIR = "/home/jazz8680/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server"
sys.path.insert(0, FREENOVE_DIR)
os.chdir(FREENOVE_DIR)

from flask import Flask, request, jsonify, Response

# ---------------------------------------------------------------------------
# Hardware singletons (lazy-initialized)
# ---------------------------------------------------------------------------

_lock = threading.Lock()
_motor = None
_servo = None
_ultrasonic = None
_infrared = None
_adc = None
_led = None
_buzzer = None
_buzzer_pwm = None
_led_animation_thread = None
_led_animation_stop = threading.Event()
_autonomy = None
_battery_led_active = True  # True = battery monitor owns LEDs
_battery_led_stop = threading.Event()

# Track current servo positions for interpolation
_servo_positions = {0: 90, 1: 90}


def get_motor():
    global _motor
    if _motor is None:
        from motor import Ordinary_Car
        _motor = Ordinary_Car()
    return _motor


def get_servo():
    global _servo
    if _servo is None:
        from servo import Servo
        _servo = Servo()
    return _servo


def get_ultrasonic():
    global _ultrasonic
    if _ultrasonic is None:
        from ultrasonic import Ultrasonic
        _ultrasonic = Ultrasonic()
    return _ultrasonic


def get_infrared():
    global _infrared
    if _infrared is None:
        from infrared import Infrared
        _infrared = Infrared()
    return _infrared


def get_adc():
    global _adc
    if _adc is None:
        from adc import ADC
        _adc = ADC()
    return _adc


def get_led():
    global _led
    if _led is None:
        from led import Led
        _led = Led()
    return _led


def get_buzzer():
    """Return a PWM buzzer on GPIO17 hardcoded to 10% volume (duty cycle)."""
    global _buzzer_pwm
    if _buzzer_pwm is None:
        from gpiozero import PWMOutputDevice
        _buzzer_pwm = PWMOutputDevice(17, frequency=1000)
    return _buzzer_pwm


def get_autonomy():
    """Return the singleton AutonomyModule, creating it on first call."""
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


def move_servo_smooth(channel, target_angle, speed=100):
    """Move servo to target angle with interpolation for speed control.

    Args:
        channel: Servo channel (0=tilt/vertical, 1=pan/horizontal)
        target_angle: Target angle 0-180
        speed: Degrees per second (1-500). Higher = faster. 0 = instant.
    """
    servo = get_servo()
    ch_str = str(channel)
    target_angle = max(0, min(180, int(target_angle)))

    if speed <= 0 or speed >= 500:
        servo.set_servo_pwm(ch_str, target_angle)
        _servo_positions[channel] = target_angle
        return

    current = _servo_positions.get(channel, 90)
    step_delay = 1.0 / speed  # seconds per degree
    direction = 1 if target_angle > current else -1

    for angle in range(int(current) + direction, target_angle + direction, direction):
        servo.set_servo_pwm(ch_str, angle)
        time.sleep(step_delay)

    _servo_positions[channel] = target_angle


# ---------------------------------------------------------------------------
# LED animation helpers
# ---------------------------------------------------------------------------

def _stop_animation():
    """Signal any running LED animation to stop and wait for it."""
    global _led_animation_thread
    _led_animation_stop.set()
    if _led_animation_thread and _led_animation_thread.is_alive():
        _led_animation_thread.join(timeout=2)
    _led_animation_thread = None


def _run_animation(name, duration):
    """Run a named LED animation in a background thread."""
    global _led_animation_thread
    _stop_animation()
    _led_animation_stop.clear()

    led = get_led()
    if not led.is_support_led_function:
        return

    def _loop():
        start = time.time()
        while not _led_animation_stop.is_set():
            if duration and (time.time() - start) > duration:
                break
            if name == "rainbow":
                led.rainbowCycle(20)
            elif name == "breathing":
                led.rainbowbreathing(10)
            elif name == "blink":
                led.colorBlink(1)
            elif name == "chase":
                led.following(50)
            else:
                break
            time.sleep(0.01)
        led.colorBlink(0)

    _led_animation_thread = threading.Thread(target=_loop, daemon=True)
    _led_animation_thread.start()


# ---------------------------------------------------------------------------
# Battery LED monitor
# ---------------------------------------------------------------------------

BATTERY_LED_BRIGHTNESS = 25  # ~10% of max 255
BATTERY_MIN_V = 6.0   # 2S Li-ion empty
BATTERY_MAX_V = 8.4   # 2S Li-ion full
BATTERY_LED_INTERVAL = 30  # seconds between updates


def _voltage_to_rgb(voltage):
    """Map battery voltage to red-yellow-green gradient."""
    pct = max(0.0, min(1.0, (voltage - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V)))
    if pct < 0.5:
        # Red → Yellow
        t = pct / 0.5
        return (255, int(255 * t), 0)
    else:
        # Yellow → Green
        t = (pct - 0.5) / 0.5
        return (int(255 * (1 - t)), 255, 0)


def _battery_led_loop():
    """Background thread: update LEDs with battery level color."""
    while not _battery_led_stop.is_set():
        if _battery_led_active:
            try:
                adc = get_adc()
                raw = adc.read_adc(2)
                multiplier = 3 if adc.pcb_version == 1 else 2
                voltage = raw * multiplier

                r, g, b = _voltage_to_rgb(voltage)
                led = get_led()
                if led.is_support_led_function:
                    led.strip.set_led_brightness(BATTERY_LED_BRIGHTNESS)
                    for i in range(8):
                        led.strip.set_led_rgb_data(i, [r, g, b])
                    led.strip.show()
            except Exception:
                pass

        _battery_led_stop.wait(BATTERY_LED_INTERVAL)


# ---------------------------------------------------------------------------
# MJPEG streaming
# ---------------------------------------------------------------------------

def _mjpeg_generator(width=320, height=240, fps=10):
    """Generate MJPEG frames from rpicam-vid."""
    cmd = [
        "rpicam-vid", "--codec", "mjpeg",
        "--width", str(width), "--height", str(height),
        "--framerate", str(fps),
        "-o", "-", "--nopreview", "-t", "0",
    ]

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

    try:
        buf = b""
        while True:
            chunk = proc.stdout.read(4096)
            if not chunk:
                break
            buf += chunk

            # Find JPEG frames (SOI=FFD8, EOI=FFD9)
            while True:
                soi = buf.find(b"\xff\xd8")
                if soi == -1:
                    buf = b""
                    break
                eoi = buf.find(b"\xff\xd9", soi + 2)
                if eoi == -1:
                    buf = buf[soi:]
                    break
                frame = buf[soi:eoi + 2]
                buf = buf[eoi + 2:]
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                )
    finally:
        proc.terminate()
        proc.wait()


# ---------------------------------------------------------------------------
# Calibration analysis prompt — embedded in /auto/calibrate response so any
# Claude session can process the data without prior context about the car.
# ---------------------------------------------------------------------------

CALIBRATION_ANALYSIS_PROMPT = """
<context>
You are analyzing calibration data from a Freenove 4WD Smart Car with mecanum wheels.
The car has an ultrasonic distance sensor and camera mounted on a pan/tilt servo head,
3 IR line-tracking sensors underneath, and a battery. The pan servo's 90° position
SHOULD point straight ahead, but manufacturing variance means the true center is offset.

The car performed:
1. A coarse ultrasonic sweep (30°-150° in 10° steps) with a camera image at each angle
2. A fine ultrasonic sweep (1° steps around the coarse minimum) with camera images every 5°
3. Four seed movements (forward, backward, strafe_left, strafe_right) at speed=700 for 0.5s each,
   capturing before/after camera images, ultrasonic distances, IR readings, and sensor fusion results

The Pi's algorithm found the minimum ultrasonic distance and assumed perpendicular-to-wall = straight ahead.
This heuristic fails in corners, near shoe racks, curtains, or irregular surfaces. Your job is to do better.
</context>

<task>
Analyze ALL the provided sensor data to determine the car's true forward direction and motor behavior,
then POST corrections to /auto/calibrate/correct on the same host.

Perform these analysis steps in order:
</task>

<step_1_scene_understanding>
Examine scene_image (the resting photo before any movement).
Identify the environment: hallway, room corner, open space, cluttered area.
Note any objects that could produce misleading ultrasonic readings (soft furnishings absorb sound,
narrow objects give inconsistent returns, angled surfaces deflect rather than reflect).
</step_1_scene_understanding>

<step_2_coarse_sweep>
Examine servo_alignment.coarse_sweep (angle→distance map) alongside servo_alignment.coarse_images.

For each angle, the image shows what the sensor is pointed at.
Identify which angles show a flat, solid wall vs. clutter, furniture edges, or open space.
A flat wall produces a smooth distance curve with a clear minimum at the perpendicular angle.
Clutter or corners produce jagged or multi-modal distance profiles.

Determine which surface (if any) is a reliable calibration reference wall.
</step_2_coarse_sweep>

<step_3_fine_sweep>
Examine servo_alignment.fine_sweep and servo_alignment.fine_images.

The fine sweep has 1° resolution. Look for:
- The true minimum distance angle (this is the perpendicular to the nearest surface)
- The shape of the curve around the minimum: a broad U-shape suggests a flat wall (good reference);
  a sharp V-shape or noisy profile suggests a corner, pole, or irregular surface (unreliable)
- servo_alignment.servo_slop_deg: how many degrees share nearly the same minimum distance.
  High slop (>10°) on a flat wall is normal; high slop on clutter means the minimum is ambiguous.

If the reference surface is reliable, the true forward direction = fine sweep minimum angle.
pan_offset_deg = (true_forward_angle - 90).
</step_3_fine_sweep>

<step_4_seed_movements>
Examine each entry in bias_seed. Each contains:
- command: which direction the car was told to move
- before_image / after_image: visual confirmation of actual movement
- pre_distance_cm / post_distance_cm: ultrasonic before/after
- fusion: sensor-fused displacement estimate (dx_cm, dy_cm, d_theta_deg)
- infrared: IR line sensors (0=no line, 1=line detected)

For each seed movement, assess:
- Did the car move in the expected direction? (forward should decrease distance to front wall)
- How much lateral drift (dy_cm) and rotation (d_theta_deg) occurred?
- Is the fusion result consistent with the visual before/after difference?
- Any collisions or stuck conditions?

Systematic patterns reveal motor biases. For example, if forward consistently drifts left (positive dy_cm),
the right-side motors are stronger and need correction.
</step_4_seed_movements>

<step_5_compute_corrections>
Based on your analysis, determine:

1. pan_offset_deg: The corrected servo center offset. This is the most critical value.
   If you identified a reliable wall, use the perpendicular angle minus 90.
   If no reliable wall, use visual cues from images to estimate forward direction.
   If uncertain, prefer the algorithmic result in raw_calibration.computed_pan_offset_deg.

2. motor_biases (optional): Per-command corrections if seed movements revealed systematic drift.
   Only override if you have clear evidence. Each bias has:
   - lateral: cm/sec of lateral drift (positive = left). Counter observed drift.
   - rotation: deg/sec of unwanted rotation (positive = CCW). Counter observed rotation.
   - speed_scale: ratio of actual vs commanded speed. Usually 0.8-1.0.

3. speed_calibration_factor (optional): cm per (PWM_unit × second). Default is 0.007.
   Only adjust if fusion dx_cm values are consistently larger or smaller than expected
   for the commanded speed and duration.

4. rotation_calibration_factor (optional): degrees per (PWM_unit × second). Default is 0.09.
   Only adjust if rotation commands show systematic over/under-rotation.
</step_5_compute_corrections>

<output_format>
POST your corrections as JSON to /auto/calibrate/correct on the same host:

{
  "pan_offset_deg": <float, required>,
  "servo_slop_deg": <float, optional>,
  "motor_biases": {
    "<command_name>": {"lateral": <float>, "rotation": <float>, "speed_scale": <float>},
    ...
  },
  "speed_calibration_factor": <float, optional>,
  "rotation_calibration_factor": <float, optional>,
  "notes": "<your reasoning: what you identified in the images, why you chose this offset, confidence level>"
}

Always include a detailed notes field explaining your reasoning — this is the audit trail
for debugging calibration issues.
</output_format>

<constraints>
- pan_offset_deg is required. All other fields are optional.
- Only include motor_biases for commands where you have clear evidence of systematic drift.
- If the environment has no reliable reference wall, say so in notes and use the algorithmic result.
- Prefer conservative corrections — a small accurate offset is better than a large uncertain one.
- The notes field should mention which images/angles informed your decision and your confidence level.
</constraints>
""".strip()


CALIBRATION_V2_PROMPT = """
<context>
You are analyzing v2 calibration data from a Freenove 4WD Smart Car with mecanum wheels.
The car has an ultrasonic distance sensor and camera mounted on a pan/tilt servo head,
3 IR line-tracking sensors underneath, photoresistors (left/right), and a battery.

The car performed a comprehensive sweep-move-sweep calibration sequence:
1. Initial baseline sweep (no movement)
2. Forward movement then sweep
3. Backward movement then sweep
4. Strafe left movement then sweep
5. Strafe right movement then sweep

Each sweep consists of:
- Horizontal sweep: pan servo across range with tilt at 90 degrees (level)
- Vertical sweep: tilt servo across range with pan at 90 degrees (center)

At each position: camera image (base64 JPEG), ultrasonic distance (cm), IR sensors
(left/middle/right, 0=no line 1=line), photoresistors (left/right voltage), battery voltage.

Each sweep entry includes:
- label: identifies the sweep stage (e.g. "initial", "after_forward")
- preceded_by: the movement command that occurred before this sweep (null for initial)
- movement: details of the preceding movement (command, speed, duration, motor values, timestamp)
- timestamp: ISO 8601 UTC time when the sweep began
</context>

<task>
Analyze the sweep data to understand the car's environment and movement behavior,
then POST corrections to /auto/calibrate/correct on the same host.
</task>

<step_1_baseline_environment>
Examine the initial sweep (label="initial"). The horizontal sweep shows the scene
left-to-right; the vertical sweep shows floor-to-ceiling.

For each position, correlate the camera image with ultrasonic distance:
- Identify walls, obstacles, open space, and surface types
- Note which directions have reliable flat surfaces for calibration reference
- Identify objects that could produce misleading ultrasonic readings
  (soft furnishings absorb sound, narrow objects give inconsistent returns,
  angled surfaces deflect rather than reflect)
- Check photoresistor values for lighting conditions and asymmetry
- Check IR sensors for floor surface (line detection)
</step_1_baseline_environment>

<step_2_movement_analysis>
For each movement (forward, backward, strafe_left, strafe_right), compare
the post-movement sweep against the initial baseline:

- Ultrasonic distance changes across all pan angles reveal actual displacement direction
- Camera image shifts reveal visual displacement
- Asymmetric distance changes reveal drift and rotation
- IR sensor changes reveal if the car crossed any floor markings
- Battery voltage drop during sequence reveals power draw under load

For each movement, determine:
1. Did it move in the expected direction? (e.g. forward should decrease distance at pan=90)
2. How much lateral drift occurred? (unexpected distance changes at side angles)
3. How much unwanted rotation occurred? (asymmetric left/right changes)
4. Was the distance traveled proportional to commanded speed x duration?
</step_2_movement_analysis>

<step_3_servo_alignment>
The horizontal sweep provides pan angle to ultrasonic distance at each step.
The minimum distance angle in the initial sweep indicates perpendicular to nearest wall.

If a flat wall is visible in the images:
- The perpendicular angle = true forward direction
- pan_offset_deg = (perpendicular_angle - 90)

Cross-reference with the vertical sweep: if tilt=90 degrees shows floor rather than
straight ahead, the tilt servo may also need offset correction.

Validate with post-movement sweeps: after forward movement, distance at the
perpendicular angle should decrease. If a different angle shows the largest decrease,
the true forward direction may differ from the ultrasonic minimum.
</step_3_servo_alignment>

<step_4_compute_corrections>
Based on your analysis, determine:

1. pan_offset_deg (required): Corrected servo center offset from 90 degrees.
   Use the perpendicular angle from horizontal sweep, validated by movement consistency.

2. motor_biases (optional): Per-command corrections from movement analysis.
   Each bias has:
   - lateral: cm/sec drift (positive = left). Counter observed drift.
   - rotation: deg/sec unwanted rotation (positive = CCW). Counter observed rotation.
   - speed_scale: actual/commanded speed ratio. Usually 0.8-1.0.
   Only include for commands with clear systematic drift evidence.

3. speed_calibration_factor (optional): cm per (PWM x second). Default 0.007.
   Adjust if movements consistently over/under-shoot expected distances.

4. rotation_calibration_factor (optional): deg per (PWM x second). Default 0.09.
   Adjust if rotation commands show systematic over/under-rotation.
</step_4_compute_corrections>

<output_format>
POST corrections as JSON to /auto/calibrate/correct:

{
  "pan_offset_deg": <float, required>,
  "servo_slop_deg": <float, optional>,
  "motor_biases": {
    "<command>": {"lateral": <float>, "rotation": <float>, "speed_scale": <float>},
    ...
  },
  "speed_calibration_factor": <float, optional>,
  "rotation_calibration_factor": <float, optional>,
  "notes": "<reasoning: what images showed, why this offset, confidence level>"
}
</output_format>

<constraints>
- pan_offset_deg is required. All other fields optional.
- Only include motor_biases for commands with clear systematic drift evidence.
- Compare each movement's sweep against the initial baseline, not against each other.
- The initial sweep is the ground truth for the environment.
- Conservative corrections preferred: small accurate offsets beat large uncertain ones.
- Include detailed notes explaining which sweep positions informed each decision.
</constraints>
""".strip()


# ---------------------------------------------------------------------------
# Flask app
# ---------------------------------------------------------------------------

app = Flask(__name__)


@app.route("/")
def index():
    return jsonify({
        "name": "Freenove 4WD Smart Car API",
        "dashboard": "GET /dashboard",
        "endpoints": {
            "sensors": {
                "GET /sensors/ultrasonic": "Ultrasonic distance (cm)",
                "GET /sensors/adc": "Photoresistors + battery voltage",
                "GET /sensors/infrared": "IR line-tracking sensors",
                "GET /sensors/camera": "Capture JPEG image (?width=&height=)",
                "POST /sensors/batch": "Request multiple sensors {sensors: [...]}",
            },
            "control": {
                "POST /control/motors": "Set motor speeds {command, speed} or {left_upper, ...}",
                "POST /control/servos": "Set servo angles {pan, tilt, speed} or [{channel, angle}, ...]",
                "POST /control/leds": "Set LEDs {color, brightness, animation, index}",
                "POST /control/buzzer": "Buzzer {state, duration}",
                "POST /control/batch": "Batch commands [{type, ...}, ...]",
                "POST /control/stop": "Emergency stop all hardware",
            },
            "camera": {
                "GET /camera/stream": "Live MJPEG stream (?width=&height=&fps=)",
                "POST /camera/sweep": "Pan+tilt sweep with photos {step, pan_min, pan_max, ...}",
            },
            "autonomy": {
                "POST /auto/step": "Decision cycle — move, scan, recover, or stop",
                "POST /auto/scan": "Ultrasonic sweep → polar distance map + grid update",
                "GET /auto/state": "Read-only state (pose, flags, optional grid)",
                "POST /auto/configure": "Tune safety thresholds and calibration",
                "POST /auto/reset": "Reset grid and/or pose",
                "POST /auto/calibrate": "Rich sensor data collection — servo alignment + images + bias seeding (~70s)",
                "POST /auto/calibrate/correct": "Apply Claude's calibration corrections",
            },
            "calibration": {
                "POST /calibrate": "Single direction calibration {direction, speed, duration}",
                "POST /calibrate/all": "All mecanum directions {speed, duration}",
                "POST /calibrate/v2": "Sweep-move-sweep with full sensor collection at each position",
            },
            "system": {
                "POST /execute": "Run Python code {code: '...'}",
                "GET /system/status": "System info (uptime, Tailscale, battery, etc.)",
                "POST /system/reboot": "Reboot the Raspberry Pi",
                "POST /system/shutdown": "Shutdown the Raspberry Pi",
            },
        },
    })


# ---- Sensors ---------------------------------------------------------------

@app.route("/sensors/ultrasonic")
def sensor_ultrasonic():
    sensor = get_ultrasonic()
    distance = sensor.get_distance()
    return jsonify({"distance_cm": distance})


@app.route("/sensors/adc")
def sensor_adc():
    adc = get_adc()
    left = adc.read_adc(0)
    right = adc.read_adc(1)
    raw_power = adc.read_adc(2)
    multiplier = 3 if adc.pcb_version == 1 else 2
    battery = round(raw_power * multiplier, 2)
    return jsonify({
        "left_photoresistor_v": left,
        "right_photoresistor_v": right,
        "battery_voltage": battery,
        "battery_raw_v": raw_power,
    })


@app.route("/sensors/infrared")
def sensor_infrared():
    ir = get_infrared()
    left = ir.read_one_infrared(1)
    middle = ir.read_one_infrared(2)
    right = ir.read_one_infrared(3)
    return jsonify({
        "left": left,
        "middle": middle,
        "right": right,
    })


@app.route("/sensors/camera")
def sensor_camera():
    width = request.args.get("width", 320, type=int)
    height = request.args.get("height", 240, type=int)

    # Clamp resolution
    width = max(64, min(width, 2592))
    height = max(64, min(height, 1944))

    result = subprocess.run(
        [
            "rpicam-still", "-o", "-",
            "--width", str(width), "--height", str(height),
            "--timeout", "1500", "--nopreview",
            "--immediate",
        ],
        capture_output=True,
        timeout=10,
    )

    if result.returncode != 0:
        return jsonify({"error": "Camera capture failed", "detail": result.stderr.decode()}), 500

    return Response(result.stdout, mimetype="image/jpeg")


@app.route("/sensors/batch", methods=["POST"])
def sensor_batch():
    data = request.get_json(force=True)
    requested = data.get("sensors", [])
    results = {}

    sensor_map = {
        "ultrasonic": sensor_ultrasonic,
        "adc": sensor_adc,
        "infrared": sensor_infrared,
    }

    for name in requested:
        if name in sensor_map:
            resp = sensor_map[name]()
            results[name] = resp.get_json()
        elif name == "camera":
            results["camera"] = {"note": "Use GET /sensors/camera directly for image binary"}
        else:
            results[name] = {"error": f"Unknown sensor: {name}"}

    return jsonify(results)


# ---- Camera Stream ---------------------------------------------------------

@app.route("/camera/stream")
def camera_stream():
    width = request.args.get("width", 320, type=int)
    height = request.args.get("height", 240, type=int)
    fps = request.args.get("fps", 10, type=int)
    width = max(64, min(width, 1280))
    height = max(64, min(height, 960))
    fps = max(1, min(fps, 30))
    return Response(
        _mjpeg_generator(width, height, fps),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


# ---- Control ---------------------------------------------------------------

@app.route("/control/motors", methods=["POST"])
def control_motors():
    data = request.get_json(force=True)
    motor = get_motor()

    # Named command mode: {command: "forward", speed: 700}
    if "command" in data:
        speed = data.get("speed", 700)
        speed = max(0, min(speed, 4095))
        commands = {
            "forward":  (-speed, -speed, -speed, -speed),
            "backward": ( speed,  speed,  speed,  speed),
            "left":     ( speed,  speed, -speed, -speed),
            "right":    (-speed, -speed,  speed,  speed),
            "strafe_left":  ( speed, -speed, -speed,  speed),
            "strafe_right": (-speed,  speed,  speed, -speed),
            "rotate_cw":    (-speed, -speed,  speed,  speed),
            "rotate_ccw":   ( speed,  speed, -speed, -speed),
            "stop":     (0, 0, 0, 0),
        }
        cmd = data["command"]
        if cmd not in commands:
            return jsonify({"error": f"Unknown command: {cmd}", "valid": list(commands.keys())}), 400
        vals = commands[cmd]
        motor.set_motor_model(*vals)
        return jsonify({"status": "ok", "command": cmd, "speed": speed, "motors": vals})

    # Direct mode: {left_upper, left_lower, right_upper, right_lower}
    lu = max(-4095, min(4095, data.get("left_upper", 0)))
    ll = max(-4095, min(4095, data.get("left_lower", 0)))
    ru = max(-4095, min(4095, data.get("right_upper", 0)))
    rl = max(-4095, min(4095, data.get("right_lower", 0)))
    motor.set_motor_model(lu, ll, ru, rl)
    return jsonify({"status": "ok", "motors": [lu, ll, ru, rl]})


@app.route("/control/servos", methods=["POST"])
def control_servos():
    data = request.get_json(force=True)
    servo = get_servo()
    speed = data.get("speed", 0)  # deg/sec, 0 = instant
    results = []

    # Object mode: {pan: 90, tilt: 120, speed: 100}
    # Channel 0 = tilt (vertical), Channel 1 = pan (horizontal)
    if "pan" in data or "tilt" in data:
        if "pan" in data:
            angle = max(0, min(180, int(data["pan"])))
            if speed > 0:
                move_servo_smooth(1, angle, speed)
            else:
                servo.set_servo_pwm("1", angle)
                _servo_positions[1] = angle
            results.append({"channel": 1, "angle": angle})
        if "tilt" in data:
            angle = max(0, min(180, int(data["tilt"])))
            if speed > 0:
                move_servo_smooth(0, angle, speed)
            else:
                servo.set_servo_pwm("0", angle)
                _servo_positions[0] = angle
            results.append({"channel": 0, "angle": angle})
        return jsonify({"status": "ok", "servos": results, "speed": speed})

    # Array mode: [{channel: 0, angle: 90}, ...]
    if isinstance(data, list):
        for cmd in data:
            ch = int(cmd.get("channel", 0))
            angle = max(0, min(180, int(cmd.get("angle", 90))))
            cmd_speed = cmd.get("speed", speed)
            if cmd_speed > 0:
                move_servo_smooth(ch, angle, cmd_speed)
            else:
                servo.set_servo_pwm(str(ch), angle)
                _servo_positions[ch] = angle
            results.append({"channel": ch, "angle": angle})
        return jsonify({"status": "ok", "servos": results})

    return jsonify({"error": "Provide {pan, tilt} or [{channel, angle}, ...]"}), 400


@app.route("/control/leds", methods=["POST"])
def control_leds():
    global _battery_led_active
    data = request.get_json(force=True)
    led = get_led()

    if not led.is_support_led_function:
        return jsonify({"error": "LED function not supported with current board config"}), 500

    # Animation mode
    animation = data.get("animation")
    if animation:
        _battery_led_active = False
        duration = data.get("duration")
        _run_animation(animation, duration)
        return jsonify({"status": "ok", "animation": animation, "duration": duration})

    # Stop any running animation
    _stop_animation()

    # Brightness
    brightness = data.get("brightness")
    if brightness is not None:
        led.strip.set_led_brightness(max(0, min(255, int(brightness))))

    # Color mode
    color = data.get("color")
    if color and isinstance(color, list) and len(color) == 3:
        _battery_led_active = False
        r, g, b = [max(0, min(255, int(c))) for c in color]
        index = data.get("index")  # specific LED index (1-8) or None for all
        if index is not None:
            led.ledIndex(1 << (int(index) - 1), r, g, b)
            return jsonify({"status": "ok", "index": index, "color": [r, g, b]})
        else:
            for i in range(8):
                led.strip.set_led_rgb_data(i, [r, g, b])
            led.strip.show()
            return jsonify({"status": "ok", "color": [r, g, b], "all": True})

    # Off — return LED ownership to battery monitor
    if data.get("off"):
        led.colorBlink(0)
        _battery_led_active = True
        return jsonify({"status": "ok", "leds": "off", "battery_led": "resumed"})

    return jsonify({"status": "ok"})


BUZZER_VOLUME = 0.10  # 10% duty cycle

@app.route("/control/buzzer", methods=["POST"])
def control_buzzer():
    data = request.get_json(force=True)
    buzzer = get_buzzer()
    state = data.get("state", False)
    duration = data.get("duration")

    buzzer.value = BUZZER_VOLUME if state else 0

    if state and duration:
        def _off():
            time.sleep(float(duration))
            buzzer.value = 0
        threading.Thread(target=_off, daemon=True).start()
        return jsonify({"status": "ok", "buzzer": "on", "volume": BUZZER_VOLUME, "auto_off_sec": duration})

    return jsonify({"status": "ok", "buzzer": "on" if state else "off", "volume": BUZZER_VOLUME if state else 0})


@app.route("/control/batch", methods=["POST"])
def control_batch():
    data = request.get_json(force=True)
    commands = data.get("commands", data if isinstance(data, list) else [])
    results = []

    for cmd in commands:
        cmd_type = cmd.get("type")
        try:
            if cmd_type == "motor":
                motor = get_motor()
                if "command" in cmd:
                    speed = cmd.get("speed", 700)
                    mapping = {
                        "forward":  (-speed, -speed, -speed, -speed),
                        "backward": ( speed,  speed,  speed,  speed),
                        "left":     ( speed,  speed, -speed, -speed),
                        "right":    (-speed, -speed,  speed,  speed),
                        "stop":     (0, 0, 0, 0),
                    }
                    vals = mapping.get(cmd["command"], (0, 0, 0, 0))
                    motor.set_motor_model(*vals)
                    results.append({"type": "motor", "status": "ok", "command": cmd["command"]})
                else:
                    vals = (
                        cmd.get("left_upper", 0), cmd.get("left_lower", 0),
                        cmd.get("right_upper", 0), cmd.get("right_lower", 0),
                    )
                    motor.set_motor_model(*vals)
                    results.append({"type": "motor", "status": "ok", "motors": vals})

            elif cmd_type == "servo":
                servo = get_servo()
                ch = str(cmd.get("channel", 0))
                angle = max(0, min(180, int(cmd.get("angle", 90))))
                servo.set_servo_pwm(ch, angle)
                results.append({"type": "servo", "status": "ok", "channel": ch, "angle": angle})

            else:
                results.append({"type": cmd_type, "status": "error", "error": f"Unknown type: {cmd_type}"})

        except Exception as e:
            results.append({"type": cmd_type, "status": "error", "error": str(e)})

    return jsonify({"status": "ok", "results": results})


@app.route("/control/stop", methods=["POST"])
def control_stop():
    """Emergency stop -- kills all outputs."""
    stopped = []

    try:
        get_motor().set_motor_model(0, 0, 0, 0)
        stopped.append("motors")
    except Exception as e:
        stopped.append(f"motors (error: {e})")

    try:
        s = get_servo()
        s.set_servo_pwm("0", 90)
        s.set_servo_pwm("1", 90)
        _servo_positions[0] = 90
        _servo_positions[1] = 90
        stopped.append("servos (centered)")
    except Exception as e:
        stopped.append(f"servos (error: {e})")

    try:
        _stop_animation()
        get_led().colorBlink(0)
        stopped.append("leds")
    except Exception as e:
        stopped.append(f"leds (error: {e})")

    try:
        get_buzzer().value = 0
        stopped.append("buzzer")
    except Exception as e:
        stopped.append(f"buzzer (error: {e})")

    try:
        if _autonomy is not None:
            _autonomy.safety.stop_monitoring()
            _autonomy.safety.collision_flag = False
            _autonomy.safety.stuck_flag = False
            stopped.append("autonomy safety")
    except Exception as e:
        stopped.append(f"autonomy safety (error: {e})")

    return jsonify({"status": "ok", "stopped": stopped})


# ---- Autonomy --------------------------------------------------------------

@app.route("/auto/step", methods=["POST"])
def auto_step():
    """Core autonomy decision cycle — move, scan, recover, or stop."""
    data = request.get_json(force=True) if request.is_json else {}
    auto = get_autonomy()

    action = data.get("action")
    command = data.get("command")
    speed = data.get("speed", 700)
    duration = data.get("duration", 0.5)
    scan_params = data.get("scan_params")
    recover_maneuver = data.get("recover_maneuver")

    result = auto.execute_step(
        command=command,
        speed=speed,
        duration=duration,
        action=action,
        scan_params=scan_params,
        recover_maneuver=recover_maneuver,
    )

    return jsonify(result)


@app.route("/auto/scan", methods=["POST"])
def auto_scan():
    """Full ultrasonic sweep — returns polar distance map and updates grid."""
    data = request.get_json(force=True) if request.is_json else {}
    auto = get_autonomy()

    pan_min = data.get("pan_min", 30)
    pan_max = data.get("pan_max", 150)
    step = data.get("step", 5)
    settle_ms = data.get("settle_ms", 150)

    result = auto.perform_scan(
        pan_min=pan_min,
        pan_max=pan_max,
        step=step,
        settle_ms=settle_ms,
    )

    return jsonify(result)


@app.route("/auto/state")
def auto_state():
    """Read-only state: pose, safety flags, optional occupancy grid."""
    auto = get_autonomy()
    include_grid = request.args.get("include_grid", "false").lower() in ("true", "1", "yes")
    grid_radius = request.args.get("grid_radius", 20, type=int)
    grid_radius = max(5, min(50, grid_radius))

    return jsonify(auto.get_state(
        include_grid=include_grid,
        grid_radius=grid_radius,
    ))


@app.route("/auto/configure", methods=["POST"])
def auto_configure():
    """Tune safety thresholds and calibration values at runtime."""
    data = request.get_json(force=True) if request.is_json else {}
    auto = get_autonomy()

    updated = auto.configure(**data)

    if not updated:
        return jsonify({
            "status": "no_change",
            "valid_keys": [
                "collision_threshold_cm", "stuck_time_threshold",
                "stuck_spread_threshold", "speed_calibration",
                "rotation_calibration", "correction_enabled",
            ],
        })

    return jsonify({"status": "ok", "updated": updated})


@app.route("/auto/reset", methods=["POST"])
def auto_reset():
    """Reset occupancy grid and/or pose for fresh exploration."""
    data = request.get_json(force=True) if request.is_json else {}
    auto = get_autonomy()

    reset_grid = data.get("reset_grid", True)
    reset_pose = data.get("reset_pose", True)

    result = auto.reset(reset_grid=reset_grid, reset_pose=reset_pose)

    return jsonify({"status": "ok", **result})


@app.route("/auto/calibrate", methods=["POST"])
def auto_calibrate():
    """Run startup calibration: servo alignment + bias table seeding.

    Collects rich multi-sensor data (ultrasonic sweeps, camera images at
    each angle, IR readings, battery data) for Claude-based correction.
    Takes ~70 seconds. Run once per session or after battery swap.
    """
    auto = get_autonomy()

    # Capture a resting-state scene image before any movement
    scene_image = None
    try:
        jpeg = _capture_jpeg()
        if jpeg:
            scene_image = base64.b64encode(jpeg).decode()
    except Exception:
        pass

    # Capture IR baseline at rest
    ir_baseline = None
    try:
        ir = get_infrared()
        ir_baseline = {
            'left': ir.read_one_infrared(1),
            'center': ir.read_one_infrared(2),
            'right': ir.read_one_infrared(3),
        }
    except Exception:
        pass

    # Battery voltage at start
    battery_voltage = None
    try:
        adc = get_adc()
        raw = adc.read_adc(2)
        multiplier = 3 if adc.pcb_version == 1 else 2
        battery_voltage = round(raw * multiplier, 2)
    except Exception:
        pass

    # Step 1: Servo alignment (now includes camera images at sweep angles)
    servo_result = auto.servo_align.calibrate()

    # Step 2: Seed bias table with short movements — pass through full results
    seed_commands = ['forward', 'backward', 'strafe_left', 'strafe_right']
    seed_results = []
    for cmd in seed_commands:
        step = auto.execute_step(command=cmd, speed=700, duration=0.5)
        seed_results.append(step)
        time.sleep(0.5)

    # Save bias table after seeding
    auto.bias_table.save()

    return jsonify({
        'status': 'ok',
        'analysis_prompt': CALIBRATION_ANALYSIS_PROMPT,
        'servo_alignment': servo_result,
        'scene_image': scene_image,
        'ir_baseline': ir_baseline,
        'battery_voltage': battery_voltage,
        'bias_seed': seed_results,
        'bias_table': auto.bias_table.table,
        'raw_calibration': {
            'computed_pan_offset_deg': servo_result.get('pan_offset_deg', 0.0),
            'notes': 'Algorithmic result — submit to POST /auto/calibrate/correct for Claude correction',
        },
    })


@app.route("/auto/calibrate/correct", methods=["POST"])
def auto_calibrate_correct():
    """Accept Claude's corrections to calibration parameters.

    Claude analyzes the rich sensor data from /auto/calibrate and pushes
    back corrected parameters: servo offset, motor biases, speed/rotation
    calibration factors.
    """
    data = request.get_json(force=True)
    auto = get_autonomy()

    if 'pan_offset_deg' not in data:
        return jsonify({
            'error': 'pan_offset_deg is required',
            'accepted_fields': [
                'pan_offset_deg', 'servo_slop_deg', 'motor_biases',
                'speed_calibration_factor', 'rotation_calibration_factor', 'notes',
            ],
        }), 400

    applied = {}

    # Apply servo alignment corrections
    servo_slop = data.get('servo_slop_deg')
    auto.servo_align.apply_corrections(data['pan_offset_deg'], servo_slop_deg=servo_slop)
    applied['pan_offset_deg'] = auto.servo_align.pan_offset_deg
    if servo_slop is not None:
        applied['servo_slop_deg'] = auto.servo_align.servo_slop_deg

    # Apply motor bias overrides
    motor_biases = data.get('motor_biases')
    biases_updated = []
    if motor_biases and isinstance(motor_biases, dict):
        for cmd, bias_data in motor_biases.items():
            if cmd in auto.bias_table.COMMANDS and isinstance(bias_data, dict):
                current = auto.bias_table.get_bias(cmd)
                for key in ('lateral', 'rotation', 'speed_scale'):
                    if key in bias_data:
                        current[key] = float(bias_data[key])
                auto.bias_table.table[cmd] = current
                biases_updated.append(cmd)
        if biases_updated:
            auto.bias_table.save()
    applied['motor_biases_updated'] = biases_updated

    # Apply speed calibration factor
    if 'speed_calibration_factor' in data:
        auto.pose.SPEED_CALIBRATION_FACTOR = float(data['speed_calibration_factor'])
        applied['speed_calibration_factor'] = auto.pose.SPEED_CALIBRATION_FACTOR

    # Apply rotation calibration factor
    if 'rotation_calibration_factor' in data:
        auto.pose.ROTATION_CALIBRATION_FACTOR = float(data['rotation_calibration_factor'])
        applied['rotation_calibration_factor'] = auto.pose.ROTATION_CALIBRATION_FACTOR

    return jsonify({
        'status': 'ok',
        'applied': applied,
        'notes': data.get('notes', ''),
        'current_state': {
            'servo_offset': auto.servo_align.pan_offset_deg,
            'servo_slop': auto.servo_align.servo_slop_deg,
            'bias_table': auto.bias_table.table,
            'pose_calibration': {
                'speed': auto.pose.SPEED_CALIBRATION_FACTOR,
                'rotation': auto.pose.ROTATION_CALIBRATION_FACTOR,
            },
        },
    })


# ---- Calibration -----------------------------------------------------------

MECANUM_DIRECTIONS = {
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
}


def _collect_all_sensors():
    """Read ultrasonic, infrared, and ADC sensors in one call."""
    result = {}

    try:
        result['ultrasonic_cm'] = get_ultrasonic().get_distance()
    except Exception:
        result['ultrasonic_cm'] = None

    try:
        ir = get_infrared()
        result['infrared'] = {
            'left': ir.read_one_infrared(1),
            'middle': ir.read_one_infrared(2),
            'right': ir.read_one_infrared(3),
        }
    except Exception:
        result['infrared'] = None

    try:
        adc = get_adc()
        raw_power = adc.read_adc(2)
        multiplier = 3 if adc.pcb_version == 1 else 2
        result['adc'] = {
            'left_photoresistor_v': adc.read_adc(0),
            'right_photoresistor_v': adc.read_adc(1),
            'battery_voltage': round(raw_power * multiplier, 2),
        }
    except Exception:
        result['adc'] = None

    return result


def _perform_calibration_sweep(pan_min, pan_max, pan_step,
                               tilt_min, tilt_max, tilt_step,
                               settle, img_width, img_height):
    """Perform horizontal and vertical sweeps collecting all sensor data.

    Horizontal sweep: pan across pan_min..pan_max with tilt held at 90 (level).
    Vertical sweep: tilt across tilt_min..tilt_max with pan held at 90 (center).

    At each position captures: camera image, ultrasonic, IR, and ADC.
    """
    horizontal = []
    # Horizontal sweep: tilt level, pan across range
    move_servo_smooth(0, 90, 200)
    time.sleep(settle)

    for pan in range(pan_min, pan_max + 1, pan_step):
        move_servo_smooth(1, pan, 200)
        time.sleep(settle)

        sensors = _collect_all_sensors()
        img = _capture_jpeg(img_width, img_height)

        horizontal.append({
            'pan': pan,
            'tilt': 90,
            'image': base64.b64encode(img).decode() if img else None,
            **sensors,
        })

    vertical = []
    # Vertical sweep: pan center, tilt across range
    move_servo_smooth(1, 90, 200)
    time.sleep(settle)

    for tilt in range(tilt_min, tilt_max + 1, tilt_step):
        move_servo_smooth(0, tilt, 200)
        time.sleep(settle)

        sensors = _collect_all_sensors()
        img = _capture_jpeg(img_width, img_height)

        vertical.append({
            'pan': 90,
            'tilt': tilt,
            'image': base64.b64encode(img).decode() if img else None,
            **sensors,
        })

    # Return to center
    move_servo_smooth(0, 90, 200)
    move_servo_smooth(1, 90, 200)

    return {'horizontal': horizontal, 'vertical': vertical}


def _capture_jpeg(width=320, height=240):
    """Capture a JPEG and return raw bytes."""
    result = subprocess.run(
        [
            "rpicam-still", "-o", "-",
            "--width", str(width), "--height", str(height),
            "--timeout", "1500", "--nopreview", "--immediate",
        ],
        capture_output=True,
        timeout=10,
    )
    if result.returncode != 0:
        return None
    return result.stdout


def _run_calibration_step(direction, speed, duration):
    """Run a single calibration: before photo -> move -> after photo."""
    motor = get_motor()
    multipliers = MECANUM_DIRECTIONS[direction]
    motor_vals = tuple(int(m * speed) for m in multipliers)

    before_img = _capture_jpeg()
    before_distance = None
    try:
        before_distance = get_ultrasonic().get_distance()
    except Exception:
        pass

    motor.set_motor_model(*motor_vals)
    time.sleep(duration)
    motor.set_motor_model(0, 0, 0, 0)
    time.sleep(0.3)  # settle

    after_img = _capture_jpeg()
    after_distance = None
    try:
        after_distance = get_ultrasonic().get_distance()
    except Exception:
        pass

    return {
        "direction": direction,
        "speed": speed,
        "duration_sec": duration,
        "motor_values": motor_vals,
        "before_image": base64.b64encode(before_img).decode() if before_img else None,
        "after_image": base64.b64encode(after_img).decode() if after_img else None,
        "before_distance_cm": before_distance,
        "after_distance_cm": after_distance,
        "distance_delta_cm": round(after_distance - before_distance, 1) if (before_distance and after_distance) else None,
    }


@app.route("/calibrate", methods=["POST"])
def calibrate_single():
    """Calibrate one direction: capture before/after images around a timed move."""
    data = request.get_json(force=True)
    direction = data.get("direction", "forward")
    speed = max(500, min(4095, data.get("speed", 700)))
    duration = max(0.2, min(10.0, data.get("duration", 1.0)))

    if direction not in MECANUM_DIRECTIONS:
        return jsonify({
            "error": f"Unknown direction: {direction}",
            "valid": list(MECANUM_DIRECTIONS.keys()),
        }), 400

    result = _run_calibration_step(direction, speed, duration)
    return jsonify({"status": "ok", **result})


@app.route("/calibrate/all", methods=["POST"])
def calibrate_all():
    """Run calibration for all mecanum directions sequentially."""
    data = request.get_json(force=True) if request.is_json else {}
    speed = max(500, min(4095, data.get("speed", 700)))
    duration = max(0.2, min(10.0, data.get("duration", 1.0)))
    pause = max(0.5, min(5.0, data.get("pause", 1.5)))
    directions = data.get("directions", list(MECANUM_DIRECTIONS.keys()))

    results = []
    for direction in directions:
        if direction not in MECANUM_DIRECTIONS:
            results.append({"direction": direction, "error": "unknown"})
            continue
        step = _run_calibration_step(direction, speed, duration)
        results.append(step)
        time.sleep(pause)

    return jsonify({"status": "ok", "calibration": results})


@app.route("/calibrate/v2", methods=["POST"])
def calibrate_v2():
    """V2 calibration: sweep-move-sweep sequence with full sensor collection.

    Performs horizontal and vertical servo sweeps at each stage, collecting
    camera images, ultrasonic distance, IR, and ADC readings at every position.

    Sequence: initial sweep -> forward -> sweep -> backward -> sweep ->
              strafe_left -> sweep -> strafe_right -> sweep
    """
    data = request.get_json(force=True) if request.is_json else {}

    # Sweep parameters
    pan_min = max(0, min(180, data.get('pan_min', 30)))
    pan_max = max(0, min(180, data.get('pan_max', 150)))
    pan_step = max(5, min(45, data.get('pan_step', 15)))
    tilt_min = max(0, min(180, data.get('tilt_min', 60)))
    tilt_max = max(0, min(180, data.get('tilt_max', 120)))
    tilt_step = max(5, min(45, data.get('tilt_step', 15)))
    settle = max(0.05, min(2.0, data.get('settle', 0.2)))
    img_width = max(64, min(640, data.get('image_width', 160)))
    img_height = max(64, min(480, data.get('image_height', 120)))

    # Movement parameters
    speed = max(500, min(4095, data.get('speed', 700)))
    duration = max(0.2, min(5.0, data.get('duration', 0.5)))
    pause = max(0.2, min(3.0, data.get('pause', 0.5)))

    motor = get_motor()
    started_at = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())

    sweep_args = (pan_min, pan_max, pan_step, tilt_min, tilt_max, tilt_step,
                  settle, img_width, img_height)

    sweeps = []

    # Initial sweep (baseline, no movement)
    sweeps.append({
        'label': 'initial',
        'preceded_by': None,
        'movement': None,
        'timestamp': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        **_perform_calibration_sweep(*sweep_args),
    })

    # Movement + sweep sequence
    movements = ['forward', 'backward', 'strafe_left', 'strafe_right']
    for cmd in movements:
        multipliers = MECANUM_DIRECTIONS[cmd]
        motor_vals = tuple(int(m * speed) for m in multipliers)

        move_timestamp = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
        motor.set_motor_model(*motor_vals)
        time.sleep(duration)
        motor.set_motor_model(0, 0, 0, 0)
        time.sleep(pause)

        sweeps.append({
            'label': f'after_{cmd}',
            'preceded_by': cmd,
            'movement': {
                'command': cmd,
                'speed': speed,
                'duration': duration,
                'motor_values': motor_vals,
                'timestamp': move_timestamp,
            },
            'timestamp': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            **_perform_calibration_sweep(*sweep_args),
        })

    completed_at = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())

    return jsonify({
        'status': 'ok',
        'started_at': started_at,
        'completed_at': completed_at,
        'parameters': {
            'pan_range': [pan_min, pan_max],
            'pan_step': pan_step,
            'tilt_range': [tilt_min, tilt_max],
            'tilt_step': tilt_step,
            'settle_sec': settle,
            'image_resolution': [img_width, img_height],
            'move_speed': speed,
            'move_duration': duration,
            'move_pause': pause,
        },
        'analysis_prompt': CALIBRATION_V2_PROMPT,
        'sweeps': sweeps,
    })


# ---- Camera Sweep ----------------------------------------------------------

@app.route("/camera/sweep", methods=["POST"])
def camera_sweep():
    """Sweep pan and tilt servos, capturing a photo at each position."""
    data = request.get_json(force=True) if request.is_json else {}

    pan_min = max(0, min(180, data.get("pan_min", 0)))
    pan_max = max(0, min(180, data.get("pan_max", 180)))
    tilt_min = max(0, min(180, data.get("tilt_min", 0)))
    tilt_max = max(0, min(180, data.get("tilt_max", 180)))
    step = max(1, min(45, data.get("step", 5)))
    width = max(64, min(640, data.get("width", 160)))
    height = max(64, min(480, data.get("height", 120)))
    settle = max(0.05, min(2.0, data.get("settle", 0.3)))

    servo = get_servo()
    images = []

    tilt_positions = list(range(tilt_min, tilt_max + 1, step))
    pan_positions = list(range(pan_min, pan_max + 1, step))

    for row_idx, tilt in enumerate(tilt_positions):
        servo.set_servo_pwm("0", tilt)
        time.sleep(settle)

        # Serpentine: reverse pan direction on odd rows
        row_pans = pan_positions if row_idx % 2 == 0 else list(reversed(pan_positions))

        for pan in row_pans:
            servo.set_servo_pwm("1", pan)
            time.sleep(settle)

            img = _capture_jpeg(width, height)
            images.append({
                "pan": pan,
                "tilt": tilt,
                "image": base64.b64encode(img).decode() if img else None,
            })

    # Return servos to center (ch0=tilt, ch1=pan)
    servo.set_servo_pwm("0", 90)  # tilt center
    servo.set_servo_pwm("1", 90)  # pan center
    _servo_positions[0] = 90
    _servo_positions[1] = 90

    return jsonify({
        "status": "ok",
        "total_images": len(images),
        "captured": sum(1 for i in images if i["image"]),
        "step_deg": step,
        "pan_range": [pan_min, pan_max],
        "tilt_range": [tilt_min, tilt_max],
        "resolution": [width, height],
        "images": images,
    })


# ---- System ----------------------------------------------------------------

@app.route("/execute", methods=["POST"])
def execute_code():
    """Run arbitrary Python code. Local network only."""
    data = request.get_json(force=True)
    code = data.get("code", "")

    if not code:
        return jsonify({"error": "No code provided"}), 400

    namespace = {
        "get_motor": get_motor,
        "get_servo": get_servo,
        "get_ultrasonic": get_ultrasonic,
        "get_infrared": get_infrared,
        "get_adc": get_adc,
        "get_led": get_led,
        "get_buzzer": get_buzzer,
        "time": time,
    }

    stdout_capture = io.StringIO()
    old_stdout = sys.stdout

    try:
        sys.stdout = stdout_capture
        exec(code, namespace)  # noqa: S102 - intentional, local network only
        sys.stdout = old_stdout
        output = stdout_capture.getvalue()
        result = namespace.get("result", None)
        return jsonify({"status": "ok", "output": output, "result": result})
    except Exception:
        sys.stdout = old_stdout
        return jsonify({
            "status": "error",
            "output": stdout_capture.getvalue(),
            "error": traceback.format_exc(),
        }), 500


@app.route("/system/status")
def system_status():
    """Return system info: uptime, temperature, Tailscale, battery."""
    info = {}

    try:
        info["uptime"] = subprocess.check_output(["uptime", "-p"], text=True).strip()
    except Exception:
        info["uptime"] = "unknown"

    try:
        temp = subprocess.check_output(["vcgencmd", "measure_temp"], text=True).strip()
        info["cpu_temp"] = temp.replace("temp=", "")
    except Exception:
        info["cpu_temp"] = "unknown"

    try:
        ts = subprocess.check_output(["tailscale", "status", "--json"], text=True, timeout=5)
        import json
        ts_data = json.loads(ts)
        self_node = ts_data.get("Self", {})
        info["tailscale"] = {
            "status": "connected" if ts_data.get("BackendState") == "Running" else ts_data.get("BackendState", "unknown"),
            "ip": self_node.get("TailscaleIPs", [None])[0],
            "hostname": self_node.get("HostName", "unknown"),
        }
    except Exception:
        info["tailscale"] = {"status": "not running"}

    try:
        throttled = subprocess.check_output(["vcgencmd", "get_throttled"], text=True).strip()
        info["throttled"] = throttled
    except Exception:
        pass

    try:
        adc = get_adc()
        raw = adc.read_adc(2)
        multiplier = 3 if adc.pcb_version == 1 else 2
        info["battery_voltage"] = round(raw * multiplier, 2)
    except Exception:
        pass

    return jsonify(info)


@app.route("/system/reboot", methods=["POST"])
def system_reboot():
    """Reboot the Raspberry Pi."""
    # Stop all hardware first
    try:
        control_stop()
    except Exception:
        pass

    def _reboot():
        time.sleep(1)
        subprocess.run(["sudo", "reboot"])

    threading.Thread(target=_reboot, daemon=True).start()
    return jsonify({"status": "ok", "message": "Rebooting in 1 second..."})


@app.route("/system/shutdown", methods=["POST"])
def system_shutdown():
    """Shutdown the Raspberry Pi."""
    try:
        control_stop()
    except Exception:
        pass

    def _shutdown():
        time.sleep(1)
        subprocess.run(["sudo", "shutdown", "-h", "now"])

    threading.Thread(target=_shutdown, daemon=True).start()
    return jsonify({"status": "ok", "message": "Shutting down in 1 second..."})


# ---- Dashboard -------------------------------------------------------------

DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Car Control</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,-apple-system,sans-serif;background:#1a1a2e;color:#e0e0e0;padding:12px}
h1{font-size:1.3rem;color:#0ff;margin-bottom:12px;text-align:center}
h2{font-size:0.95rem;color:#0ff;margin-bottom:8px;border-bottom:1px solid #333;padding-bottom:4px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;max-width:900px;margin:0 auto}
.card{background:#16213e;border-radius:10px;padding:14px;border:1px solid #1a3a5c}
.full{grid-column:1/-1}
.cam-wrap{text-align:center}
.cam-wrap img{border-radius:8px;width:100%;max-width:480px;background:#000}
.sensor-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:6px}
.sensor-val{background:#0d1b2a;border-radius:6px;padding:8px;text-align:center}
.sensor-val .label{font-size:0.7rem;color:#888;text-transform:uppercase}
.sensor-val .value{font-size:1.2rem;font-weight:700;color:#0f0}
.dpad{display:grid;grid-template-columns:repeat(5,1fr);grid-template-rows:repeat(5,1fr);gap:4px;width:220px;height:220px;margin:0 auto}
.dpad button{border:none;border-radius:6px;background:#1a3a5c;color:#e0e0e0;font-size:0.65rem;cursor:pointer;display:flex;align-items:center;justify-content:center;transition:background 0.15s}
.dpad button:hover{background:#0ff;color:#000}
.dpad button:active{background:#0a0;color:#fff}
.dpad .empty{background:transparent;cursor:default}
.dpad .stop-btn{background:#c00;color:#fff;font-weight:700;font-size:0.8rem}
.dpad .stop-btn:hover{background:#f00}
.slider-row{display:flex;align-items:center;gap:8px;margin:6px 0}
.slider-row label{width:40px;font-size:0.8rem;color:#888}
.slider-row input[type=range]{flex:1;accent-color:#0ff}
.slider-row .val{width:36px;text-align:right;font-size:0.85rem;font-weight:600}
.btn{border:none;border-radius:6px;padding:8px 14px;cursor:pointer;font-size:0.8rem;transition:background 0.15s}
.btn-primary{background:#0ff;color:#000}.btn-primary:hover{background:#0ae}
.btn-danger{background:#c00;color:#fff}.btn-danger:hover{background:#f00}
.btn-warn{background:#fa0;color:#000}.btn-warn:hover{background:#fc0}
.btn-sm{padding:5px 10px;font-size:0.75rem}
.btn-group{display:flex;gap:6px;flex-wrap:wrap;margin-top:6px}
.color-row{display:flex;align-items:center;gap:8px;margin:6px 0}
.color-row input[type=color]{width:50px;height:32px;border:none;border-radius:4px;cursor:pointer}
.speed-input{width:70px;background:#0d1b2a;border:1px solid #333;border-radius:4px;color:#e0e0e0;padding:4px;font-size:0.85rem;text-align:center}
.sys-info{font-size:0.8rem;color:#888;line-height:1.6}
.sys-info span{color:#0f0}
.ir-dots{display:flex;gap:8px;justify-content:center;margin:4px 0}
.ir-dot{width:20px;height:20px;border-radius:50%;border:2px solid #555}
.ir-dot.active{background:#0f0;border-color:#0f0}
</style>
</head>
<body>

<h1>Freenove 4WD Smart Car</h1>

<div class="grid">

<!-- Camera -->
<div class="card full cam-wrap">
<h2>Live Camera</h2>
<img id="cam" src="/camera/stream?width=480&height=360&fps=10" alt="Camera Feed">
</div>

<!-- Sensors -->
<div class="card">
<h2>Sensors</h2>
<div class="sensor-grid">
  <div class="sensor-val"><div class="label">Distance</div><div class="value" id="s-dist">--</div></div>
  <div class="sensor-val"><div class="label">Battery</div><div class="value" id="s-batt">--</div></div>
  <div class="sensor-val"><div class="label">CPU Temp</div><div class="value" id="s-temp">--</div></div>
  <div class="sensor-val"><div class="label">Light L</div><div class="value" id="s-ll">--</div></div>
  <div class="sensor-val"><div class="label">Light R</div><div class="value" id="s-lr">--</div></div>
  <div class="sensor-val"><div class="label">IR</div><div class="value"><div class="ir-dots"><div class="ir-dot" id="ir-l"></div><div class="ir-dot" id="ir-m"></div><div class="ir-dot" id="ir-r"></div></div></div></div>
</div>
<div class="btn-group"><button class="btn btn-sm btn-primary" onclick="refreshSensors()">Refresh</button><label style="font-size:0.75rem;display:flex;align-items:center;gap:4px"><input type="checkbox" id="auto-refresh" checked> Auto</label></div>
</div>

<!-- Motor Control -->
<div class="card">
<h2>Motors</h2>
<div style="display:flex;align-items:center;gap:8px;margin-bottom:6px">
<label style="font-size:0.8rem;color:#888">Speed:</label>
<input type="range" id="motor-speed" min="500" max="4095" value="700" style="flex:1;accent-color:#0ff">
<span id="motor-speed-val" style="font-size:0.85rem;width:40px;text-align:right">700</span>
</div>
<div class="dpad">
  <button onclick="mc('diagonal_fl')">&#8598;FL</button>
  <button onclick="mc('forward')">&#8593;FWD</button>
  <button onclick="mc('diagonal_fr')">&#8599;FR</button>
  <button onclick="mc('strafe_left')">&#8592;STR</button>
  <div class="empty"></div>

  <button onclick="mc('strafe_left')">&#8592;L</button>
  <button onclick="mc('forward')">&#8593;</button>
  <button onclick="mc('strafe_right')">&#8594;R</button>
  <button onclick="mc('rotate_ccw')">&#8634;CCW</button>
  <button class="stop-btn" onclick="mc('stop')">STOP</button>

  <button onclick="mc('rotate_cw')">&#8635;CW</button>
  <button onclick="mc('left')">&#8630;TL</button>
  <button onclick="mc('backward')">&#8595;</button>
  <button onclick="mc('right')">&#8631;TR</button>
  <div class="empty"></div>

  <button onclick="mc('diagonal_bl')">&#8601;BL</button>
  <button onclick="mc('backward')">&#8595;BWD</button>
  <button onclick="mc('diagonal_br')">&#8600;BR</button>
  <div class="empty"></div>
  <div class="empty"></div>
</div>
</div>

<!-- Servos -->
<div class="card">
<h2>Servos</h2>
<div class="slider-row">
  <label>Pan</label>
  <input type="range" id="servo-pan" min="0" max="180" value="90" oninput="servoMove()">
  <span class="val" id="pan-val">90</span>
</div>
<div class="slider-row">
  <label>Tilt</label>
  <input type="range" id="servo-tilt" min="0" max="180" value="90" oninput="servoMove()">
  <span class="val" id="tilt-val">90</span>
</div>
<div class="btn-group">
  <button class="btn btn-sm btn-primary" onclick="servoCenter()">Center</button>
</div>
</div>

<!-- LEDs -->
<div class="card">
<h2>LEDs</h2>
<div class="color-row">
  <input type="color" id="led-color" value="#ff0000">
  <button class="btn btn-sm btn-primary" onclick="setLedColor()">Set Color</button>
  <button class="btn btn-sm btn-danger" onclick="ledOff()">Off</button>
</div>
<div class="slider-row">
  <label>Bright</label>
  <input type="range" id="led-bright" min="0" max="255" value="128" oninput="document.getElementById('bright-val').textContent=this.value">
  <span class="val" id="bright-val">128</span>
</div>
<div class="btn-group">
  <button class="btn btn-sm btn-primary" onclick="ledAnim('rainbow')">Rainbow</button>
  <button class="btn btn-sm btn-primary" onclick="ledAnim('breathing')">Breathe</button>
  <button class="btn btn-sm btn-primary" onclick="ledAnim('chase')">Chase</button>
  <button class="btn btn-sm btn-primary" onclick="ledAnim('blink')">Blink</button>
</div>
</div>

<!-- Buzzer -->
<div class="card">
<h2>Buzzer</h2>
<div class="btn-group">
  <button class="btn btn-primary" onmousedown="buzzOn()" onmouseup="buzzOff()" ontouchstart="buzzOn()" ontouchend="buzzOff()">Hold to Buzz</button>
  <button class="btn btn-sm btn-primary" onclick="buzzBeep(0.2)">Beep</button>
  <button class="btn btn-sm btn-primary" onclick="buzzBeep(0.5)">Long Beep</button>
</div>
<p style="font-size:0.7rem;color:#666;margin-top:6px">Volume: 10% (hardcoded PWM)</p>
</div>

<!-- System -->
<div class="card">
<h2>System</h2>
<div class="sys-info" id="sys-info">Loading...</div>
<div class="btn-group" style="margin-top:10px">
  <button class="btn btn-sm btn-primary" onclick="stopAll()">Stop All</button>
  <button class="btn btn-sm btn-warn" onclick="if(confirm('Reboot the Pi?'))sysCmd('reboot')">Reboot</button>
  <button class="btn btn-sm btn-danger" onclick="if(confirm('Shutdown the Pi?'))sysCmd('shutdown')">Shutdown</button>
</div>
</div>

</div>

<script>
const API='';
function post(url,body){return fetch(API+url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}).then(r=>r.json())}
function get(url){return fetch(API+url).then(r=>r.json())}

// Motor
document.getElementById('motor-speed').oninput=function(){document.getElementById('motor-speed-val').textContent=this.value};
function mc(cmd){post('/control/motors',{command:cmd,speed:parseInt(document.getElementById('motor-speed').value)})}

// Servo - debounced
let servoTimer=null;
function servoMove(){
  document.getElementById('pan-val').textContent=document.getElementById('servo-pan').value;
  document.getElementById('tilt-val').textContent=document.getElementById('servo-tilt').value;
  clearTimeout(servoTimer);
  servoTimer=setTimeout(()=>{
    post('/control/servos',{pan:parseInt(document.getElementById('servo-pan').value),tilt:parseInt(document.getElementById('servo-tilt').value)});
  },50);
}
function servoCenter(){
  document.getElementById('servo-pan').value=90;
  document.getElementById('servo-tilt').value=90;
  servoMove();
}

// LEDs
function hexToRgb(hex){const r=parseInt(hex.slice(1,3),16),g=parseInt(hex.slice(3,5),16),b=parseInt(hex.slice(5,7),16);return[r,g,b]}
function setLedColor(){
  const c=hexToRgb(document.getElementById('led-color').value);
  const b=parseInt(document.getElementById('led-bright').value);
  post('/control/leds',{color:c,brightness:b});
}
function ledOff(){post('/control/leds',{off:true})}
function ledAnim(name){post('/control/leds',{animation:name,duration:30})}

// Buzzer
function buzzOn(){post('/control/buzzer',{state:true})}
function buzzOff(){post('/control/buzzer',{state:false})}
function buzzBeep(d){post('/control/buzzer',{state:true,duration:d})}

// Sensors
async function refreshSensors(){
  try{
    const [us,adc,ir]=await Promise.all([get('/sensors/ultrasonic'),get('/sensors/adc'),get('/sensors/infrared')]);
    document.getElementById('s-dist').textContent=us.distance_cm.toFixed(1)+'cm';
    document.getElementById('s-batt').textContent=adc.battery_voltage.toFixed(1)+'V';
    document.getElementById('s-ll').textContent=adc.left_photoresistor_v.toFixed(2)+'V';
    document.getElementById('s-lr').textContent=adc.right_photoresistor_v.toFixed(2)+'V';
    document.getElementById('ir-l').className='ir-dot'+(ir.left?' active':'');
    document.getElementById('ir-m').className='ir-dot'+(ir.middle?' active':'');
    document.getElementById('ir-r').className='ir-dot'+(ir.right?' active':'');
  }catch(e){console.error('sensor refresh error',e)}
  try{
    const sys=await get('/system/status');
    document.getElementById('s-temp').textContent=sys.cpu_temp||'--';
    let html='Uptime: <span>'+sys.uptime+'</span><br>';
    if(sys.tailscale)html+='Tailscale: <span>'+(sys.tailscale.status||'--')+'</span>'+(sys.tailscale.ip?' ('+sys.tailscale.ip+')':'')+'<br>';
    if(sys.throttled)html+='Throttle: <span>'+sys.throttled+'</span>';
    document.getElementById('sys-info').innerHTML=html;
  }catch(e){}
}

// Auto-refresh
setInterval(()=>{if(document.getElementById('auto-refresh').checked)refreshSensors()},2000);
refreshSensors();

// System
function stopAll(){post('/control/stop')}
function sysCmd(cmd){post('/system/'+cmd).then(r=>alert(r.message||'Command sent'))}

// Keyboard controls
document.addEventListener('keydown',e=>{
  if(e.target.tagName==='INPUT')return;
  const map={w:'forward',s:'backward',a:'strafe_left',d:'strafe_right',q:'rotate_ccw',e:'rotate_cw',ArrowUp:'forward',ArrowDown:'backward',ArrowLeft:'left',ArrowRight:'right',' ':'stop'};
  if(map[e.key])mc(map[e.key]);
});
document.addEventListener('keyup',e=>{
  if(e.target.tagName==='INPUT')return;
  if(['w','s','a','d','q','e','ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(e.key))mc('stop');
});
</script>
</body>
</html>"""


@app.route("/dashboard")
def dashboard():
    return Response(DASHBOARD_HTML, mimetype="text/html")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Start battery LED monitor
    threading.Thread(target=_battery_led_loop, daemon=True).start()

    print("Starting Freenove Car API server on port 5000...")
    print("Dashboard: http://0.0.0.0:5000/dashboard")
    print("API: http://0.0.0.0:5000/")
    app.run(host="0.0.0.0", port=5000, threaded=True)

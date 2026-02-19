# Freenove 4WD Smart Car REST API Specification

**Base URL:** `http://<pi-ip>:5000`
**Server:** Flask (threaded)
**Hardware:** Freenove FNK0043 4WD Smart Car Kit, Raspberry Pi 3A+

---

## Sensors

### GET /sensors/ultrasonic

Returns the distance reading from the HC-SR04 ultrasonic sensor (mounted on servo head).

**Response:**
```json
{
  "distance_cm": 79.7
}
```

| Field | Type | Description |
|-------|------|-------------|
| `distance_cm` | float | Distance to nearest obstacle in centimeters |

---

### GET /sensors/adc

Returns analog readings from the PCF8591/ADS7830 ADC: two photoresistors and battery voltage.

**Response:**
```json
{
  "left_photoresistor_v": 2.52,
  "right_photoresistor_v": 2.74,
  "battery_voltage": 7.56,
  "battery_raw_v": 2.52
}
```

| Field | Type | Description |
|-------|------|-------------|
| `left_photoresistor_v` | float | Left photoresistor voltage (0-3.3V, higher = darker) |
| `right_photoresistor_v` | float | Right photoresistor voltage (0-3.3V, higher = darker) |
| `battery_voltage` | float | Battery voltage after multiplier (PCB v1: x3, v2: x2) |
| `battery_raw_v` | float | Raw ADC reading for power channel |

---

### GET /sensors/infrared

Returns the three IR line-tracking sensor states (0 = no line, 1 = line detected).

**Response:**
```json
{
  "left": 0,
  "middle": 0,
  "right": 0
}
```

| Field | Type | Description |
|-------|------|-------------|
| `left` | int (0/1) | Left IR sensor (GPIO14) |
| `middle` | int (0/1) | Middle IR sensor (GPIO15) |
| `right` | int (0/1) | Right IR sensor (GPIO23) |

---

### GET /sensors/camera

Captures a JPEG image from the OV5647 camera via `rpicam-still`.

**Query Parameters:**

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `width` | int | 320 | 64-2592 | Image width in pixels |
| `height` | int | 240 | 64-1944 | Image height in pixels |

**Response:** Raw JPEG binary (`Content-Type: image/jpeg`)

**Example:**
```
GET /sensors/camera?width=640&height=480
```

---

### POST /sensors/batch

Request multiple sensor readings in a single call.

**Request Body:**
```json
{
  "sensors": ["ultrasonic", "adc", "infrared"]
}
```

| Field | Type | Description |
|-------|------|-------------|
| `sensors` | string[] | Array of sensor names: `ultrasonic`, `adc`, `infrared`, `camera` |

**Response:**
```json
{
  "ultrasonic": {"distance_cm": 79.7},
  "adc": {"left_photoresistor_v": 2.52, "right_photoresistor_v": 2.74, "battery_voltage": 7.56, "battery_raw_v": 2.52},
  "infrared": {"left": 0, "middle": 0, "right": 0}
}
```

> **Note:** Camera returns `{"note": "Use GET /sensors/camera directly for image binary"}` in batch mode since binary data cannot be embedded in JSON.

---

## Control

### POST /control/motors

Set motor speeds using named commands or direct per-wheel control.

**Named Command Mode:**
```json
{
  "command": "forward",
  "speed": 1500
}
```

| Field | Type | Default | Range | Description |
|-------|------|---------|-------|-------------|
| `command` | string | required | see below | Named movement command |
| `speed` | int | 1500 | 0-4095 | Motor speed (PWM duty) |

**Available commands:** `forward`, `backward`, `left`, `right`, `strafe_left`, `strafe_right`, `rotate_cw`, `rotate_ccw`, `stop`

**Direct Mode:**
```json
{
  "left_upper": 1500,
  "left_lower": 1500,
  "right_upper": 1500,
  "right_lower": 1500
}
```

| Field | Type | Default | Range | Description |
|-------|------|---------|-------|-------------|
| `left_upper` | int | 0 | -4095 to 4095 | Front-left motor (negative = reverse) |
| `left_lower` | int | 0 | -4095 to 4095 | Rear-left motor |
| `right_upper` | int | 0 | -4095 to 4095 | Front-right motor |
| `right_lower` | int | 0 | -4095 to 4095 | Rear-right motor |

**Response:**
```json
{
  "status": "ok",
  "command": "forward",
  "speed": 1500,
  "motors": [1500, 1500, 1500, 1500]
}
```

---

### POST /control/servos

Set servo angles using named positions or per-channel array. Supports optional `speed` parameter for smooth interpolated movement.

**Object Mode:**
```json
{
  "pan": 90,
  "tilt": 120,
  "speed": 60
}
```

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `pan` | int | 0-180 | Horizontal servo (PCA9685 channel 9, software ch 1) |
| `tilt` | int | 0-180 | Vertical servo (PCA9685 channel 8, software ch 0) |
| `speed` | int | 0-500 | Movement speed in degrees/sec. 0 = instant jump (default). |

**Array Mode:**
```json
[
  {"channel": 0, "angle": 90},
  {"channel": 1, "angle": 120}
]
```

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `channel` | int | 0-7 | Servo channel on PCA9685 (mapped to channels 8-15) |
| `angle` | int | 0-180 | Target angle in degrees |

**Response:**
```json
{
  "status": "ok",
  "servos": [{"channel": 0, "angle": 90}, {"channel": 1, "angle": 120}]
}
```

---

### POST /control/leds

Control the 8 WS2812 addressable RGB LEDs.

**Solid Color (all LEDs):**
```json
{
  "color": [255, 0, 0],
  "brightness": 100
}
```

**Solid Color (single LED):**
```json
{
  "color": [0, 255, 0],
  "index": 3
}
```

**Animation:**
```json
{
  "animation": "rainbow",
  "duration": 10
}
```

**Turn Off:**
```json
{
  "off": true
}
```

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `color` | int[3] | [0-255, 0-255, 0-255] | RGB color array |
| `brightness` | int | 0-255 | Global LED brightness |
| `index` | int | 1-8 | Target specific LED (omit for all) |
| `animation` | string | see below | Start a named animation |
| `duration` | float | null = infinite | Animation duration in seconds |
| `off` | bool | true | Turn all LEDs off |

**Available animations:** `rainbow`, `breathing`, `blink`, `chase`

---

### POST /control/buzzer

Activate the buzzer (hardcoded to 10% volume via PWM on GPIO17).

**Request Body:**
```json
{
  "state": true,
  "duration": 0.5
}
```

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `state` | bool | false | Turn buzzer on/off |
| `duration` | float | null | Auto-off after N seconds (omit for manual control) |

**Response:**
```json
{
  "status": "ok",
  "buzzer": "on",
  "volume": 0.1,
  "auto_off_sec": 0.5
}
```

> Volume is hardcoded to 10% duty cycle. No volume parameter is accepted.

---

### POST /control/batch

Execute multiple motor and servo commands in sequence.

**Request Body:**
```json
{
  "commands": [
    {"type": "motor", "command": "forward", "speed": 1000},
    {"type": "servo", "channel": 0, "angle": 45},
    {"type": "servo", "channel": 1, "angle": 120}
  ]
}
```

**Motor command fields:** Same as `/control/motors` plus `"type": "motor"`
**Servo command fields:** Same as array mode of `/control/servos` plus `"type": "servo"`

**Response:**
```json
{
  "status": "ok",
  "results": [
    {"type": "motor", "status": "ok", "command": "forward"},
    {"type": "servo", "status": "ok", "channel": "0", "angle": 45},
    {"type": "servo", "status": "ok", "channel": "1", "angle": 120}
  ]
}
```

---

### POST /control/stop

Emergency stop: kills all motors, centers servos, turns off LEDs, silences buzzer.

**Request Body:** `{}` (empty JSON object)

**Response:**
```json
{
  "status": "ok",
  "stopped": ["motors", "servos (centered)", "leds", "buzzer"]
}
```

---

## Calibration

### POST /calibrate

Run a single calibration step: capture before image + ultrasonic, drive in one direction, capture after image + ultrasonic.

**Request Body:**
```json
{
  "direction": "forward",
  "speed": 1500,
  "duration": 1.0
}
```

| Field | Type | Default | Range | Description |
|-------|------|---------|-------|-------------|
| `direction` | string | "forward" | see below | Mecanum direction |
| `speed` | int | 1500 | 500-4095 | Motor speed |
| `duration` | float | 1.0 | 0.2-10.0 | Drive duration in seconds |

**Available directions:** `forward`, `backward`, `left`, `right`, `strafe_left`, `strafe_right`, `rotate_cw`, `rotate_ccw`, `diagonal_fl`, `diagonal_fr`, `diagonal_bl`, `diagonal_br`

**Response:**
```json
{
  "status": "ok",
  "direction": "forward",
  "speed": 1500,
  "duration_sec": 1.0,
  "motor_values": [1500, 1500, 1500, 1500],
  "before_image": "<base64-encoded JPEG>",
  "after_image": "<base64-encoded JPEG>",
  "before_distance_cm": 79.7,
  "after_distance_cm": 45.2,
  "distance_delta_cm": -34.5
}
```

---

### POST /calibrate/all

Run calibration for all mecanum directions sequentially with pauses between each.

**Request Body:**
```json
{
  "speed": 1500,
  "duration": 1.0,
  "pause": 1.5,
  "directions": ["forward", "backward", "strafe_left", "strafe_right"]
}
```

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `speed` | int | 1500 | Motor speed (500-4095) |
| `duration` | float | 1.0 | Drive duration per direction (0.2-10.0s) |
| `pause` | float | 1.5 | Pause between directions (0.5-5.0s) |
| `directions` | string[] | all 12 | Subset of directions to test |

**Response:**
```json
{
  "status": "ok",
  "calibration": [
    {"direction": "forward", "speed": 1500, "duration_sec": 1.0, "motor_values": [1500,1500,1500,1500], "before_image": "...", "after_image": "...", "before_distance_cm": 79.7, "after_distance_cm": 45.2, "distance_delta_cm": -34.5},
    {"direction": "backward", "...": "..."}
  ]
}
```

---

### POST /calibrate/v2

Comprehensive sweep-move-sweep calibration sequence. At each stage, performs horizontal and vertical servo sweeps collecting camera images, ultrasonic distance, IR, and ADC readings at every position.

**Sequence:**
1. Initial baseline sweep (no movement)
2. Forward → sweep
3. Backward → sweep
4. Strafe left → sweep
5. Strafe right → sweep

Each sweep consists of:
- **Horizontal sweep:** pan servo across range with tilt held at 90° (level)
- **Vertical sweep:** tilt servo across range with pan held at 90° (center)

**Request Body:**
```json
{
  "pan_min": 30, "pan_max": 150, "pan_step": 15,
  "tilt_min": 60, "tilt_max": 120, "tilt_step": 15,
  "settle": 0.2,
  "image_width": 160, "image_height": 120,
  "speed": 1500, "duration": 0.5, "pause": 0.5
}
```

| Field | Type | Default | Range | Description |
|-------|------|---------|-------|-------------|
| `pan_min` | int | 30 | 0-180 | Horizontal sweep start angle |
| `pan_max` | int | 150 | 0-180 | Horizontal sweep end angle |
| `pan_step` | int | 15 | 5-45 | Degrees between horizontal stops |
| `tilt_min` | int | 60 | 0-180 | Vertical sweep start angle |
| `tilt_max` | int | 150 | 0-180 | Vertical sweep end angle |
| `tilt_step` | int | 15 | 5-45 | Degrees between vertical stops |
| `settle` | float | 0.2 | 0.05-2.0 | Servo settle time per position (seconds) |
| `image_width` | int | 160 | 64-640 | Capture width in pixels |
| `image_height` | int | 120 | 64-480 | Capture height in pixels |
| `speed` | int | 1500 | 500-4095 | Motor PWM speed for movements |
| `duration` | float | 0.5 | 0.2-5.0 | Movement duration (seconds) |
| `pause` | float | 0.5 | 0.2-3.0 | Settle time after motor stop (seconds) |

**Response:**
```json
{
  "status": "ok",
  "started_at": "2026-02-19T01:29:46Z",
  "completed_at": "2026-02-19T01:30:40Z",
  "parameters": {
    "pan_range": [30, 150],
    "pan_step": 15,
    "tilt_range": [60, 120],
    "tilt_step": 15,
    "settle_sec": 0.2,
    "image_resolution": [160, 120],
    "move_speed": 1500,
    "move_duration": 0.5,
    "move_pause": 0.5
  },
  "analysis_prompt": "<structured prompt for Claude correction — see below>",
  "sweeps": [
    {
      "label": "initial",
      "preceded_by": null,
      "movement": null,
      "timestamp": "2026-02-19T01:29:46Z",
      "horizontal": [
        {
          "pan": 30, "tilt": 90,
          "image": "<base64 JPEG>",
          "ultrasonic_cm": 120.3,
          "infrared": {"left": 0, "middle": 1, "right": 0},
          "adc": {"left_photoresistor_v": 2.5, "right_photoresistor_v": 2.7, "battery_voltage": 7.8}
        }
      ],
      "vertical": [
        {
          "pan": 90, "tilt": 60,
          "image": "<base64 JPEG>",
          "ultrasonic_cm": 85.1,
          "infrared": {"left": 0, "middle": 1, "right": 0},
          "adc": {"left_photoresistor_v": 2.5, "right_photoresistor_v": 2.7, "battery_voltage": 7.8}
        }
      ]
    },
    {
      "label": "after_forward",
      "preceded_by": "forward",
      "movement": {
        "command": "forward",
        "speed": 1500,
        "duration": 0.5,
        "motor_values": [-1500, -1500, -1500, -1500],
        "timestamp": "2026-02-19T01:29:57Z"
      },
      "timestamp": "2026-02-19T01:29:58Z",
      "horizontal": ["..."],
      "vertical": ["..."]
    }
  ]
}
```

| Field | Type | Description |
|-------|------|-------------|
| `started_at` | string | ISO 8601 UTC timestamp when calibration began |
| `completed_at` | string | ISO 8601 UTC timestamp when calibration finished |
| `parameters` | object | Echo of resolved parameters (after clamping) |
| `analysis_prompt` | string | Structured prompt instructing Claude how to analyze sweeps and POST corrections to `/auto/calibrate/correct` |
| `sweeps` | array | Array of 5 sweep results (initial + 4 post-movement) |

**Sweep entry fields:**

| Field | Type | Description |
|-------|------|-------------|
| `label` | string | Stage identifier: `initial`, `after_forward`, `after_backward`, `after_strafe_left`, `after_strafe_right` |
| `preceded_by` | string\|null | Movement command that occurred before this sweep (`null` for initial) |
| `movement` | object\|null | Details of preceding movement (`null` for initial) |
| `timestamp` | string | ISO 8601 UTC time when the sweep began |
| `horizontal` | array | Data points from horizontal sweep (pan across range, tilt at 90°) |
| `vertical` | array | Data points from vertical sweep (tilt across range, pan at 90°) |

**Data point fields (each entry in horizontal/vertical arrays):**

| Field | Type | Description |
|-------|------|-------------|
| `pan` | int | Pan servo angle at this position |
| `tilt` | int | Tilt servo angle at this position |
| `image` | string\|null | Base64-encoded JPEG, or `null` if capture failed |
| `ultrasonic_cm` | float\|null | Distance reading, or `null` if sensor failed |
| `infrared` | object\|null | `{left, middle, right}` IR line sensor states (0/1) |
| `adc` | object\|null | `{left_photoresistor_v, right_photoresistor_v, battery_voltage}` |

> **Runtime:** With default parameters (pan 30-150 step 15, tilt 60-120 step 15): 9 horizontal + 5 vertical = 14 positions per sweep × 5 sweeps = 70 captures. At ~2s per capture, expect ~2-3 minutes total.

> **Payload size:** ~650KB with default parameters (70 images at ~8KB each + metadata).

> **Analysis workflow:** Same as `/auto/calibrate` — analyze the sweep data, then POST corrections to `/auto/calibrate/correct`. The analysis prompt covers baseline environment mapping, per-movement displacement analysis, servo alignment detection, and motor bias computation.

---

## System

### POST /execute

Run arbitrary Python code on the Pi. Hardware getter functions are available in the execution namespace.

**Request Body:**
```json
{
  "code": "result = get_ultrasonic().get_distance()"
}
```

| Field | Type | Description |
|-------|------|-------------|
| `code` | string | Python code to execute |

**Available in namespace:** `get_motor()`, `get_servo()`, `get_ultrasonic()`, `get_infrared()`, `get_adc()`, `get_led()`, `get_buzzer()`, `time`

Set `result` variable in your code to return a value in the response.

**Response (success):**
```json
{
  "status": "ok",
  "output": "printed text from stdout",
  "result": 79.7
}
```

**Response (error):**
```json
{
  "status": "error",
  "output": "any stdout before the error",
  "error": "Traceback (most recent call last):\n  ..."
}
```

---

## Camera

### GET /camera/stream

Live MJPEG video stream from the OV5647 camera via `rpicam-vid`.

**Query Parameters:**

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `width` | int | 320 | 64-1280 | Frame width |
| `height` | int | 240 | 64-960 | Frame height |
| `fps` | int | 10 | 1-30 | Target framerate |

**Response:** `multipart/x-mixed-replace` MJPEG stream

**Example:** `<img src="http://<pi-ip>:5000/camera/stream?width=480&height=360&fps=10">`

---

### POST /camera/sweep

Pan+tilt servo sweep capturing a photo at each position (serpentine scan pattern).

**Request Body:**
```json
{
  "pan_min": 0, "pan_max": 180,
  "tilt_min": 0, "tilt_max": 180,
  "step": 5,
  "width": 160, "height": 120,
  "settle": 0.3
}
```

| Field | Type | Default | Range | Description |
|-------|------|---------|-------|-------------|
| `pan_min` | int | 0 | 0-180 | Pan start angle |
| `pan_max` | int | 180 | 0-180 | Pan end angle |
| `tilt_min` | int | 0 | 0-180 | Tilt start angle |
| `tilt_max` | int | 180 | 0-180 | Tilt end angle |
| `step` | int | 5 | 1-45 | Degrees between captures |
| `width` | int | 160 | 64-640 | Image width |
| `height` | int | 120 | 64-480 | Image height |
| `settle` | float | 0.3 | 0.05-2.0 | Servo settle time between positions (seconds) |

**Response:**
```json
{
  "status": "ok",
  "total_images": 1369,
  "captured": 1369,
  "step_deg": 5,
  "pan_range": [0, 180],
  "tilt_range": [0, 180],
  "resolution": [160, 120],
  "images": [
    {"pan": 0, "tilt": 0, "image": "<base64 JPEG>"},
    {"pan": 5, "tilt": 0, "image": "<base64 JPEG>"}
  ]
}
```

---

### GET /system/status

Returns system information: uptime, CPU temperature, Tailscale status, battery voltage, and throttle state.

**Response:**
```json
{
  "uptime": "up 55 minutes",
  "cpu_temp": "38.6'C",
  "tailscale": {"status": "connected", "ip": "100.x.x.x", "hostname": "rasberrypi"},
  "throttled": "throttled=0x50005",
  "battery_voltage": 7.53
}
```

---

### POST /system/reboot

Reboot the Raspberry Pi. Stops all hardware first.

**Request Body:** `{}` (empty)

**Response:**
```json
{"status": "ok", "message": "Rebooting in 1 second..."}
```

---

### POST /system/shutdown

Shutdown the Raspberry Pi. Stops all hardware first.

**Request Body:** `{}` (empty)

**Response:**
```json
{"status": "ok", "message": "Shutting down in 1 second..."}
```

---

## Autonomy

Autonomous navigation endpoints for Claude-as-brain driving. The Pi handles real-time safety (collision avoidance, stuck detection) via a background thread; Claude handles strategy via these HTTP endpoints.

### POST /auto/step

Core decision cycle — one HTTP call per decision. Supports movement commands, scans, recovery, and stop.

**Movement Mode (default):**
```json
{
  "command": "forward",
  "speed": 1500,
  "duration": 0.5
}
```

**Scan Mode:**
```json
{
  "action": "scan",
  "scan_params": {"pan_min": 30, "pan_max": 150, "step": 5, "settle_ms": 150}
}
```

**Recovery Mode:**
```json
{
  "action": "recover",
  "recover_maneuver": "back_and_rotate"
}
```

**Stop Mode:**
```json
{
  "action": "stop"
}
```

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `command` | string | "forward" | Movement command (same names as `/control/motors`) |
| `speed` | int | 500 | Motor PWM speed (0-4095) |
| `duration` | float | 0.5 | Movement duration in seconds (0.1-10.0) |
| `action` | string | null | Override: `"scan"`, `"recover"`, or `"stop"` |
| `scan_params` | object | null | Parameters for scan action (see `/auto/scan`) |
| `recover_maneuver` | string | "back_and_rotate" | Recovery maneuver name |

**Response (movement mode):**
```json
{
  "command": "forward",
  "speed": 1500,
  "requested_duration": 0.5,
  "actual_duration": 0.487,
  "pre_distance_cm": 85.3,
  "post_distance_cm": 62.1,
  "infrared": {"left": 0, "middle": 0, "right": 0},
  "battery_voltage": 7.52,
  "collision": false,
  "stuck": false,
  "pose": {"x_cm": 0.0, "y_cm": 5.1, "heading_deg": 90.0},
  "fusion": {
    "dx_cm": 24.15,
    "dy_cm": 0.08,
    "d_theta_deg": 0.3,
    "weights": {"visual": 0.45, "ultrasonic": 0.35, "dead_reckoning": 0.2},
    "fused": true
  },
  "calibration_warning": false,
  "bias_stale": false
}
```

When self-correction is enabled (default), movement responses include additional fields:

| Field | Type | Description |
|-------|------|-------------|
| `fusion` | object | Fused displacement estimate combining visual odometry, ultrasonic reference, and dead reckoning with per-source weights |
| `calibration_warning` | bool | True if any bias exceeds soft guardrails (rotation >8 deg/sec or speed scale <0.6 or >1.4) |
| `bias_stale` | bool | True if battery voltage dropped >0.5V from calibration baseline |

**Available commands:** `forward`, `backward`, `left`, `right`, `strafe_left`, `strafe_right`, `rotate_cw`, `rotate_ccw`, `diagonal_fl`, `diagonal_fr`, `diagonal_bl`, `diagonal_br`, `stop`

**Recovery maneuvers:** `back_up`, `back_and_rotate`, `wiggle_free`, `full_retreat`, `strafe_escape_left`, `strafe_escape_right`

---

### POST /auto/scan

Full ultrasonic sweep — rotates the pan servo through a range of angles, reading distance at each position. Updates the internal occupancy grid.

**Request Body:**
```json
{
  "pan_min": 30,
  "pan_max": 150,
  "step": 5,
  "settle_ms": 150
}
```

| Field | Type | Default | Range | Description |
|-------|------|---------|-------|-------------|
| `pan_min` | int | 30 | 0-180 | Start pan angle (0=right, 180=left) |
| `pan_max` | int | 150 | 0-180 | End pan angle |
| `step` | int | 5 | 1-45 | Degrees between readings |
| `settle_ms` | int | 150 | 50-2000 | Servo settle time (ms) |

**Response:**
```json
{
  "readings": [
    {"pan_angle": 30, "distance_cm": 145.2},
    {"pan_angle": 35, "distance_cm": 142.8},
    "..."
  ],
  "pose": {"x_cm": 0.0, "y_cm": 0.0, "heading_deg": 90.0},
  "count": 25
}
```

> **Timing:** A 30°-150° sweep at 5° steps with 150ms settle takes ~9 seconds.

---

### GET /auto/state

Read-only snapshot of current autonomy state.

**Query Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `include_grid` | bool | false | Include ASCII occupancy grid |
| `grid_radius` | int | 20 | Half-width of text grid (5-50) |

**Response:**
```json
{
  "pose": {"x_cm": 12.3, "y_cm": 45.6, "heading_deg": 90.0},
  "collision_flag": false,
  "stuck_flag": false
}
```

**Response (with grid):**
```json
{
  "pose": {"x_cm": 12.3, "y_cm": 45.6, "heading_deg": 90.0},
  "collision_flag": false,
  "stuck_flag": false,
  "grid": "??????????????????????????????????\n??????????????..............??????\n??????????????..............??????\n??????????????......^.......??????\n??????????????..............??????\n??????????####..............######\n??????????????????????????????????"
}
```

Grid characters: `#` = wall/occupied, `.` = free space, `?` = unknown, `^`/`>`/`v`/`<` = car (heading arrow).

---

### POST /auto/configure

Update safety thresholds and calibration values at runtime without restarting.

**Request Body:**
```json
{
  "collision_threshold_cm": 20,
  "speed_calibration": 0.008
}
```

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `collision_threshold_cm` | float | 15 | Emergency stop distance (cm) |
| `stuck_time_threshold` | float | 1.5 | Seconds of movement before stuck check |
| `stuck_spread_threshold` | float | 2.0 | Min distance spread (cm) to not be stuck |
| `speed_calibration` | float | 0.007 | cm per (PWM unit * second) |
| `rotation_calibration` | float | 0.09 | degrees per (PWM unit * second) |
| `correction_enabled` | bool | true | Enable/disable self-correction (sensor fusion + bias) |

**Response:**
```json
{
  "status": "ok",
  "updated": {
    "collision_threshold_cm": 20.0,
    "speed_calibration": 0.008
  }
}
```

---

### POST /auto/reset

Reset occupancy grid and/or dead-reckoning pose for fresh exploration.

**Request Body:**
```json
{
  "reset_grid": true,
  "reset_pose": true
}
```

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `reset_grid` | bool | true | Clear occupancy grid to all-unknown |
| `reset_pose` | bool | true | Reset position to origin (0,0) facing north |

**Response:**
```json
{
  "status": "ok",
  "grid": "reset",
  "pose": "reset",
  "safety_flags": "cleared"
}
```

---

### POST /auto/calibrate

Rich sensor data collection for Claude-based calibration correction. Performs servo alignment detection with camera images at each sweep angle, then seeds the motor bias table with short movements capturing full sensor data. Takes ~70 seconds.

**Request Body:** `{}` (empty JSON object)

**Response:**
```json
{
  "status": "ok",
  "analysis_prompt": "<structured prompt with XML tags — see below>",
  "servo_alignment": {
    "status": "ok",
    "pan_offset_deg": -2.0,
    "servo_slop_deg": 3.0,
    "true_center_angle": 88,
    "wall_distance_cm": 62.3,
    "coarse_sweep": {"30": 150.2, "40": 145.1, "50": 138.9, "...": "..."},
    "fine_sweep": {"60": 125.3, "61": 124.8, "62": 124.2, "...": "..."},
    "coarse_images": {"30": "<base64 JPEG>", "40": "<base64 JPEG>", "...": "..."},
    "fine_images": {"60": "<base64 JPEG>", "65": "<base64 JPEG>", "...": "..."}
  },
  "scene_image": "<base64 JPEG of resting-state photo>",
  "ir_baseline": {"left": 1, "center": 1, "right": 0},
  "battery_voltage": 7.8,
  "bias_seed": [
    {
      "command": "forward",
      "speed": 1500,
      "requested_duration": 0.5,
      "actual_duration": 0.487,
      "pre_distance_cm": 35.6,
      "post_distance_cm": 30.2,
      "infrared": {"left": 1, "middle": 1, "right": 0},
      "battery_voltage": 7.8,
      "collision": false,
      "stuck": false,
      "pose": {"x_cm": 0.0, "y_cm": 5.1, "heading_deg": 90.0},
      "fusion": {"dx_cm": 24.3, "dy_cm": 0.1, "d_theta_deg": 0.5, "weights": {"visual": 0.45, "ultrasonic": 0.35, "dead_reckoning": 0.2}, "fused": true},
      "before_image": "<base64 JPEG>",
      "after_image": "<base64 JPEG>",
      "calibration_warning": false,
      "bias_stale": false
    }
  ],
  "bias_table": {
    "forward": {"lateral": 0.2, "rotation": 1.1, "speed_scale": 0.92},
    "backward": {"lateral": -0.1, "rotation": -0.5, "speed_scale": 0.88}
  },
  "raw_calibration": {
    "computed_pan_offset_deg": -2.0,
    "notes": "Algorithmic result — submit to POST /auto/calibrate/correct for Claude correction"
  }
}
```

| Field | Type | Description |
|-------|------|-------------|
| `analysis_prompt` | string | Structured prompt (XML tags) instructing Claude how to analyze the data and POST corrections to `/auto/calibrate/correct`. Covers scene understanding, sweep analysis, seed movement interpretation, and the exact correction JSON schema. |
| `servo_alignment` | object | Servo centering results including `coarse_images` (13 images at 10° steps) and `fine_images` (every 5° around minimum) |
| `scene_image` | string\|null | Base64 JPEG of the resting scene before any movement |
| `ir_baseline` | object\|null | IR sensor readings at rest (left, center, right) |
| `battery_voltage` | float\|null | Battery voltage at calibration start |
| `bias_seed` | array | Full step results from seed movements (forward, backward, strafe_left, strafe_right) including before/after images, distances, IR, fusion |
| `bias_table` | object | Learned motor correction factors per command |
| `raw_calibration` | object | The algorithmic offset result, flagged for Claude correction |

> **Payload size:** ~515KB total (34 images at ~15KB each + metadata).

> **When to run:** Once per session, or after battery swap. The car should be facing a wall within 200cm.

> **Claude-in-the-loop workflow:**
> 1. Call `POST /auto/calibrate` → receive rich sensor data with images
> 2. Analyze images + ultrasonic profiles + IR + fusion results
> 3. Determine true forward direction, wall locations, motor drift
> 4. Call `POST /auto/calibrate/correct` → push corrected parameters

---

### POST /auto/calibrate/correct

Accept Claude's corrections to calibration parameters after analyzing the rich data from `/auto/calibrate`.

**Request Body:**
```json
{
  "pan_offset_deg": -14.0,
  "servo_slop_deg": 16,
  "motor_biases": {
    "forward": {"lateral": -2.1, "rotation": -0.5, "speed_scale": 0.95},
    "strafe_left": {"lateral": -5.0, "rotation": -1.2, "speed_scale": 1.0}
  },
  "speed_calibration_factor": 0.05,
  "rotation_calibration_factor": 0.8,
  "notes": "Wall identified at 90° in coarse sweep. Shoe rack at 45° produced misleading short distance."
}
```

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `pan_offset_deg` | float | Yes | Corrected servo center offset from 90° |
| `servo_slop_deg` | float | No | Corrected slop value |
| `motor_biases` | object | No | Per-command bias overrides (keys: lateral, rotation, speed_scale) |
| `speed_calibration_factor` | float | No | Override PoseTracker cm per (PWM × second) |
| `rotation_calibration_factor` | float | No | Override PoseTracker degrees per (PWM × second) |
| `notes` | string | No | Claude's reasoning trail for debugging |

**Response:**
```json
{
  "status": "ok",
  "applied": {
    "pan_offset_deg": -14.0,
    "servo_slop_deg": 16.0,
    "motor_biases_updated": ["forward", "strafe_left"],
    "speed_calibration_factor": 0.05,
    "rotation_calibration_factor": 0.8
  },
  "notes": "Wall identified at 90° in coarse sweep...",
  "current_state": {
    "servo_offset": -14.0,
    "servo_slop": 16.0,
    "bias_table": {"forward": {"lateral": -2.1, "rotation": -0.5, "speed_scale": 0.95}},
    "pose_calibration": {"speed": 0.05, "rotation": 0.8}
  }
}
```

| Field | Type | Description |
|-------|------|-------------|
| `applied` | object | Summary of what was applied |
| `notes` | string | Echo of Claude's reasoning |
| `current_state` | object | Full current calibration state for verification |

> **Effects:** Servo physically moves to corrected center. Motor bias overrides persist to `motor_bias.json`. Speed/rotation factors update PoseTracker immediately. Subsequent `/auto/step` calls use the corrected values.

---

## Dashboard

### GET /dashboard

Interactive web-based control panel. Features:
- Live MJPEG camera feed
- Motor direction pad with all mecanum directions + speed slider
- Pan/tilt servo sliders
- LED color picker, brightness slider, and animation buttons
- Buzzer hold-to-buzz and beep buttons
- Auto-refreshing sensor readout (ultrasonic, ADC, infrared, CPU temp)
- System info panel (uptime, Tailscale, throttle state)
- System controls (Stop All, Reboot, Shutdown)
- Keyboard controls (WASD + QE + Arrows + Space)

---

## Mecanum Direction Reference

The motor value signs for each mecanum direction (multiply by speed):

| Direction | Front-Left | Rear-Left | Front-Right | Rear-Right |
|-----------|-----------|-----------|-------------|------------|
| forward | -1 | -1 | -1 | -1 |
| backward | +1 | +1 | +1 | +1 |
| left | +1 | +1 | -1 | -1 |
| right | -1 | -1 | +1 | +1 |
| strafe_left | +1 | -1 | -1 | +1 |
| strafe_right | -1 | +1 | +1 | -1 |
| rotate_cw | -1 | -1 | +1 | +1 |
| rotate_ccw | +1 | +1 | -1 | -1 |
| diagonal_fl | 0 | -1 | -1 | 0 |
| diagonal_fr | -1 | 0 | 0 | -1 |
| diagonal_bl | +1 | 0 | 0 | +1 |
| diagonal_br | 0 | +1 | +1 | 0 |

---

## Hardware Configuration

| Component | Interface | Address/Pin |
|-----------|-----------|-------------|
| PCA9685 PWM controller | I2C | 0x40 (channels 0-7: motors, 8-15: servos) |
| PCF8591/ADS7830 ADC | I2C | 0x48 |
| WS2812 LEDs (8x) | SPI | GPIO10 (MOSI) |
| HC-SR04 ultrasonic | GPIO | Trigger: GPIO27, Echo: GPIO22 |
| Buzzer | GPIO PWM | GPIO17 (10% duty cycle) |
| IR sensors (3x) | GPIO | Left: GPIO14, Middle: GPIO15, Right: GPIO23 |
| OV5647 camera | CSI | libcamera/rpicam stack |

**Power switches:** S1 = Pi + electronics, S2 = 5V actuator rail (servos, LEDs, ultrasonic)

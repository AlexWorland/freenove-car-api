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
| `pan` | int | 0-180 | Horizontal servo (PCA9685 channel 8) |
| `tilt` | int | 0-180 | Vertical servo (PCA9685 channel 9) |
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
| forward | +1 | +1 | +1 | +1 |
| backward | -1 | -1 | -1 | -1 |
| left | -1 | -1 | +1 | +1 |
| right | +1 | +1 | -1 | -1 |
| strafe_left | -1 | +1 | +1 | -1 |
| strafe_right | +1 | -1 | -1 | +1 |
| rotate_cw | +1 | +1 | -1 | -1 |
| rotate_ccw | -1 | -1 | +1 | +1 |
| diagonal_fl | 0 | +1 | +1 | 0 |
| diagonal_fr | +1 | 0 | 0 | +1 |
| diagonal_bl | -1 | 0 | 0 | -1 |
| diagonal_br | 0 | -1 | -1 | 0 |

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

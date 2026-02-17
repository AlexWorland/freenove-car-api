# Freenove 4WD Smart Car REST API

REST API server and web dashboard for controlling a [Freenove FNK0043 4WD Smart Car Kit](https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi) over HTTP.

## Features

- **Live camera feed** via MJPEG streaming
- **Motor control** with all 12 mecanum drive directions
- **Servo control** with optional speed interpolation
- **WS2812 LED control** with color, brightness, and animations
- **Sensor readout** (ultrasonic, ADC, infrared, camera)
- **Web dashboard** for interactive control from a browser
- **Calibration endpoint** with before/after camera captures
- **Camera sweep** for panoramic image capture
- **Code execution** endpoint for remote scripting
- **System management** (status, reboot, shutdown)

## Hardware

- Raspberry Pi 3A+ (Debian Trixie)
- Freenove FNK0043 carrier board
- PCA9685 PWM controller (motors + servos)
- PCF8591 ADC (photoresistors + battery)
- HC-SR04 ultrasonic sensor
- WS2812 addressable LEDs (x8)
- OV5647 camera (CSI)
- 4x mecanum wheel DC motors
- 2x hobby servos (pan/tilt)

## Quick Start

```bash
# On the Raspberry Pi
pip install flask
cd ~/car-api
python3 server.py
```

Dashboard: `http://<pi-ip>:5000/dashboard`
API index: `http://<pi-ip>:5000/`

## API Documentation

See [API_SPEC.md](API_SPEC.md) for full endpoint documentation with request/response schemas.

## Keyboard Controls (Dashboard)

| Key | Action |
|-----|--------|
| W/S | Forward / Backward |
| A/D | Strafe Left / Right |
| Q/E | Rotate CCW / CW |
| Arrows | Forward / Back / Turn Left / Turn Right |
| Space | Stop |

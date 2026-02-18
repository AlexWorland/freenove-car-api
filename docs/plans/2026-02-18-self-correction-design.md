# Self-Correction Design: Hybrid Multi-Signal Calibration

**Date:** 2026-02-18
**Constraints:** Software-only (no new hardware), maximum precision, existing sensors only
**Dependencies:** OpenCV (optional, graceful degradation without it)

---

## Problem Statement

The Freenove 4WD Smart Car has two calibration issues that compound during autonomous navigation:

1. **Wheel drift** -- Individual motors spin at slightly different rates, causing the car to veer off course during straight-line commands. Dead reckoning accumulates this error.
2. **Servo alignment** -- The pan/tilt servo's reported 90-degree position may not correspond to the camera/ultrasonic sensor's true forward direction.

## Available Sensors

| Sensor | Feedback type | Limitations |
|--------|--------------|-------------|
| Ultrasonic (1x, on pan servo) | Distance to nearest surface | Single point, ~30ms/read, max ~400cm |
| Camera (on same pan/tilt head) | Visual scene (320x240 JPEG) | Requires image processing, CPU-limited |
| IR line sensors (3x, downward) | Binary dark/light | Useless without floor markings |

No wheel encoders, IMU, gyroscope, or compass.

---

## Architecture: Sensor Fusion

Every movement produces three independent estimates of what actually happened. A `SensorFusion` class combines them with confidence-based dynamic weighting.

```
Command: "forward, speed=500, 0.5s"

        +-------------------+
        |  Dead Reckoning   |  "I think I moved 3.5cm at heading 90 deg"
        |  (weight: 0.2)    |  Fast, always available, drifts over time
        +---------+---------+
                  |
        +---------+---------+
        |  Visual Odometry  |  "Features shifted 14px down 3px right, so you moved
        |  (weight: 0.5)    |   3.2cm forward and rotated 1.2 deg clockwise"
        +---------+---------+  High precision, needs texture, relative only
                  |
        +---------+---------+
        |  Ultrasonic Ref   |  "Wall ahead went from 85cm to 81.5cm, wall left
        |  (weight: 0.3)    |   stayed at 60cm -- you went 3.5cm forward, no drift"
        +---------+---------+  Absolute, only works near walls
                  |
        +---------+---------+
        |  Fused Estimate   |  Weighted: dx=3.3cm, dy=0.1cm, d_theta=0.9 deg
        |  -> Update Pose   |  Update grid, correct motor bias table
        +-------------------+
```

Weights are dynamic: if no walls are nearby, ultrasonic weight drops to 0 and the others rebalance. If the scene lacks visual texture, visual odometry weight drops. Dead reckoning is always available as fallback.

---

## Component 1: Visual Odometry

Compares before/after photos to estimate actual car displacement and rotation.

### Algorithm

1. Before movement: capture 320x240 photo facing forward
2. After movement: capture another photo from same servo angle
3. Detect ~50 strong feature points in "before" image (`goodFeaturesToTrack`)
4. Track those features in "after" image (`calcOpticalFlowPyrLK`)
5. Filter outliers (> 2 std deviations from median displacement)
6. Compute dominant translation (dx, dy in pixels) and rotation from remaining vectors
7. Convert pixel displacement to cm using calibrated `px_per_cm_at_1m` factor, scaled by ultrasonic distance

### Calibration Factor

A feature 50cm away moves more pixels per cm of car movement than a feature 3m away. Mitigations:
- Use ultrasonic distance as rough depth estimate for scaling
- Average across many features at varying depths (errors cancel)
- Calibrate `px_per_cm_at_1m` empirically during startup routine

### Performance

- 320x240 sparse optical flow: ~100-200ms on Pi 3A+
- Acceptable within the step cycle budget

### Confidence Scoring

- confidence = 1.0: 30+ features tracked with low reprojection error
- confidence -> 0.0: < 10 features (blank walls, darkness, motion blur)

---

## Component 2: Ultrasonic Reference Correction

A quick 3-point sweep (left 45 deg, center 90 deg, right 135 deg) before and after each movement provides absolute position reference relative to room walls.

### Interpretation

| Pattern | Meaning |
|---------|---------|
| All three drop equally | Pure forward motion, no drift |
| Left drops more than right | Car drifted left (or rotated CCW) |
| Right drops more than left | Car drifted right (or rotated CW) |
| Center unchanged, sides changed | Pure rotation |
| All three unchanged | Car is stuck |

### Distinguishing Drift from Rotation

A 3 deg CCW rotation looks similar to a 2cm left drift with only 3 points. Solution: cross-reference with visual odometry. Ultrasonic tells us "something asymmetric happened," camera tells us whether it was rotation or translation.

### Confidence Scoring

- confidence = 1.0: all 3 readings < 150cm, consistent wall distances
- confidence -> 0.0: all readings > 200cm (open space) or noisy/inconsistent

### Timing

3 servo positions x 150ms settle + 3 readings = ~1.5 seconds per sweep.

---

## Component 3: Servo Alignment Detection

Startup calibration routine to find true pan center.

### Algorithm

1. Do a full scan to find the nearest flat wall
2. Point at that wall
3. Sweep pan from 60 deg to 120 deg in 1 deg increments, reading ultrasonic at each
4. The angle producing minimum distance is perpendicular to the wall = "true straight ahead"
5. Difference from 90 deg = `pan_offset_deg`
6. Apply offset to all future pan commands

Also detects servo mechanical slop: if the minimum is flat across several degrees, the servo has play in that range.

---

## Component 4: Motor Bias Learning

Every movement teaches the car about its hardware imperfections. A per-command correction table adjusts motor PWM values to compensate.

### Bias Table Structure

```python
motor_bias = {
    "forward":      {"lateral": +0.8, "rotation": +3.6, "speed_scale": 0.91},
    "backward":     {"lateral": -0.3, "rotation": -1.2, "speed_scale": 0.88},
    "strafe_left":  {"lateral": +0.1, "rotation": +5.0, "speed_scale": 0.85},
    ...
}
```

- `lateral`: cm/sec of sideways drift
- `rotation`: deg/sec of unintended rotation
- `speed_scale`: actual speed as fraction of expected speed

### How Corrections Apply

Before executing a motor command, per-wheel speeds are adjusted:
- Rotation bias: add differential between left and right sides
- Lateral bias: adjust front/rear differential
- Per-wheel adjustment clamped to +/-15% of commanded speed

### Learning Rate

Exponential moving average with alpha=0.3. Recent observations weighted more than old ones, so the system adapts to battery voltage drop or surface changes.

### Persistence

Saved to `~/freenove-car-api/motor_bias.json`. Loads on server startup. Survives reboots.

### Stale Detection

If battery voltage drops > 0.5V from calibration time, bias table marked stale, learning rate increases to alpha=0.6 for faster adaptation.

---

## Integration into Step Cycle

### New Step Flow

```
Pre-read ultrasonic (3-point) ---+
Pre-capture photo ---------------+
                                 +-- Move with bias-corrected motors
Post-read ultrasonic (3-point) --+
Post-capture photo --------------+
                                 v
                  Sensor Fusion (3 estimates -> 1 corrected pose)
                                 |
                    +------------+-------------+
                    v            v             v
              Update pose   Update bias    Update grid
                            table          (with corrected pose)
```

### Timing Budget

| Phase | Current | With correction |
|-------|---------|-----------------|
| Pre-sensors | ~30ms | ~600ms (3-point sweep + photo) |
| Movement | 500ms | 500ms |
| Post-sensors | ~30ms | ~600ms (3-point sweep + photo) |
| Fusion math | 0ms | ~200ms (optical flow + weighted avg) |
| **Total** | **~560ms** | **~1900ms** |

### Graceful Degradation

- OpenCV not installed: visual odometry returns confidence=0, fusion uses ultrasonic + dead reckoning
- No walls nearby: ultrasonic returns confidence=0, fusion uses visual + dead reckoning
- Both fail: behaves exactly like current system (pure dead reckoning)
- 3-point sweep can be disabled via `/auto/configure` for faster steps

---

## Error Handling

### Confidence-Based Weighting

```python
raw_weights = {
    'visual': visual_confidence * 0.5,
    'ultrasonic': ultrasonic_confidence * 0.3,
    'dead_reckoning': 0.2,
}
total = sum(raw_weights.values())
weights = {k: v / total for k, v in raw_weights.items()}
```

### Bias Table Guardrails

| Parameter | Max | Rationale |
|-----------|-----|-----------|
| Per-wheel speed adjustment | +/-15% | Larger = hardware problem |
| Rotation bias | +/-10 deg/sec | Beyond this = spinning |
| Speed scale factor | 0.5-1.5x | Outside = mechanically wrong |

Exceeding limits: correction clamped, `calibration_warning` flag set in step response.

### OpenCV Absence

```python
try:
    import cv2
    _cv2_available = True
except ImportError:
    _cv2_available = False
```

VisualOdometry returns confidence=0 when unavailable. No errors, no special code paths.

---

## Files

| File | Action | Contents |
|------|--------|---------|
| `calibration.py` | CREATE | `SensorFusion`, `VisualOdometry`, `UltrasonicReference`, `MotorBiasTable`, `ServoAlignment` |
| `autonomy.py` | MODIFY | `execute_step()` calls fusion, applies bias corrections |
| `server.py` | MODIFY | Add `/auto/calibrate` endpoint, load/save bias table |
| `motor_bias.json` | CREATE (runtime) | Persisted bias table, auto-generated |

---

## Startup Calibration Routine (POST /auto/calibrate)

1. Full 180 deg ultrasonic scan to find nearest wall
2. Servo alignment detection against that wall
3. Short forward/backward/strafe sequence with fusion active to seed bias table
4. Duration: ~30 seconds, needed once per session or after battery swap

# Deep Sleep Wakeup Sources

The ESP32-S3 supports multiple wakeup sources from deep sleep. This implementation uses two complementary sources to balance power efficiency with responsiveness.

## Wakeup Sources

### 1. Timer Wakeup
- **Duration**: 300 seconds (5 minutes) - configurable via `DEEP_SLEEP_DURATION_SEC`
- **Purpose**: Periodic sensor readings and MQTT data publishing
- **Wakeup cause**: `ESP_SLEEP_WAKEUP_TIMER`

### 2. Motion Interrupt Wakeup (EXT0)
- **GPIO**: GPIO3 (connected to MPU6050 INT pin)
- **Trigger**: Rising edge (GPIO goes HIGH when motion detected)
- **Purpose**: Immediate wakeup when device is moved/rotated
- **Wakeup cause**: `ESP_SLEEP_WAKEUP_EXT0`

## Hardware Configuration

```
MPU6050 INT pin -----> GPIO3 (ESP32-S3)
                       - Pull-down enabled when awake
                       - RTC domain keeps GPIO monitoring during deep sleep
```

## MPU6050 Motion Detection Configuration

The MPU6050 has built-in motion detection hardware that generates an interrupt when acceleration exceeds a threshold. **This configuration is non-trivial and has several pitfalls.**

### Current Settings
```cpp
motion_sensor.setup_motion_detection(3, 3);  // threshold=3, duration=3ms
```

| Parameter | Value | Meaning |
|-----------|-------|---------|
| Threshold | 3 | 6mg (1 LSB = 2mg at 2G range) |
| Duration | 3 | 3ms acceleration must exceed threshold |

### Critical Configuration Requirements

#### 1. High-Pass Filter (HPF) is MANDATORY

**Problem**: Without HPF, motion detection compares raw accelerometer output against the threshold. Since gravity contributes ~1g (1000mg) to the reading, a stationary device would always exceed any reasonable threshold.

**Solution**: Enable the accelerometer HPF which filters out the DC component (gravity):
```cpp
// ACCEL_CONFIG register 0x1C
// bits 4:3 = AFS_SEL (range), bits 2:0 = ACCEL_HPF
// HPF=001 (5Hz cutoff) filters gravity while passing motion
i2c.transmit(dev_handle, MPU6050_REG_ACCEL_CONFIG, 0x01);
```

HPF Options:
| Value | Cutoff Frequency | Use Case |
|-------|------------------|----------|
| 000 | Reset | Not useful |
| 001 | 5Hz | Good for motion detection |
| 010 | 2.5Hz | More sensitive |
| 011 | 1.25Hz | Very sensitive |
| 100 | 0.63Hz | Ultra sensitive |
| 111 | Hold | Sample and hold |

#### 2. Interrupt Latching Behavior

**Problem**: The motion interrupt can be configured as latched or pulse. A pulse might be missed if the ESP32 is in deep sleep when it occurs.

**Solution**: Configure latched interrupt that stays HIGH until explicitly cleared:
```cpp
// INT_PIN_CFG register 0x37
// Bit 5: LATCH_INT_EN = 1 (interrupt latched until INT_STATUS read)
i2c.transmit(dev_handle, MPU6050_REG_INT_PIN_CFG, 0x20);
```

**Important**: Reading INT_STATUS (0x3A) clears the latch. If motion continues, the interrupt will immediately re-trigger.

#### 3. Threshold Sensitivity Issues

**Problem**: Very low thresholds (like 3 = 6mg) can trigger on:
- Environmental vibrations
- Temperature-induced sensor drift
- Electrical noise

**Trade-off**:
- Lower threshold = more responsive but more false positives
- Higher threshold = fewer false wakeups but might miss gentle motion

Recommended ranges:
| Use Case | Threshold | Sensitivity |
|----------|-----------|-------------|
| Tap detection | 20-50 | 40-100mg |
| Gentle motion | 3-10 | 6-20mg |
| Strong motion | 50-100 | 100-200mg |

#### 4. Duration Parameter

The duration specifies how long (in ms) acceleration must exceed threshold before triggering.

**Trade-off**:
- Short duration (1-5ms) = quick response, more noise susceptible
- Long duration (10-50ms) = filters noise, slower response

### Interrupt Pin Configuration Summary

```cpp
// Register 0x37 (INT_PIN_CFG)
// Bit 7: INT_LEVEL    = 0 (active HIGH)
// Bit 6: INT_OPEN     = 0 (push-pull, not open-drain)
// Bit 5: LATCH_INT_EN = 1 (latched until cleared)
// Bit 4: INT_RD_CLEAR = 0 (cleared by reading INT_STATUS only)
// Value: 0x20
```

### Enabling Motion Interrupt

```cpp
// Register 0x38 (INT_ENABLE)
// Bit 6: MOT_EN = 1 (motion detection interrupt enable)
// Value: 0x40
```

## Implementation Files

| File | Purpose |
|------|---------|
| `main/deep_sleep.hpp` | Constants and function declarations |
| `main/deep_sleep.cpp` | Sleep preparation and entry logic |
| `main/mpu6050.cpp` | Motion detection configuration |
| `main/heatsens.cpp` | Wakeup cause logging |

## Sleep/Wake Cycle

```
                    +------------------+
                    |   Device Awake   |
                    |  (publish data)  |
                    +--------+---------+
                             |
                             v
                    +------------------+
                    | deep_sleep_prepare() |
                    | - Stop MQTT      |
                    | - Disconnect WiFi|
                    | - Clear pending  |
                    |   motion interrupt|
                    +--------+---------+
                             |
                             v
                    +------------------+
                    | deep_sleep_enter()  |
                    | - Enable timer   |
                    |   wakeup (300s)  |
                    | - Enable EXT0    |
                    |   wakeup (GPIO3) |
                    +--------+---------+
                             |
                             v
                    +------------------+
                    |   Deep Sleep     |
                    | (ultra-low power)|
                    +--------+---------+
                             |
              +--------------+--------------+
              |                             |
              v                             v
     +----------------+           +------------------+
     | Timer expired  |           | Motion detected  |
     | (300 seconds)  |           | (GPIO3 HIGH)     |
     +-------+--------+           +--------+---------+
              |                             |
              +-------------+---------------+
                            |
                            v
                    +------------------+
                    |   Device Reset   |
                    |   (app_main)     |
                    +------------------+
```

## Key Code Sections

### Configuring EXT0 Wakeup (deep_sleep.cpp)
```cpp
// Wake up when GPIO goes HIGH (MPU6050 INT pin triggers on motion)
esp_sleep_enable_ext0_wakeup(MOTION_WAKEUP_GPIO, 1);
```

### Clearing Pending Interrupts Before Sleep (deep_sleep.cpp)

**Critical**: Must clear any pending motion interrupt before entering deep sleep, otherwise the device will wake immediately.

```cpp
auto &motion_sensor = Mpu6050::getInstance();
if (motion_sensor.is_initialized) {
    bool motion_pending;
    motion_sensor.check_motion_interrupt(&motion_pending);  // Clears by reading INT_STATUS
    if (motion_pending) {
        ESP_LOGI(TAG, "Cleared pending motion interrupt before sleep");
    }
}
```

### Detecting Wakeup Cause (heatsens.cpp)
```cpp
esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
switch (wakeup_cause) {
    case ESP_SLEEP_WAKEUP_TIMER:
        // Periodic wakeup - do normal sensor read cycle
        break;
    case ESP_SLEEP_WAKEUP_EXT0:
        // Motion detected - device was moved
        break;
}
```

## Troubleshooting

### Device wakes immediately after entering sleep
- Motion interrupt may still be asserted
- Solution: Call `check_motion_interrupt()` to clear INT_STATUS before sleep

### Motion not detected / no wakeup on movement
- HPF may not be enabled (gravity saturating the comparison)
- Threshold too high for the motion intensity
- Check INT_ENABLE register has bit 6 set

### Too many false wakeups
- Threshold too low
- Environmental vibrations
- Try increasing threshold or duration

### Interrupt stuck HIGH
- INT_STATUS not being read to clear the latch
- Motion may be continuous (re-triggering immediately after clear)

## Power Considerations

- During deep sleep, only the RTC domain remains powered
- GPIO3 is monitored by the RTC GPIO controller
- MPU6050 remains powered and continues motion detection independently
- Typical deep sleep current: ~10uA (ESP32-S3) + MPU6050 standby current (~10uA)

## Configuration Constants

| Constant | Value | Location |
|----------|-------|----------|
| `DEEP_SLEEP_DURATION_SEC` | 300 | deep_sleep.hpp |
| `MOTION_WAKEUP_GPIO` | GPIO_NUM_3 | deep_sleep.hpp |
| Motion threshold | 3 (6mg) | heatsens.cpp |
| Motion duration | 3ms | heatsens.cpp |

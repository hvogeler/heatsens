# Deep Sleep Implementation

This document describes the deep sleep implementation for the heat sensor, including both timer-based and motion-based wake-up sources.

## Overview

The ESP32-S3 enters deep sleep between measurement cycles to conserve power. Two wake-up sources are configured:

1. **Timer wake-up** - Periodic wake-up at fixed intervals (backup/primary)
2. **Motion wake-up (ext0)** - Immediate wake-up when the device is moved

## Hardware Configuration

| Component | GPIO | Description |
|-----------|------|-------------|
| I2C SCL | GPIO 44 | I2C clock for MPU-6050 and BMP280 |
| I2C SDA | GPIO 43 | I2C data for MPU-6050 and BMP280 |
| MPU-6050 INT | GPIO 10 | Motion interrupt (ext0 wake-up source) |

**Note:** GPIO 10 is an RTC GPIO (0-21 range required for ext0 wake-up on ESP32-S3).

## Source Files

| File | Purpose |
|------|---------|
| `main/deep_sleep.hpp` | Constants and function declarations |
| `main/deep_sleep.cpp` | Sleep preparation and entry logic |
| `main/mpu6050.cpp` | Motion detection configuration |
| `main/heatsens.cpp` | Wake-up cause handling and sleep cycle |

## Configuration Constants

Defined in `main/deep_sleep.hpp`:

```cpp
constexpr uint64_t DEEP_SLEEP_DURATION_SEC = 10;      // Timer interval (seconds)
constexpr gpio_num_t MPU6050_INT_GPIO = GPIO_NUM_10;  // Motion interrupt GPIO
```

## Wake-up Sources

### 1. Timer Wake-up

The RTC timer wakes the device at regular intervals.

**Configuration:**
```cpp
uint64_t sleep_time_us = DEEP_SLEEP_DURATION_SEC * 1000000ULL;
esp_sleep_enable_timer_wakeup(sleep_time_us);
```

**Current setting:** 10 seconds (for testing)
**Production setting:** ~10 minutes (adjust `DEEP_SLEEP_DURATION_SEC`)

**Wake-up cause:** `ESP_SLEEP_WAKEUP_TIMER`

### 2. Motion Wake-up (ext0)

The MPU-6050 motion sensor triggers an interrupt when movement is detected. This interrupt is connected to GPIO 10 and configured as an ext0 wake-up source.

**ESP32 Configuration:**
```cpp
esp_sleep_enable_ext0_wakeup(MPU6050_INT_GPIO, 1);  // Wake on HIGH
```

**Wake-up cause:** `ESP_SLEEP_WAKEUP_EXT0`

## MPU-6050 Motion Detection

The MPU-6050 hardware motion detection is configured in `Mpu6050::setup_motion_detection()`.

### Interrupt Pin Configuration

Register `INT_PIN_CFG` (0x37) = `0x10`:
- Active HIGH output
- Push-pull driver
- Interrupt held until `INT_STATUS` is read (latch mode)
- Cleared on read of `INT_STATUS`

### Motion Threshold

Register `MOT_THR` (0x1F):
- 1 LSB = 2mg (at 2g accelerometer range)
- Current setting: 50 (= 100mg threshold)

### Motion Duration

Register `MOT_DUR` (0x20):
- 1 LSB = 1ms
- Current setting: 50ms (debounce filter)

### Interrupt Enable

Register `INT_ENABLE` (0x38) = `0x40`:
- Bit 6: Motion detection interrupt enabled

### Current Configuration Call

In `heatsens.cpp`:
```cpp
motion_sensor.setup_motion_detection(50, 50);
// threshold=50 -> 100mg
// duration=50  -> 50ms debounce
```

## Sleep Cycle Flow

### 1. Preparation (`deep_sleep_prepare`)

Before entering deep sleep, network connections must be cleanly shut down:

```
1. Stop MQTT client (depends on WiFi)
   └── Acquire mutex, call mqtt.stop()

2. Wait 100ms for MQTT disconnect

3. Disconnect WiFi
   └── Acquire mutex, call wifi.wifi_disconnect()
```

### 2. Enter Sleep (`deep_sleep_enter`)

```
1. Disable all previous wake-up sources
   └── esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL)

2. Configure timer wake-up
   └── esp_sleep_enable_timer_wakeup(sleep_time_us)

3. Configure ext0 wake-up (motion)
   └── esp_sleep_enable_ext0_wakeup(GPIO_NUM_10, 1)

4. Flush logs (100ms delay)

5. Enter deep sleep
   └── esp_deep_sleep_start()  // Does not return
```

### 3. Wake-up Handling

On wake-up, the device performs a full reset. The wake-up cause is detected in `app_main()`:

```cpp
esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
switch (wakeup_cause)
{
case ESP_SLEEP_WAKEUP_TIMER:
    // Periodic timer expired
    break;
case ESP_SLEEP_WAKEUP_EXT0:
    // Motion detected
    break;
case ESP_SLEEP_WAKEUP_UNDEFINED:
    // Fresh boot (power-on reset)
    break;
}
```

### 4. Awake Cycle

After wake-up, the device:

1. Initializes all peripherals (WiFi, MQTT, sensors)
2. Runs `AWAKE_LOOP_ITERATIONS` (5) measurement cycles
3. Each cycle: read sensor, publish to MQTT, wait 1 second
4. Calls `deep_sleep_prepare()` then `deep_sleep_enter()`

## Hardware Wiring

Connect the MPU-6050 INT pin to ESP32-S3 GPIO 10:

```
MPU-6050          ESP32-S3 (LilyGo T-Display-S3)
────────          ─────────────────────────────
VCC        ───>   3.3V
GND        ───>   GND
SDA        ───>   GPIO 43
SCL        ───>   GPIO 44
INT        ───>   GPIO 10  (motion wake-up)
```

## Power Considerations

During deep sleep:
- Main CPU is powered off
- RTC memory and RTC peripherals remain powered
- WiFi/BT radio is off
- GPIO state is reset (except RTC GPIOs)

The MPU-6050 remains powered and monitors for motion independently, asserting INT when motion threshold is exceeded.

## Adjusting Sensitivity

### Less Sensitive (fewer false wake-ups)
```cpp
motion_sensor.setup_motion_detection(100, 100);  // 200mg, 100ms
```

### More Sensitive (wake on slight movement)
```cpp
motion_sensor.setup_motion_detection(25, 25);    // 50mg, 25ms
```

## Troubleshooting

### Device doesn't wake on motion
1. Verify GPIO 10 is connected to MPU-6050 INT pin
2. Check that `setup_motion_detection()` is called before sleep
3. Ensure INT pin is not read (`check_motion_interrupt()`) right before sleep (clears latch)

### Device wakes immediately
1. Motion threshold too low - increase threshold parameter
2. INT pin floating - ensure proper connection
3. Check if INT is already HIGH before sleep

### Timer wake-up not working
1. Verify `DEEP_SLEEP_DURATION_SEC` is set correctly
2. Check for errors in `esp_sleep_enable_timer_wakeup()` return value

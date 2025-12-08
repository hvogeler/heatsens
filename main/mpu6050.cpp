#include "mpu6050.hpp"
#include "i2c.hpp"
#include <math.h>

void Mpu6050::set_accel_scale(mpu6050_accel_range range) {
    switch (range) {
        case MPU6050_ACCEL_RANGE_2G:
            accel_scale = MPU6050_ACCEL_SCALE_2G;
            break;
        case MPU6050_ACCEL_RANGE_4G:
            accel_scale = MPU6050_ACCEL_SCALE_4G;
            break;
        case MPU6050_ACCEL_RANGE_8G:
            accel_scale = MPU6050_ACCEL_SCALE_8G;
            break;
        case MPU6050_ACCEL_RANGE_16G:
            accel_scale = MPU6050_ACCEL_SCALE_16G;
            break;
    }
}

void Mpu6050::set_gyro_scale(mpu6050_gyro_range range) {
    switch (range) {
        case MPU6050_GYRO_RANGE_250:
            gyro_scale = MPU6050_GYRO_SCALE_250;
            break;
        case MPU6050_GYRO_RANGE_500:
            gyro_scale = MPU6050_GYRO_SCALE_500;
            break;
        case MPU6050_GYRO_RANGE_1000:
            gyro_scale = MPU6050_GYRO_SCALE_1000;
            break;
        case MPU6050_GYRO_RANGE_2000:
            gyro_scale = MPU6050_GYRO_SCALE_2000;
            break;
    }
}

esp_err_t Mpu6050::init(mpu6050_accel_range accel_range, mpu6050_gyro_range gyro_range) {
    esp_err_t err;
    uint8_t who_am_i;
    bool found = false;

    auto& i2c = I2c::getInstance();
    std::lock_guard<std::mutex> i2c_lock(i2c.getMutex());

    // Try address 0x68 first (AD0 = 0)
    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = MPU6050_I2C_ADDRESS_0;
    dev_config.scl_speed_hz = I2C_MASTER_FREQ_HZ;

    err = i2c.add_device(dev_config, dev_handle);
    if (err == ESP_OK) {
        err = i2c.receive(dev_handle, MPU6050_REG_WHO_AM_I, &who_am_i, 1);
        if (err == ESP_OK && who_am_i == MPU6050_WHO_AM_I_VALUE) {
            i2c_dev_addr = MPU6050_I2C_ADDRESS_0;
            found = true;
            ESP_LOGI(TAG, "Found MPU6050 at address 0x%02x (WHO_AM_I: 0x%02x)",
                     i2c_dev_addr, who_am_i);
        } else {
            i2c.rm_device(dev_handle);
        }
    }

    // Try address 0x69 if not found at 0x68
    if (!found) {
        ESP_LOGW(TAG, "MPU6050 not found at 0x68, trying 0x69...");
        dev_config.device_address = MPU6050_I2C_ADDRESS_1;

        err = i2c.add_device(dev_config, dev_handle);
        if (err == ESP_OK) {
            err = i2c.receive(dev_handle, MPU6050_REG_WHO_AM_I, &who_am_i, 1);
            if (err == ESP_OK && who_am_i == MPU6050_WHO_AM_I_VALUE) {
                i2c_dev_addr = MPU6050_I2C_ADDRESS_1;
                found = true;
                ESP_LOGI(TAG, "Found MPU6050 at address 0x%02x (WHO_AM_I: 0x%02x)",
                         i2c_dev_addr, who_am_i);
            }
        }
    }

    if (!found) {
        ESP_LOGE(TAG, "MPU6050 not found at either 0x68 or 0x69");
        logger.error(TAG, "MPU6050 not found at either 0x68 or 0x69");
        return ESP_FAIL;
    }

    // Reset the device
    err = i2c.transmit(dev_handle, MPU6050_REG_PWR_MGMT_1, 0x80);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset MPU6050");
        logger.error(TAG, "Failed to reset MPU6050");
        return err;
    }

    // Wait for reset to complete
    vTaskDelay(pdMS_TO_TICKS(100));

    // Wake up the device (exit sleep mode, use internal 8MHz oscillator)
    err = i2c.transmit(dev_handle, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
        logger.error(TAG, "Failed to wake up MPU6050");
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Configure sample rate divider (1kHz / (1 + SMPLRT_DIV))
    // Setting to 0 gives 1kHz sample rate
    err = i2c.transmit(dev_handle, MPU6050_REG_SMPLRT_DIV, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set sample rate");
        return err;
    }

    // Configure DLPF (Digital Low Pass Filter)
    // CONFIG = 0x00: Accel BW=260Hz, Gyro BW=256Hz
    err = i2c.transmit(dev_handle, MPU6050_REG_CONFIG, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure DLPF");
        return err;
    }

    // Configure gyroscope range
    err = i2c.transmit(dev_handle, MPU6050_REG_GYRO_CONFIG, gyro_range);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope");
        return err;
    }
    set_gyro_scale(gyro_range);

    // Configure accelerometer range
    err = i2c.transmit(dev_handle, MPU6050_REG_ACCEL_CONFIG, accel_range);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer");
        return err;
    }
    set_accel_scale(accel_range);

    // Configure power management 2 - enable all accelerometer axes
    // Bits 5-3: Disable standby for accel X, Y, Z (0 = enabled)
    // Bits 2-0: Disable standby for gyro X, Y, Z (0 = enabled)
    err = i2c.transmit(dev_handle, MPU6050_REG_PWR_MGMT_2, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWR_MGMT_2");
        return err;
    }

    is_initialized = true;
    ESP_LOGI(TAG, "MPU6050 initialization complete");
    logger.info(TAG, "MPU6050 initialization complete");
    return ESP_OK;
}

esp_err_t Mpu6050::read_raw(mpu6050_raw_data* data) {
    if (!is_initialized) {
        ESP_LOGE(TAG, "MPU6050 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    auto& i2c = I2c::getInstance();
    std::lock_guard<std::mutex> lock_i2c(i2c.getMutex());

    // Read all 14 bytes (accel, temp, gyro) in one transaction
    uint8_t raw_data[14];
    esp_err_t err = i2c.receive(dev_handle, MPU6050_REG_ACCEL_XOUT_H, raw_data, 14);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data");
        logger.error(TAG, "Failed to read sensor data");
        return err;
    }

    // Combine high and low bytes (big-endian format)
    data->accel_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    data->accel_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    data->accel_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
    data->temp = (int16_t)((raw_data[6] << 8) | raw_data[7]);
    data->gyro_x = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    data->gyro_y = (int16_t)((raw_data[10] << 8) | raw_data[11]);
    data->gyro_z = (int16_t)((raw_data[12] << 8) | raw_data[13]);

    return ESP_OK;
}

esp_err_t Mpu6050::read_scaled(mpu6050_data* data) {
    mpu6050_raw_data raw;
    esp_err_t err = read_raw(&raw);
    if (err != ESP_OK) {
        return err;
    }

    // Convert to physical units
    data->accel_x = (float)raw.accel_x / accel_scale;
    data->accel_y = (float)raw.accel_y / accel_scale;
    data->accel_z = (float)raw.accel_z / accel_scale;

    // Temperature in degrees C = (TEMP_OUT Register Value / 340) + 36.53
    data->temp = ((float)raw.temp / 340.0f) + 36.53f;

    data->gyro_x = (float)raw.gyro_x / gyro_scale;
    data->gyro_y = (float)raw.gyro_y / gyro_scale;
    data->gyro_z = (float)raw.gyro_z / gyro_scale;

    return ESP_OK;
}

esp_err_t Mpu6050::setup_motion_detection(uint8_t threshold, uint8_t duration) {
    if (!is_initialized) {
        ESP_LOGE(TAG, "MPU6050 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    auto& i2c = I2c::getInstance();
    std::lock_guard<std::mutex> lock_i2c(i2c.getMutex());

    esp_err_t err;

    // Configure interrupt pin
    // INT_PIN_CFG: Active high, push-pull, held until cleared, cleared on read of INT_STATUS
    err = i2c.transmit(dev_handle, MPU6050_REG_INT_PIN_CFG, 0x10);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure interrupt pin");
        return err;
    }

    // Set motion detection threshold (1 LSB = 2mg for 2g range)
    // For example, threshold=10 means 20mg
    err = i2c.transmit(dev_handle, MPU6050_REG_MOT_THR, threshold);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set motion threshold");
        return err;
    }

    // Set motion detection duration (1 LSB = 1ms)
    err = i2c.transmit(dev_handle, MPU6050_REG_MOT_DUR, duration);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set motion duration");
        return err;
    }

    // Configure motion detection control
    // Bits 7-6: Accel on delay (00 = 1 sample)
    // Bits 5-4: FF_COUNT (00 = 1 sample)
    // Bits 3-2: MOT_COUNT (00 = 1 sample)
    // Setting all to increment by 1 for faster detection
    err = i2c.transmit(dev_handle, MPU6050_REG_MOT_DETECT_CTRL, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure motion detection control");
        return err;
    }

    // Enable motion detection interrupt
    // Bit 6: MOT_EN (Motion detection interrupt enable)
    err = i2c.transmit(dev_handle, MPU6050_REG_INT_ENABLE, 0x40);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable motion interrupt");
        return err;
    }

    // Read back and verify configuration
    uint8_t verify_thr, verify_dur, verify_int_en;
    i2c.receive(dev_handle, MPU6050_REG_MOT_THR, &verify_thr, 1);
    i2c.receive(dev_handle, MPU6050_REG_MOT_DUR, &verify_dur, 1);
    i2c.receive(dev_handle, MPU6050_REG_INT_ENABLE, &verify_int_en, 1);

    ESP_LOGI(TAG, "Motion detection configured: threshold=%d (%.1fmg), duration=%dms",
             threshold, threshold * 2.0f, duration);
    ESP_LOGI(TAG, "Verified - THR: 0x%02x, DUR: 0x%02x, INT_EN: 0x%02x",
             verify_thr, verify_dur, verify_int_en);
    logger.info(TAG, "Motion detection configured: threshold=%d, duration=%dms",
                threshold, duration);

    return ESP_OK;
}

esp_err_t Mpu6050::check_motion_interrupt(bool* motion_detected) {
    if (!is_initialized) {
        ESP_LOGE(TAG, "MPU6050 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    auto& i2c = I2c::getInstance();
    std::lock_guard<std::mutex> lock_i2c(i2c.getMutex());

    uint8_t int_status;
    esp_err_t err = i2c.receive(dev_handle, MPU6050_REG_INT_STATUS, &int_status, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read interrupt status");
        return err;
    }

    // Bit 6 is the motion detection interrupt flag
    *motion_detected = (int_status & 0x40) != 0;

    // Log interrupt status for debugging
    ESP_LOGD(TAG, "INT_STATUS: 0x%02x, Motion bit: %d", int_status, *motion_detected);

    if (*motion_detected) {
        last_motion_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        ESP_LOGI(TAG, "Motion interrupt detected!");
    }

    return ESP_OK;
}

float Mpu6050::get_accel_magnitude(const mpu6050_data* data) {
    // Calculate total acceleration vector magnitude
    float mag = sqrtf(data->accel_x * data->accel_x +
                      data->accel_y * data->accel_y +
                      data->accel_z * data->accel_z);

    // Subtract 1g (gravity) to get motion component
    // When stationary, magnitude should be ~1g due to gravity
    return fabsf(mag - 1.0f);
}

bool Mpu6050::is_moving(const mpu6050_data* data, float threshold_g) {
    std::lock_guard<std::mutex> lock(mutex_);

    float current_magnitude = get_accel_magnitude(data);

    // Calculate difference from previous reading
    float delta = fabsf(current_magnitude - prev_accel_magnitude);

    // Update previous magnitude for next comparison
    prev_accel_magnitude = current_magnitude;

    // If delta exceeds threshold or current magnitude is high, we're moving
    bool moving = (delta > threshold_g) || (current_magnitude > threshold_g);

    if (moving) {
        last_motion_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }

    return moving;
}

bool Mpu6050::is_rotated(const mpu6050_data* data, float threshold_g) {
    std::lock_guard<std::mutex> lock(mutex_);

    // First call - just store current values
    if (!has_prev_accel) {
        prev_accel_x = data->accel_x;
        prev_accel_y = data->accel_y;
        prev_accel_z = data->accel_z;
        has_prev_accel = true;
        return false;
    }

    // Calculate change in each axis
    float delta_x = fabsf(data->accel_x - prev_accel_x);
    float delta_y = fabsf(data->accel_y - prev_accel_y);
    float delta_z = fabsf(data->accel_z - prev_accel_z);

    // Find maximum change across all axes
    float max_delta = delta_x;
    if (delta_y > max_delta) max_delta = delta_y;
    if (delta_z > max_delta) max_delta = delta_z;

    // Update previous values for next comparison
    prev_accel_x = data->accel_x;
    prev_accel_y = data->accel_y;
    prev_accel_z = data->accel_z;

    // Rotation detected if any axis changed significantly
    bool rotated = (max_delta > threshold_g);

    if (rotated) {
        last_motion_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        ESP_LOGD(TAG, "Rotation detected: ΔX=%.3f, ΔY=%.3f, ΔZ=%.3f g",
                 delta_x, delta_y, delta_z);
    }

    return rotated;
}

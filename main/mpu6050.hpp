#include <mutex>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mqtt_logger.hpp"

// MPU6050 Register addresses
#define MPU6050_REG_WHO_AM_I 0x75
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_PWR_MGMT_2 0x6C
#define MPU6050_REG_SMPLRT_DIV 0x19
#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_INT_PIN_CFG 0x37
#define MPU6050_REG_INT_ENABLE 0x38
#define MPU6050_REG_INT_STATUS 0x3A
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_TEMP_OUT_H 0x41
#define MPU6050_REG_GYRO_XOUT_H 0x43
#define MPU6050_REG_SIGNAL_PATH_RESET 0x68
#define MPU6050_REG_USER_CTRL 0x6A
#define MPU6050_REG_MOT_THR 0x1F
#define MPU6050_REG_MOT_DUR 0x20
#define MPU6050_REG_MOT_DETECT_CTRL 0x69

// MPU6050 WHO_AM_I value
#define MPU6050_WHO_AM_I_VALUE 0x68

// MPU6050 I2C addresses (AD0 pin determines address)
#define MPU6050_I2C_ADDRESS_0 0x68  // AD0 = 0
#define MPU6050_I2C_ADDRESS_1 0x69  // AD0 = 1

// Accelerometer sensitivity scale factors (LSB/g)
#define MPU6050_ACCEL_SCALE_2G 16384.0f
#define MPU6050_ACCEL_SCALE_4G 8192.0f
#define MPU6050_ACCEL_SCALE_8G 4096.0f
#define MPU6050_ACCEL_SCALE_16G 2048.0f

// Gyroscope sensitivity scale factors (LSB/(deg/s))
#define MPU6050_GYRO_SCALE_250 131.0f
#define MPU6050_GYRO_SCALE_500 65.5f
#define MPU6050_GYRO_SCALE_1000 32.8f
#define MPU6050_GYRO_SCALE_2000 16.4f

// Accelerometer range options
enum mpu6050_accel_range {
    MPU6050_ACCEL_RANGE_2G = 0x00,
    MPU6050_ACCEL_RANGE_4G = 0x08,
    MPU6050_ACCEL_RANGE_8G = 0x10,
    MPU6050_ACCEL_RANGE_16G = 0x18
};

// Gyroscope range options
enum mpu6050_gyro_range {
    MPU6050_GYRO_RANGE_250 = 0x00,
    MPU6050_GYRO_RANGE_500 = 0x08,
    MPU6050_GYRO_RANGE_1000 = 0x10,
    MPU6050_GYRO_RANGE_2000 = 0x18
};

// Raw sensor data structure
struct mpu6050_raw_data {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};

// Scaled sensor data structure
struct mpu6050_data {
    float accel_x;  // g
    float accel_y;  // g
    float accel_z;  // g
    float temp;     // degrees Celsius
    float gyro_x;   // deg/s
    float gyro_y;   // deg/s
    float gyro_z;   // deg/s
};

class Mpu6050 {
    i2c_master_dev_handle_t dev_handle;
    uint8_t i2c_dev_addr;

    float accel_scale;
    float gyro_scale;

    // For tracking movement state
    float prev_accel_magnitude;
    uint32_t last_motion_time_ms;

    // For tracking rotation (gravity vector direction)
    float prev_accel_x;
    float prev_accel_y;
    float prev_accel_z;
    bool has_prev_accel;

    mutable std::mutex mutex_;
    MqttLogger logger;

    static constexpr char* TAG = "Mpu6050";

public:
    bool is_initialized = false;

public:
    // Delete copy constructor and assignment operator
    Mpu6050(const Mpu6050&) = delete;
    Mpu6050& operator=(const Mpu6050&) = delete;

    // Static method to get the singleton instance
    static Mpu6050& getInstance() {
        static Mpu6050 instance;
        return instance;
    }

    std::mutex& getMutex() { return mutex_; }

    esp_err_t init(mpu6050_accel_range accel_range = MPU6050_ACCEL_RANGE_2G,
                   mpu6050_gyro_range gyro_range = MPU6050_GYRO_RANGE_250);
    esp_err_t read_raw(mpu6050_raw_data* data);
    esp_err_t read_scaled(mpu6050_data* data);

    // Motion detection functions
    // NOTE: Motion interrupt flag latches (stays set) until read, so you won't miss motion
    // However, reading INT_STATUS clears the flag, and it can immediately re-trigger if motion continues
    esp_err_t setup_motion_detection(uint8_t threshold, uint8_t duration);

    // Check and clear motion interrupt flag
    // Returns true if motion was detected since last check
    // WARNING: This clears the interrupt flag! If motion is ongoing, it will re-trigger immediately
    esp_err_t check_motion_interrupt(bool* motion_detected);

    // Calculate total acceleration magnitude (excluding gravity)
    // Useful for detecting movement - compare against a threshold
    float get_accel_magnitude(const mpu6050_data* data);

    // Calculate if device is stationary (low acceleration variance over time)
    // Call this periodically with current data to track movement state
    bool is_moving(const mpu6050_data* data, float threshold_g = 0.1f);

    // Detect rotation by tracking gravity vector direction changes
    // More sensitive to gentle rotation than is_moving()
    // threshold_g: minimum change in any axis to detect rotation (e.g., 0.1g ≈ 6° rotation)
    bool is_rotated(const mpu6050_data* data, float threshold_g = 0.1f);

private:
    Mpu6050() : dev_handle(nullptr), accel_scale(MPU6050_ACCEL_SCALE_2G),
                gyro_scale(MPU6050_GYRO_SCALE_250), prev_accel_magnitude(0.0f),
                last_motion_time_ms(0), prev_accel_x(0.0f), prev_accel_y(0.0f),
                prev_accel_z(0.0f), has_prev_accel(false) {
        // Don't initialize MPU6050 here - I2C must be set up first
    }

    void set_accel_scale(mpu6050_accel_range range);
    void set_gyro_scale(mpu6050_gyro_range range);
};

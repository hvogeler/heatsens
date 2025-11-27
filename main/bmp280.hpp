#include <mutex>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mqtt_logger.hpp"

#define I2C_SDA_PIN GPIO_NUM_43
#define I2C_SCL_PIN GPIO_NUM_44
#define I2C_MASTER_FREQ_HZ 100000 // 100kHz - reduced for stability

#define BMP280_REG_ID 0xD0
#define BMP280_REG_RESET 0xE0
#define BMP280_REG_STATUS 0xF3
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_TEMP_MSB 0xFA

// BMP280 calibration data registers
#define BMP280_REG_CALIB_START 0x88

// BMP280 chip IDs
#define BMP280_CHIP_ID 0x58
#define BME280_CHIP_ID 0x60

// BMP280 I2C addresses
#define BMP280_I2C_ADDRESS_0 0x76
#define BMP280_I2C_ADDRESS_1 0x77

// Calibration data structure
struct bmp280_calib_data
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
};

class Bmp280
{
    i2c_master_bus_handle_t i2c_bus_handle;
    i2c_master_dev_handle_t dev_handle;
    bmp280_calib_data calib_data;
    uint8_t i2c_dev_addr;

    mutable std::mutex mutex_;
    MqttLogger logger;

public:
    bool is_initialized = false;

public:
    // Delete copy constructor and assignment operator
    Bmp280(const Bmp280 &) = delete;
    Bmp280 &operator=(const Bmp280 &) = delete;

    // Static method to get the singleton instance
    static Bmp280 &getInstance()
    {
        static Bmp280 instance;
        return instance;
    }
    std::mutex &getMutex() { return mutex_; }

    void init();
    esp_err_t read_raw(int32_t *raw_temp, int32_t *raw_press);
    void compensate_temp_press(int32_t raw_temp, int32_t raw_press,
                               double *temperature, double *pressure);

private:
    Bmp280() : i2c_bus_handle(nullptr),
               dev_handle(nullptr), i2c_dev_addr(0), logger(MqttLogger())
    {
        // Don't initialize BMP280 here - I2C must be set up first
    }

    // Helper methods for BMP280 communication
    esp_err_t write_reg(uint8_t reg, uint8_t value);
    esp_err_t read_reg(uint8_t reg, uint8_t *data, size_t len);
    esp_err_t read_calibration();
};

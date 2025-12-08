#include "driver/i2c_master.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
#include "esp_log.h"
#include <string>
#include <mutex>

#define I2C_MASTER_FREQ_HZ 100000 // 100kHz - reduced for stability

static constexpr char *TAG = "I2c";

class I2c
{
    static constexpr gpio_num_t I2C_SDA_PIN = GPIO_NUM_43;
    static constexpr gpio_num_t I2C_SCL_PIN = GPIO_NUM_44;
    i2c_master_bus_handle_t bus_handle;
    mutable std::mutex mutex_;

private:
    // Delete copy constructor and assignment operator
    I2c(const I2c &) = delete;
    I2c &operator=(const I2c &) = delete;

public:
    // Static method to get the singleton instance
    static I2c &getInstance()
    {
        static I2c instance;
        return instance;
    }
    std::mutex &getMutex() { return mutex_; }

    I2c() : bus_handle(nullptr) {}
    void init();
    esp_err_t add_device(i2c_device_config_t, i2c_master_dev_handle_t &);
    void del_bus();
    esp_err_t rm_device(i2c_master_dev_handle_t &);
    esp_err_t transmit(i2c_master_dev_handle_t &dev_handle, uint8_t reg, uint8_t value);
    esp_err_t receive(i2c_master_dev_handle_t &dev_handle, uint8_t reg, uint8_t *data, size_t len);
};
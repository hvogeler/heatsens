#include "i2c.hpp"
#include "esp_log.h"

void I2c::init()
{
    esp_err_t err;

    // Configure I2C master bus
    i2c_master_bus_config_t bus_config = {};
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.scl_io_num = I2c::I2C_SCL_PIN;
    bus_config.sda_io_num = I2c::I2C_SDA_PIN;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    err = i2c_new_master_bus(&bus_config, &bus_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "I2C master bus created successfully");
}

esp_err_t I2c::add_device(i2c_device_config_t dev_config, i2c_master_dev_handle_t &dev_handle)
{
    if (!bus_handle)
    {
        ESP_LOGE(TAG, "I2C Bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);
    return err;
}

void I2c::del_bus()
{
    ESP_LOGE(TAG, "I2C bus deleted");
    i2c_del_master_bus(bus_handle);
    bus_handle = nullptr;
    return;
}

esp_err_t I2c::rm_device(i2c_master_dev_handle_t &dev_handle)
{
    return i2c_master_bus_rm_device(dev_handle);
}

esp_err_t I2c::transmit(i2c_master_dev_handle_t &dev_handle, uint8_t reg, uint8_t value)
{
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_transmit(dev_handle, write_buf, 2, -1);
}

// I2C read register helper
esp_err_t I2c::receive(i2c_master_dev_handle_t &dev_handle, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg, 1, data, len, -1);
}
#include "bmp280.hpp"
#include <string>

// static const char *TAG = "Bmp280";
static const std::string TAG = "Bmp280";

// I2C write register helper
esp_err_t Bmp280::write_reg(uint8_t reg, uint8_t value)
{
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_transmit(dev_handle, write_buf, 2, -1);
}

// I2C read register helper
esp_err_t Bmp280::read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg, 1, data, len, -1);
}

// Read calibration data from BMP280
esp_err_t Bmp280::read_calibration()
{
    uint8_t calib_data_raw[24];
    esp_err_t err = ESP_FAIL;

    // Retry up to 3 times if read fails
    for (int attempt = 0; attempt < 3; attempt++)
    {
        err = read_reg(BMP280_REG_CALIB_START, calib_data_raw, 24);
        if (err == ESP_OK)
        {
            break;
        }
        ESP_LOGW(TAG.c_str(), "Calibration read attempt %d failed", attempt + 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG.c_str(), "Failed to read calibration after 3 attempts");
        logger.error(TAG, "Failed to read calibration after 3 attempts");
        return err;
    }

    // Parse calibration data (little-endian)
    calib_data.dig_T1 = (calib_data_raw[1] << 8) | calib_data_raw[0];
    calib_data.dig_T2 = (calib_data_raw[3] << 8) | calib_data_raw[2];
    calib_data.dig_T3 = (calib_data_raw[5] << 8) | calib_data_raw[4];
    calib_data.dig_P1 = (calib_data_raw[7] << 8) | calib_data_raw[6];
    calib_data.dig_P2 = (calib_data_raw[9] << 8) | calib_data_raw[8];
    calib_data.dig_P3 = (calib_data_raw[11] << 8) | calib_data_raw[10];
    calib_data.dig_P4 = (calib_data_raw[13] << 8) | calib_data_raw[12];
    calib_data.dig_P5 = (calib_data_raw[15] << 8) | calib_data_raw[14];
    calib_data.dig_P6 = (calib_data_raw[17] << 8) | calib_data_raw[16];
    calib_data.dig_P7 = (calib_data_raw[19] << 8) | calib_data_raw[18];
    calib_data.dig_P8 = (calib_data_raw[21] << 8) | calib_data_raw[20];
    calib_data.dig_P9 = (calib_data_raw[23] << 8) | calib_data_raw[22];

    return ESP_OK;
}

// Read raw temperature and pressure values
esp_err_t Bmp280::read_raw(int32_t *raw_temp, int32_t *raw_press)
{
    // Trigger forced mode measurement
    esp_err_t err = write_reg(BMP280_REG_CTRL_MEAS, 0x56); // osrs_t=010, osrs_p=101, mode=10
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG.c_str(), "Failed to trigger forced mode");
        logger.error(TAG, "Failed to trigger forced mode");
        return err;
    }

    // Poll status register until measurement is complete
    // Status bit 3 (measuring) should be 0 when done
    uint8_t status;
    int timeout = 100; // 100 attempts with 1ms delays = ~100ms max

    while (timeout > 0)
    {
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay between polls
        err = read_reg(BMP280_REG_STATUS, &status, 1);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG.c_str(), "Failed to read status register");
            logger.error(TAG, "Failed to read status register");

            return err;
        }

        // Bit 3 is the 'measuring' bit - 0 means measurement complete
        if ((status & 0x08) == 0)
        {
            break; // Measurement complete
        }
        timeout--;
    }

    if (timeout == 0)
    {
        ESP_LOGW(TAG.c_str(), "Measurement timeout - status: 0x%02x", status);
        logger.error(TAG, "Measurement timeout - status: 0x%02x", status);

        // Continue anyway - data might still be valid
    }

    // Read the measurement data
    uint8_t data[6];
    err = read_reg(BMP280_REG_PRESS_MSB, data, 6);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG.c_str(), "Failed to read measurement data");
        logger.error(TAG, "Failed to read measurement data");
        return err;
    }

    // Combine the 20-bit values
    *raw_press = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4);
    *raw_temp = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((uint32_t)data[5] >> 4);

    if (*raw_temp == 0 || *raw_temp == 0x80000 || *raw_press == 0 || *raw_press == 0x80000)
    {
        ESP_LOGW(TAG.c_str(), "Invalid raw values - temp: 0x%x, press: 0x%x", *raw_temp, *raw_press);
        logger.warning(TAG, "Invalid raw values - temp: 0x%x, press: 0x%x", *raw_temp, *raw_press);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

// Compensate temperature and pressure using calibration data
void Bmp280::compensate_temp_press(int32_t raw_temp, int32_t raw_press,
                                   double *temperature, double *pressure)
{
    // Temperature compensation (from BMP280 datasheet)
    int32_t var1, var2, t_fine;
    var1 = ((((raw_temp >> 3) - ((int32_t)calib_data.dig_T1 << 1))) * ((int32_t)calib_data.dig_T2)) >> 11;
    var2 = (((((raw_temp >> 4) - ((int32_t)calib_data.dig_T1)) *
              ((raw_temp >> 4) - ((int32_t)calib_data.dig_T1))) >>
             12) *
            ((int32_t)calib_data.dig_T3)) >>
           14;
    t_fine = var1 + var2;
    *temperature = (t_fine * 5 + 128) / 25600.0f;

    // Pressure compensation (from BMP280 datasheet)
    int64_t var1_64, var2_64, p_64;
    var1_64 = ((int64_t)t_fine) - 128000;
    var2_64 = var1_64 * var1_64 * (int64_t)calib_data.dig_P6;
    var2_64 = var2_64 + ((var1_64 * (int64_t)calib_data.dig_P5) << 17);
    var2_64 = var2_64 + (((int64_t)calib_data.dig_P4) << 35);
    var1_64 = ((var1_64 * var1_64 * (int64_t)calib_data.dig_P3) >> 8) +
              ((var1_64 * (int64_t)calib_data.dig_P2) << 12);
    var1_64 = (((((int64_t)1) << 47) + var1_64)) * ((int64_t)calib_data.dig_P1) >> 33;

    if (var1_64 == 0)
    {
        *pressure = 0; // Avoid division by zero
        return;
    }

    p_64 = 1048576 - raw_press;
    p_64 = (((p_64 << 31) - var2_64) * 3125) / var1_64;
    var1_64 = (((int64_t)calib_data.dig_P9) * (p_64 >> 13) * (p_64 >> 13)) >> 25;
    var2_64 = (((int64_t)calib_data.dig_P8) * p_64) >> 19;
    p_64 = ((p_64 + var1_64 + var2_64) >> 8) + (((int64_t)calib_data.dig_P7) << 4);
    *pressure = p_64 / 256.0f;
}

void Bmp280::init()
{
    esp_err_t err;

    // Configure I2C master bus
    i2c_master_bus_config_t bus_config = {};
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.scl_io_num = I2C_SCL_PIN;
    bus_config.sda_io_num = I2C_SDA_PIN;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    err = i2c_new_master_bus(&bus_config, &i2c_bus_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG.c_str(), "Failed to create I2C master bus: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG.c_str(), "I2C master bus created successfully");

    uint8_t chip_id;
    bool found = false;

    // Expect bmp280 on address 0x76 - SDO pulled to GND
    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = BMP280_I2C_ADDRESS_0;
    dev_config.scl_speed_hz = I2C_MASTER_FREQ_HZ;

    err = i2c_master_bus_add_device(i2c_bus_handle, &dev_config, &dev_handle);
    if (err == ESP_OK)
    {
        err = read_reg(BMP280_REG_ID, &chip_id, 1);
        if (err == ESP_OK && (chip_id == BMP280_CHIP_ID || chip_id == BME280_CHIP_ID))
        {
            i2c_dev_addr = BMP280_I2C_ADDRESS_0;
            found = true;
            ESP_LOGI(TAG.c_str(), "Found %s at address 0x%02x (chip ID: 0x%02x)",
                     chip_id == BME280_CHIP_ID ? "BME280" : "BMP280",
                     i2c_dev_addr, chip_id);
        }
        else
        {
            i2c_master_bus_rm_device(dev_handle);
        }
    }

    // Try address 0x77 if not found at 0x76
    // if (!found)
    // {
    //     ESP_LOGW(TAG, "BMP280 not found at 0x76, trying 0x77...");
    //     dev_config.device_address = BMP280_I2C_ADDRESS_1;

    //     err = i2c_master_bus_add_device(i2c_bus_handle, &dev_config, &bmp280_dev_handle);
    //     if (err == ESP_OK)
    //     {
    //         err = bmp280_read_reg(BMP280_REG_ID, &chip_id, 1);
    //         if (err == ESP_OK && (chip_id == BMP280_CHIP_ID || chip_id == BME280_CHIP_ID))
    //         {
    //             bmp280_addr = BMP280_I2C_ADDRESS_1;
    //             found = true;
    //             ESP_LOGI(TAG, "Found %s at address 0x%02x (chip ID: 0x%02x)",
    //                      chip_id == BME280_CHIP_ID ? "BME280" : "BMP280",
    //                      bmp280_addr, chip_id);
    //         }
    //     }
    // }

    if (!found)
    {
        ESP_LOGE(TAG.c_str(), "BMP280/BME280 not found at either 0x76 or 0x77");
        i2c_del_master_bus(i2c_bus_handle);
        i2c_bus_handle = nullptr;
        return;
    }

    // Reset the sensor
    err = write_reg(BMP280_REG_RESET, 0xB6);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG.c_str(), "Failed to reset BMP280");
        return;
    }

    // Wait for sensor to complete reset - datasheet specifies startup time
    vTaskDelay(pdMS_TO_TICKS(100)); // Increased delay for reliable reset

    // Verify sensor is responsive after reset
    uint8_t status;
    err = read_reg(BMP280_REG_STATUS, &status, 1);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG.c_str(), "Sensor not responding after reset");
        return;
    }
    ESP_LOGI(TAG.c_str(), "Sensor status after reset: 0x%02x", status);

    // Read calibration data
    err = read_calibration();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG.c_str(), "Failed to read calibration data");
        return;
    }
    ESP_LOGI(TAG.c_str(), "Calibration data read successfully");
    if (calib_data.dig_T1 == 0 || calib_data.dig_P1 == 0)
    {
        ESP_LOGE(TAG.c_str(), "Invalid calibration data - T1: %u, P1: %u",
                 calib_data.dig_T1, calib_data.dig_P1);
        return;
    }
    ESP_LOGI(TAG.c_str(), "Calibration: T1=%u, T2=%d, T3=%d, P1=%u",
             calib_data.dig_T1, calib_data.dig_T2, calib_data.dig_T3, calib_data.dig_P1);

    // Configure sensor
    // Set oversampling: temp x2, pressure x16, forced mode
    err = write_reg(BMP280_REG_CTRL_MEAS, 0x56); // osrs_t=010, osrs_p=101, mode=10
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG.c_str(), "Failed to configure CTRL_MEAS");
        return;
    }

    // Set config: standby 500ms, filter off, SPI disabled
    err = write_reg(BMP280_REG_CONFIG, 0x00);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG.c_str(), "Failed to configure CONFIG");
        return;
    }

    is_initialized = true;
    ESP_LOGI(TAG.c_str(), "BMP280 initialization complete");
}
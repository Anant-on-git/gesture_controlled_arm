#include "mpu6050.h"

#include <inttypes.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MPU6050_I2C_PORT I2C_NUM_0
#define MPU6050_I2C_TIMEOUT_MS 100

#define MPU6050_I2C_ADDRESS 0x68
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_WHO_AM_I 0x75
#define MPU6050_WHO_AM_I_VALUE 0x68

#define MPU6050_WAKE_VALUE 0x00
#define MPU6050_ACCEL_SCALE_LSB_PER_G 16384.0f
#define MPU6050_GYRO_SCALE_LSB_PER_DEG_S 131.0f
#define MPU6050_GRAVITY_M_S2 9.80665f
#define MPU6050_DEG_TO_RAD (float)(M_PI / 180.0)

static const char *TAG = "mpu6050";

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_mpu6050_dev = NULL;
static bool s_mpu6050_initialized = false;

static esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    if (s_mpu6050_dev == NULL || data == NULL || len == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_transmit_receive(
        s_mpu6050_dev,
        &reg_addr,
        sizeof(reg_addr),
        data,
        len,
        MPU6050_I2C_TIMEOUT_MS
    );
}

static esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t value)
{
    if (s_mpu6050_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t write_buffer[2] = {reg_addr, value};
    return i2c_master_transmit(
        s_mpu6050_dev,
        write_buffer,
        sizeof(write_buffer),
        MPU6050_I2C_TIMEOUT_MS
    );
}

static int16_t mpu6050_parse_int16(uint8_t msb, uint8_t lsb)
{
    return (int16_t)((msb << 8) | lsb);
}

esp_err_t mpu6050_init(const mpu6050_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_mpu6050_initialized) {
        ESP_LOGI(TAG, "MPU6050 already initialized");
        return ESP_OK;
    }

    ESP_LOGI(
        TAG,
        "Starting MPU6050 init on I2C port %d, SDA=%d, SCL=%d, clk=%" PRIu32,
        MPU6050_I2C_PORT,
        config->sda_gpio,
        config->scl_gpio,
        config->i2c_clock_hz
    );

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = MPU6050_I2C_PORT,
        .sda_io_num = config->sda_gpio,
        .scl_io_num = config->scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "I2C master bus created");

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_I2C_ADDRESS,
        .scl_speed_hz = config->i2c_clock_hz,
        .scl_wait_us = 0,
    };

    err = i2c_master_bus_add_device(s_i2c_bus, &device_config, &s_mpu6050_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
        (void)i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
        return err;
    }
    ESP_LOGI(TAG, "MPU6050 device added at address 0x%02X", MPU6050_I2C_ADDRESS);

    err = i2c_master_probe(s_i2c_bus, MPU6050_I2C_ADDRESS, MPU6050_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 probe failed: %s", esp_err_to_name(err));
        (void)i2c_master_bus_rm_device(s_mpu6050_dev);
        (void)i2c_del_master_bus(s_i2c_bus);
        s_mpu6050_dev = NULL;
        s_i2c_bus = NULL;
        return err;
    }
    ESP_LOGI(TAG, "MPU6050 probe succeeded");

    uint8_t who_am_i = 0;
    err = mpu6050_register_read(MPU6050_REG_WHO_AM_I, &who_am_i, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read failed: %s", esp_err_to_name(err));
        (void)i2c_master_bus_rm_device(s_mpu6050_dev);
        (void)i2c_del_master_bus(s_i2c_bus);
        s_mpu6050_dev = NULL;
        s_i2c_bus = NULL;
        return err;
    }
    ESP_LOGI(TAG, "WHO_AM_I read returned 0x%02X", who_am_i);

    if (who_am_i != MPU6050_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "Unexpected WHO_AM_I value: 0x%02X", who_am_i);
        (void)i2c_master_bus_rm_device(s_mpu6050_dev);
        (void)i2c_del_master_bus(s_i2c_bus);
        s_mpu6050_dev = NULL;
        s_i2c_bus = NULL;
        return ESP_ERR_INVALID_RESPONSE;
    }

    err = mpu6050_register_write_byte(MPU6050_REG_PWR_MGMT_1, MPU6050_WAKE_VALUE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake MPU6050: %s", esp_err_to_name(err));
        (void)i2c_master_bus_rm_device(s_mpu6050_dev);
        (void)i2c_del_master_bus(s_i2c_bus);
        s_mpu6050_dev = NULL;
        s_i2c_bus = NULL;
        return err;
    }
    ESP_LOGI(TAG, "Wake command sent to MPU6050");

    vTaskDelay(pdMS_TO_TICKS(100));
    s_mpu6050_initialized = true;
    ESP_LOGI(TAG, "MPU6050 initialized on SDA=%d SCL=%d", config->sda_gpio, config->scl_gpio);

    return ESP_OK;
}

esp_err_t mpu6050_read(imu_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_mpu6050_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t raw_data[14] = {0};
    esp_err_t err = mpu6050_register_read(MPU6050_REG_ACCEL_XOUT_H, raw_data, sizeof(raw_data));
    if (err != ESP_OK) {
        return err;
    }

    const int16_t accel_x_raw = mpu6050_parse_int16(raw_data[0], raw_data[1]);
    const int16_t accel_y_raw = mpu6050_parse_int16(raw_data[2], raw_data[3]);
    const int16_t accel_z_raw = mpu6050_parse_int16(raw_data[4], raw_data[5]);
    const int16_t gyro_x_raw = mpu6050_parse_int16(raw_data[8], raw_data[9]);
    const int16_t gyro_y_raw = mpu6050_parse_int16(raw_data[10], raw_data[11]);
    const int16_t gyro_z_raw = mpu6050_parse_int16(raw_data[12], raw_data[13]);

    data->accel_x = ((float)accel_x_raw / MPU6050_ACCEL_SCALE_LSB_PER_G) * MPU6050_GRAVITY_M_S2;
    data->accel_y = ((float)accel_y_raw / MPU6050_ACCEL_SCALE_LSB_PER_G) * MPU6050_GRAVITY_M_S2;
    data->accel_z = ((float)accel_z_raw / MPU6050_ACCEL_SCALE_LSB_PER_G) * MPU6050_GRAVITY_M_S2;
    data->gyro_x = ((float)gyro_x_raw / MPU6050_GYRO_SCALE_LSB_PER_DEG_S) * MPU6050_DEG_TO_RAD;
    data->gyro_y = ((float)gyro_y_raw / MPU6050_GYRO_SCALE_LSB_PER_DEG_S) * MPU6050_DEG_TO_RAD;
    data->gyro_z = ((float)gyro_z_raw / MPU6050_GYRO_SCALE_LSB_PER_DEG_S) * MPU6050_DEG_TO_RAD;

    return ESP_OK;
}

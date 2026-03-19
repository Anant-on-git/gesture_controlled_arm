#include "mpu6050.h"

#include <stddef.h>

esp_err_t mpu6050_init(const mpu6050_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t mpu6050_read(imu_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    data->accel_x = 0.0f;
    data->accel_y = 0.0f;
    data->accel_z = 0.0f;
    data->gyro_x = 0.0f;
    data->gyro_y = 0.0f;
    data->gyro_z = 0.0f;

    return ESP_OK;
}

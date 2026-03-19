#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

#include "common.h"
#include "esp_err.h"

typedef struct {
    int sda_gpio;
    int scl_gpio;
    uint32_t i2c_clock_hz;
} mpu6050_config_t;

esp_err_t mpu6050_init(const mpu6050_config_t *config);
esp_err_t mpu6050_read(imu_data_t *data);

#endif

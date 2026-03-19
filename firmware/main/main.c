#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "mpu6050.h"
#include "serial_comm.h"

#define MPU6050_SDA_GPIO 8
#define MPU6050_SCL_GPIO 9
#define MPU6050_I2C_CLOCK_HZ 400000U

#define SERIAL_UART_PORT 1
#define SERIAL_TX_GPIO 43
#define SERIAL_RX_GPIO 44
#define SERIAL_BAUD_RATE 115200
#define STREAM_PERIOD_MS 100

static const char *TAG = "main";

void app_main(void)
{
    const mpu6050_config_t mpu_config = {
        .sda_gpio = MPU6050_SDA_GPIO,
        .scl_gpio = MPU6050_SCL_GPIO,
        .i2c_clock_hz = MPU6050_I2C_CLOCK_HZ,
    };

    const serial_comm_config_t serial_config = {
        .uart_port = SERIAL_UART_PORT,
        .tx_gpio = SERIAL_TX_GPIO,
        .rx_gpio = SERIAL_RX_GPIO,
        .baud_rate = SERIAL_BAUD_RATE,
    };

    imu_data_t imu_data = {0};

    ESP_ERROR_CHECK(serial_comm_init(&serial_config));

    const esp_err_t mpu_err = mpu6050_init(&mpu_config);
    if (mpu_err != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init failed: %s", esp_err_to_name(mpu_err));
    }

    while (1) {
        if (mpu6050_read(&imu_data) == ESP_OK) {
            (void)serial_comm_send_imu_data(&imu_data);
        }

        vTaskDelay(pdMS_TO_TICKS(STREAM_PERIOD_MS));
    }
}

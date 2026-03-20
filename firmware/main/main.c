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
    uint32_t loop_count = 0;

    printf("BOOT: app_main entered\n");
    fflush(stdout);
    ESP_LOGI(TAG, "Firmware startup");

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

    printf("BOOT: initializing serial\n");
    fflush(stdout);
    ESP_ERROR_CHECK(serial_comm_init(&serial_config));
    printf("BOOT: serial init complete\n");
    fflush(stdout);

    printf("BOOT: initializing mpu6050\n");
    fflush(stdout);
    const esp_err_t mpu_err = mpu6050_init(&mpu_config);
    if (mpu_err != ESP_OK) {
        printf("BOOT: mpu6050 init failed: %s\n", esp_err_to_name(mpu_err));
        fflush(stdout);
        ESP_LOGE(TAG, "MPU6050 init failed: %s", esp_err_to_name(mpu_err));
    } else {
        printf("BOOT: mpu6050 init complete\n");
        fflush(stdout);
    }

    while (1) {
        const esp_err_t read_err = mpu6050_read(&imu_data);
        if (read_err == ESP_OK) {
            if ((loop_count % 20U) == 0U) {
                printf(
                    "LOOP: sample %.3f %.3f %.3f %.3f %.3f %.3f\n",
                    imu_data.accel_x,
                    imu_data.accel_y,
                    imu_data.accel_z,
                    imu_data.gyro_x,
                    imu_data.gyro_y,
                    imu_data.gyro_z
                );
                fflush(stdout);
            }

            (void)serial_comm_send_imu_data(&imu_data);
        } else if ((loop_count % 20U) == 0U) {
            printf("LOOP: mpu6050_read failed: %s\n", esp_err_to_name(read_err));
            fflush(stdout);
        }

        loop_count++;
        vTaskDelay(pdMS_TO_TICKS(STREAM_PERIOD_MS));
    }
}

#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include <stddef.h>

#include "common.h"
#include "esp_err.h"

typedef struct {
    int uart_port;
    int tx_gpio;
    int rx_gpio;
    int baud_rate;
} serial_comm_config_t;

esp_err_t serial_comm_init(const serial_comm_config_t *config);
esp_err_t serial_comm_send_imu_data(const imu_data_t *data);
esp_err_t serial_comm_send_gesture(gesture_t gesture);
esp_err_t serial_comm_send_text(const char *message, size_t length);

#endif

#include "serial_comm.h"

#include <stddef.h>
#include <stdio.h>
#define SERIAL_COMM_JSON_BUFFER_SIZE 192

static bool s_serial_comm_initialized = false;

esp_err_t serial_comm_init(const serial_comm_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    (void)config;
    s_serial_comm_initialized = true;

    return ESP_OK;
}

esp_err_t serial_comm_send_imu_data(const imu_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_serial_comm_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    char json_packet[SERIAL_COMM_JSON_BUFFER_SIZE];
    const int written = snprintf(
        json_packet,
        sizeof(json_packet),
        "{\"accel_x\":%.3f,\"accel_y\":%.3f,\"accel_z\":%.3f,"
        "\"gyro_x\":%.3f,\"gyro_y\":%.3f,\"gyro_z\":%.3f}\n",
        data->accel_x,
        data->accel_y,
        data->accel_z,
        data->gyro_x,
        data->gyro_y,
        data->gyro_z
    );

    if (written < 0 || written >= (int)sizeof(json_packet)) {
        return ESP_ERR_INVALID_SIZE;
    }

    return serial_comm_send_text(json_packet, (size_t)written);
}

esp_err_t serial_comm_send_gesture(gesture_t gesture)
{
    (void)gesture;

    if (!s_serial_comm_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const char *gesture_name = "NONE";

    switch (gesture) {
    case GESTURE_UP:
        gesture_name = "UP";
        break;
    case GESTURE_DOWN:
        gesture_name = "DOWN";
        break;
    case GESTURE_LEFT:
        gesture_name = "LEFT";
        break;
    case GESTURE_RIGHT:
        gesture_name = "RIGHT";
        break;
    case GESTURE_GRAB:
        gesture_name = "GRAB";
        break;
    case GESTURE_RELEASE:
        gesture_name = "RELEASE";
        break;
    case GESTURE_NONE:
    default:
        break;
    }

    char json_packet[48];
    const int written = snprintf(
        json_packet,
        sizeof(json_packet),
        "{\"gesture\":\"%s\"}\n",
        gesture_name
    );

    if (written < 0 || written >= (int)sizeof(json_packet)) {
        return ESP_ERR_INVALID_SIZE;
    }

    return serial_comm_send_text(json_packet, (size_t)written);
}

esp_err_t serial_comm_send_text(const char *message, size_t length)
{
    if (message == NULL && length > 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_serial_comm_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const size_t bytes_written = fwrite(message, 1U, length, stdout);
    if (bytes_written != length) {
        return ESP_FAIL;
    }

    if (fflush(stdout) != 0) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

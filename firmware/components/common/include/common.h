#ifndef COMMON_H
#define COMMON_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} imu_data_t;

typedef enum {
    GESTURE_NONE = 0,
    GESTURE_UP,
    GESTURE_DOWN,
    GESTURE_LEFT,
    GESTURE_RIGHT,
    GESTURE_GRAB,
    GESTURE_RELEASE
} gesture_t;

bool common_is_valid_imu_data(const imu_data_t *data);

#endif

idf_component_register(
    SRCS "mpu6050.c"
    INCLUDE_DIRS "include"
    REQUIRES common
    PRIV_REQUIRES esp_driver_i2c esp_timer freertos
)

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES common mpu6050 serial_comm
)

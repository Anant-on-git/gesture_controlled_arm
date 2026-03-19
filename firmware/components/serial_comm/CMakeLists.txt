idf_component_register(
    SRCS "serial_comm.c"
    INCLUDE_DIRS "include"
    REQUIRES common
    PRIV_REQUIRES esp_driver_uart
)

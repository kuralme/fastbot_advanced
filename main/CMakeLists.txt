idf_component_register(
    SRCS "main.c" "esp32_serial_transport.c" "odometry.c" "motor_driver.c"
    INCLUDE_DIRS "."
    REQUIRES micro_ros_espidf_component
    PRIV_REQUIRES esp_driver_uart esp_driver_gpio esp_driver_ledc
)
idf_component_register(SRCS "main.c" "serial_transport.c"
    INCLUDE_DIRS "." REQUIRES micro_ros_espidf_component car_motion espressif__pid_ctrl espressif__bdc_motor esp_timer driver esp_adc icm42670p)

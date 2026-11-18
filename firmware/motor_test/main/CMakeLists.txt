idf_component_register(SRCS "main.c" "../../chassis/main/serial_transport.c"
    INCLUDE_DIRS "."
    REQUIRES micro_ros_espidf_component encoder pwm_motor esp_timer)

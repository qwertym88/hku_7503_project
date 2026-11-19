idf_component_register(SRCS "lifter.c" "lifter_control.c" INCLUDE_DIRS "."
    REQUIRES driver esp_timer motor)

cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../chassis/components/pwm_motor"
    "${CMAKE_CURRENT_LIST_DIR}/../chassis/components/espressif__bdc_motor")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(servo_zero)

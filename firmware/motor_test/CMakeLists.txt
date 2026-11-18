cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../.ref/micro_ros_espidf_component"
    "${CMAKE_CURRENT_LIST_DIR}/../chassis/components/encoder"
    "${CMAKE_CURRENT_LIST_DIR}/../chassis/components/pwm_motor"
    "${CMAKE_CURRENT_LIST_DIR}/../chassis/components/espressif__bdc_motor")
set(BUILD_WLAN_INTERFACE OFF CACHE BOOL "USB serial only" FORCE)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(motor_test)

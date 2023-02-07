cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(gauge C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(gauge)
target_sources(gauge PRIVATE gauge.c)
target_link_libraries(gauge PRIVATE pico_stdlib hardware_spi hardware_pwm)

pico_add_extra_outputs(gauge)

cmake_minimum_required(VERSION 3.13)

add_library(VL53L1X STATIC
    VL53L1X.cpp
)

# Automatically expose headers
target_include_directories(VL53L1X INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

# Link Pico SDK components so it can find pico/stdlib.h
target_link_libraries(VL53L1X PUBLIC pico_stdlib hardware_i2c)

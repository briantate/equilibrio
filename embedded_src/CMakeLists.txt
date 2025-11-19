cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(balancing_ball_zephyr)
target_sources(app PRIVATE 
                src/main.c
                src/resistive_touch_sensor.c
                src/pid.c
                )

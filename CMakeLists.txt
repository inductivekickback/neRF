# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
add_compile_definitions(NRFX_GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS=3)
include($ENV{ZEPHYR_BASE}/../nrf/cmake/boilerplate.cmake)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(nerf)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

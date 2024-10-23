/**
 * @file gpio.hpp
 * @author Nilusink
 * @brief helper functions for GPIO pins
 * @version 0.1
 * @date 2024-10-23
 * 
 */
#pragma once

#include <gpiod.h>

#define A 0
#define B 8
#define C 16
#define D 4

typedef gpiod_line* pin_t;

/**
 * @brief create a gpio line and set it to input / output
 * 
 * @param chip_name chip device
 * @param pin pin offset
 * @param is_input true: input, false: output
 */
gpiod_line* get_pin(
    gpiod_chip* chip,
    unsigned int pin,
    bool is_input,
    bool default_value = false
);

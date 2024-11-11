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

// typedef struct {
//     gpiod_chip* chip_name;
//     gpiod_line* pin_number;
// } pin_t;

typedef gpiod_line* pin_t;

/**
 * @brief get a gpio chip
 * 
 * @param chip_name normally "gpiochipX"
 */
gpiod_chip* get_chip(const char* chip_name);

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

/**
 * @brief set a pins satate
 * 
 * @returns 0 if success
 *
 */
int pin_write(pin_t pin, bool value);

/**
 * @brief read a pins state
 * 
 */
bool pin_read(pin_t pin);

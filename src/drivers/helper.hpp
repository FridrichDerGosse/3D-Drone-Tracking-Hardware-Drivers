/**
* @file helper.hpp
 * @author Nilusink
 * @brief helper functions / types
 * @version 1.0
 * @date 2024-10-23
 *
 */
#pragma once
#include <gpiod.h>
#include <unistd.h>

// macros
// constants
#define MS 1000
#define S  1000000

#define sleep(T) usleep(T*S)
#define msleep(T) usleep(T*MS)

// types
typedef short int int16_t;
typedef unsigned short int uint16_t;
typedef unsigned char uint8_t;

// math functions
double map(double value, double in_min, double in_max, double out_min, double out_max);

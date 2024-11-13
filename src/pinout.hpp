/**
 * @file pinout.hpp
 * @author Nilusink
 * @brief pinout configuration
 * @version 1.0
 * @date 2024-11-10
 *
 */
#pragma once

// Armsom GPIO naming convention
#define A 0
#define B 8
#define C 16
#define D 24


// steppers
#define YAW_CHIP "gpiochip4"
#define YAW_IN1 B+3
#define YAW_IN2 B+2
#define YAW_IN3 C+4
#define YAW_IN4 C+6

#define PITCH_CHIP "gpiochip3"
#define PITCH_LEFT_IN1 B+5
#define PITCH_LEFT_IN2 A+4
#define PITCH_LEFT_IN3 C+2
#define PITCH_LEFT_IN4 A+7

#define PITCH_RIGHT_IN1 B+6
#define PITCH_RIGHT_IN2 B+1
#define PITCH_RIGHT_IN3 B+2
#define PITCH_RIGHT_IN4 B+3

// end-switches
#define END_SWITCH_LR A+0

#define END_SWITCH_UP C+3
#define END_SWITCH_DOWN C+1

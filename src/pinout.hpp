#pragma once

#include "drivers/gpio.hpp"

// steppers
#define STEPPER0_CHIP "gpiochip3"
#define STEPPER0_IN1 B+3
#define STEPPER0_IN2 B+2
#define STEPPER0_IN3 B+1
#define STEPPER0_IN4 B+6

#define STEPPER1_CHIP "gpiochip?"
#define STEPPER1_IN1 0
#define STEPPER1_IN2 0
#define STEPPER1_IN3 0
#define STEPPER1_IN4 0

#define STEPPER2_CHIP "gpiochip?"
#define STEPPER2_IN1 0
#define STEPPER2_IN2 0
#define STEPPER2_IN3 0
#define STEPPER2_IN4 0

// end-switches
#define END_SWITCH_LEFT A+7
#define END_SWITCH_RIGHT C+2

#define END_SWITCH_UP 0
#define END_SWITCH_DOWN 0

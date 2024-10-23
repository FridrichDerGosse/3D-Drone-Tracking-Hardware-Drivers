#include "stepper.hpp"

#include <iostream>
#include <ostream>
#include <unistd.h>
#include <bitset>
#include <algorithm>

using namespace stepper;

const int stepper::sequence[8] = {
    0b1001,
    0b1000,
    0b1100,
    0b0100,
    0b0110,
    0b0010,
    0b0011,
    0b0001
};


// Basic stepper
void Base::update_pins() const
{
    std::bitset<4> tmp(sequence[current_step % sequence_size]);
    // std::cout << "updating pins: " << tmp << std::endl;

    // update all pins corresponding to the current sequence position
    gpiod_line_set_value(pins.pin_a, sequence[current_step % sequence_size] & 0b0001);
    gpiod_line_set_value(pins.pin_b, sequence[current_step % sequence_size] & 0b0010);
    gpiod_line_set_value(pins.pin_c, sequence[current_step % sequence_size] & 0b0100);
    gpiod_line_set_value(pins.pin_d, sequence[current_step % sequence_size] & 0b1000);
}

Base::Base(const stepper_pinout_t pins)
    : pins(pins)
{
    // turn motor on
    update_pins();
}

int16_t Base::get_current_step() const
{
    return current_step;
}

void Base::set_speed(uint8_t speed)
{
    step_delay_us = map(std::min((int)speed, 250), 0, 255, 10000, 400);
}

void Base::move_steps(const signed short int n) {
    // step in one direction
    for (signed short int i = 0; i < n; i++) {
        current_step++;

        update_pins();

        // stepper speed
        usleep(step_delay_us);
    };

    // step in the other direction
    for (signed short int i = 0; i > n; i--) {
        current_step--;

        update_pins();

        // stepper speed
        usleep(step_delay_us);
    };
};

void Base::off() const
{
    gpiod_line_set_value(pins.pin_a, false);
    gpiod_line_set_value(pins.pin_b, false);
    gpiod_line_set_value(pins.pin_c, false);
    gpiod_line_set_value(pins.pin_d, false);
};


// Horizontal Stepper
bool Horizontal::check_calibrated() const
{
    return n_steps != 0;
};

Horizontal::Horizontal(
    const stepper_pinout_t pins,
    const pin_t end_left,
    const pin_t end_right
)
    : Base(pins), end_left_pin(end_left), end_right_pin(end_right)
{};

double Horizontal::get_current_angle() const
{
    return map(
        get_current_step(),
        max_step_right,
        max_step_left,
        -angle_size / 2,
        angle_size / 2
    );
};

uint8_t Horizontal::calibrate() {
    // set n_steps to be able to move the motor
    n_steps = Base::steps_per_rev;
    max_step_left = Base::steps_per_rev;
    max_step_right = -Base::steps_per_rev;

    // move left until hitting the end switch
    while (!gpiod_line_get_value(end_left_pin))
        if (move_steps(1) > 0)
            std::cout << "error aligning: stepper can't find end switch" << std::endl;

    max_step_left = get_current_step();

    // move right until hitting the end switch
    while (!gpiod_line_get_value(end_right_pin))
        if (move_steps(-1) > 0)
            std::cout << "error aligning: stepper can't find end switch" << std::endl;

    max_step_right = get_current_step();

    // calculate fancy stuff
    n_steps = max_step_left - max_step_right;

    // move to center
    move_absolute_angle(0);
}

uint8_t Horizontal::move_steps(int16_t n) {
    if (!check_calibrated())
    {
        std::cout << "can't move, not calibrated yet!" << std::endl;
        return 1;
    }

    if (get_current_step() + n > max_step_left)
    {
        std::cout << "steps out of moveable range, moving to max left" << std::endl;
        n = max_step_left - get_current_step();
        return 2;
    }

    if (get_current_step() + n < max_step_right)
    {
        std::cout << "steps out of moveable range, moving to max right" << std::endl;
        n = max_step_right - get_current_step();
        return 3;
    }

    Base::move_steps(n);
    return 0;
};

void Horizontal::move_relative_angle(const double angle_delta)
{
    move_steps(map(angle_delta, 0, angle_size, 0, n_steps));
};

void Horizontal::move_absolute_angle(const double angle)
{
    double angle_delta = angle - get_current_angle();
    move_relative_angle(angle_delta);
};


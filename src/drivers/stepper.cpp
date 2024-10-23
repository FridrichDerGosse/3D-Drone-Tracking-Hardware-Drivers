#include "stepper.hpp"

#include <iostream>
#include <unistd.h>
#include <algorithm>

using namespace stepper;

// uln2003 step pattern
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
    // update all pins corresponding to the current sequence position
    gpiod_line_set_value(pins.pin_a, sequence[current_step % sequence_size] & 0b0001);
    gpiod_line_set_value(pins.pin_b, sequence[current_step % sequence_size] & 0b0010);
    gpiod_line_set_value(pins.pin_c, sequence[current_step % sequence_size] & 0b0100);
    gpiod_line_set_value(pins.pin_d, sequence[current_step % sequence_size] & 0b1000);
}

bool Base::can_move() const
{
    return true;
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
    // map speed to delay (max 250 because idk why but it won't work if i don't do this)
    step_delay_us = map(std::min((int)speed, 250), 0, 255, 10000, 400);
}

int8_t Base::move_steps(const signed short int n) {
    // check if movement is allowed
    if (!can_move())
    {
        std::cout << "can't move" << std::endl;
        return 4;
    }

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

    return 0;
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

bool Horizontal::can_move() const
{
    // "override" end-switches for calibration
    if (is_calibrating)
        return true;

    return !gpiod_line_get_value(end_left_pin) and !gpiod_line_get_value(end_right_pin);
}

Horizontal::Horizontal(
    const stepper_pinout_t pins,
    const pin_t end_left,
    const pin_t end_right
)
    : Base(pins), end_left_pin(end_left), end_right_pin(end_right)
{};

double Horizontal::get_current_angle() const
{
    // convert step position to angular position
    return map(
        get_current_step(),
        max_step_right,
        max_step_left,
        -angle_size / 2,
        angle_size / 2
    );
};

int8_t Horizontal::calibrate() {
    // set to calibration mode so the stepper is allowed to move
    is_calibrating = true;

    // set a maximum of one rotation to find the end-switch
    max_step_left = Base::steps_per_rev;
    max_step_right = -Base::steps_per_rev;

    // move left until hitting the end switch
    while (!gpiod_line_get_value(end_left_pin))
    {
        if (move_steps(1) > 0)
        {
            std::cerr << "error aligning: stepper can't find end switch" << std::endl;
            return 1;
        }
        
        if (gpiod_line_get_value(end_right_pin))
        {
            std::cerr << "error aligning: wrong end switch touched - check wiring" << std::endl;
        }
    }


    max_step_left = get_current_step();

    // move right until hitting the end switch
    while (!gpiod_line_get_value(end_right_pin))
    {
        if (move_steps(-1) > 0)
        {
            std::cerr << "error aligning: stepper can't find end switch" << std::endl;
            return 1;
        }
    }

    max_step_right = get_current_step();

    // calculate fancy stuff
    n_steps = max_step_left - max_step_right;
    is_calibrating = false;

    // move to center
    move_absolute_angle(0);
    return 0;
}

int8_t Horizontal::move_steps(int16_t n) {
    if (!check_calibrated() && !is_calibrating)
    {
        std::cerr << "can't move, not calibrated yet!" << std::endl;
        return 1;
    }

    if (get_current_step() + n > max_step_left)
    {
        std::cerr << "steps out of moveable range, moving to max left" << std::endl;
        n = max_step_left - get_current_step();
        return 2;
    }

    if (get_current_step() + n < max_step_right)
    {
        std::cerr << "steps out of moveable range, moving to max right" << std::endl;
        n = max_step_right - get_current_step();
        return 3;
    }

    return Base::move_steps(n);
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

